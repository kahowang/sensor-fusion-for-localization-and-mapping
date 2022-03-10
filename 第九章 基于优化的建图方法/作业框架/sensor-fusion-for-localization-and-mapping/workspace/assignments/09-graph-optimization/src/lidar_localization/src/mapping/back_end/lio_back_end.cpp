/*
 * @Description: LIO mapping backend, implementation
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */

#include "lidar_localization/mapping/back_end/lio_back_end.hpp"

#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/tools/file_manager.hpp"

namespace lidar_localization {

LIOBackEnd::LIOBackEnd() {
    InitWithConfig();
}

bool LIOBackEnd::InitWithConfig() {
    std::string config_file_path = WORK_SPACE_PATH + "/config/mapping/lio_back_end.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::cout << "-----------------Init LIO Backend-------------------" << std::endl;

    InitDataPath(config_node);
    InitParam(config_node);
    InitGraphOptimizer(config_node);
    InitIMUPreIntegrator(config_node);
    InitOdoPreIntegrator(config_node);

    return true;
}

bool LIOBackEnd::InitDataPath(const YAML::Node& config_node) {
    std::string data_path = config_node["data_path"].as<std::string>();
    if (data_path == "./") {
        data_path = WORK_SPACE_PATH;
    }

    if (!FileManager::CreateDirectory(data_path + "/slam_data"))
        return false;

    key_frames_path_ = data_path + "/slam_data/key_frames";
    scan_context_path_ = data_path + "/slam_data/scan_context";
    trajectory_path_ = data_path + "/slam_data/trajectory";

    if (!FileManager::InitDirectory(key_frames_path_, "Point Cloud Key Frames"))
        return false;
    if (!FileManager::InitDirectory(scan_context_path_, "Scan Context Index & Data"))
        return false;
    if (!FileManager::InitDirectory(trajectory_path_, "Estimated Trajectory"))
        return false;

    return true;
}


bool LIOBackEnd::InitParam(const YAML::Node& config_node) {
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();

    return true;
}

bool LIOBackEnd::InitGraphOptimizer(const YAML::Node& config_node) {
    std::string graph_optimizer_type = config_node["graph_optimizer_type"].as<std::string>();

    if (graph_optimizer_type == "g2o") {
        graph_optimizer_ptr_ = std::make_shared<G2oGraphOptimizer>("lm_var");
    } else {
        LOG(ERROR) << "Optimizer " << graph_optimizer_type << " NOT FOUND!";
        return false;
    }
    std::cout << "\tOptimizer:" << graph_optimizer_type << std::endl << std::endl;

    graph_optimizer_config_.use_gnss = config_node["use_gnss"].as<bool>();
    graph_optimizer_config_.use_loop_close = config_node["use_loop_close"].as<bool>();
    graph_optimizer_config_.use_imu_pre_integration = config_node["use_imu_pre_integration"].as<bool>();
    graph_optimizer_config_.use_odo_pre_integration = config_node["use_odo_pre_integration"].as<bool>();

    graph_optimizer_config_.optimization_step_size.key_frame = (
        config_node["optimization_step_size"]["key_frame"].as<int>()
    );
    graph_optimizer_config_.optimization_step_size.loop_closure = (
        config_node["optimization_step_size"]["loop_closure"].as<int>()
    );

    // x-y-z & yaw-roll-pitch
    for (int i = 0; i < 6; ++i) {
        graph_optimizer_config_.odom_edge_noise(i) =
            config_node[graph_optimizer_type + "_param"]["odom_edge_noise"][i].as<double>();
        graph_optimizer_config_.close_loop_noise(i) =
            config_node[graph_optimizer_type + "_param"]["close_loop_noise"][i].as<double>();
    }

    // x-y-z:
    for (int i = 0; i < 3; i++) {
        graph_optimizer_config_.gnss_noise(i) =
            config_node[graph_optimizer_type + "_param"]["gnss_noise"][i].as<double>();
    }

    return true;
}

bool LIOBackEnd::InitIMUPreIntegrator(const YAML::Node& config_node) {
    imu_pre_integrator_ptr_ = nullptr;
    
    if (graph_optimizer_config_.use_imu_pre_integration) {
        imu_pre_integrator_ptr_ = std::make_shared<IMUPreIntegrator>(config_node["imu_pre_integration"]);
    }

    return true;
}

bool LIOBackEnd::InitOdoPreIntegrator(const YAML::Node& config_node) {
    odo_pre_integrator_ptr_ = nullptr;
    
    if (graph_optimizer_config_.use_odo_pre_integration) {
        odo_pre_integrator_ptr_ = std::make_shared<OdoPreIntegrator>(config_node["odo_pre_integration"]);
    }

    return true;
}

bool LIOBackEnd::InsertLoopPose(const LoopPose& loop_pose) {
    if (!graph_optimizer_config_.use_loop_close)
        return false;

    // get vertex IDs:
    const int vertex_index_i = loop_pose.index0;
    const int vertex_index_j = loop_pose.index1;
    // get relative pose measurement:
    Eigen::Matrix4d relative_pose = loop_pose.pose.cast<double>();
    // add constraint, lidar frontend / loop closure detection:
    graph_optimizer_ptr_->AddPRVAGRelativePoseEdge(
        vertex_index_i, vertex_index_j, 
        relative_pose, graph_optimizer_config_.close_loop_noise
    );

    // update loop closure count:
    ++counter_.loop_closure;
    
    LOG(INFO) << "Add loop closure: " << loop_pose.index0 << "," << loop_pose.index1 << std::endl;

    return true;
}

bool LIOBackEnd::UpdateIMUPreIntegration(const IMUData &imu_data) {
    if ( !graph_optimizer_config_.use_imu_pre_integration || nullptr == imu_pre_integrator_ptr_ )
        return false;
    
    if (
        !imu_pre_integrator_ptr_->IsInited() ||
        imu_pre_integrator_ptr_->Update(imu_data)
    ) {
        return true;
    }

    return false;
}

bool LIOBackEnd::UpdateOdoPreIntegration(const VelocityData &velocity_data) {
    if ( !graph_optimizer_config_.use_odo_pre_integration || nullptr == odo_pre_integrator_ptr_ )
        return false;
    
    if (
        !odo_pre_integrator_ptr_->IsInited() ||
        odo_pre_integrator_ptr_->Update(velocity_data)
    ) {
        return true;
    }

    return false;
}

bool LIOBackEnd::Update(
    const CloudData& cloud_data, 
    const PoseData& laser_odom, 
    const PoseData& gnss_pose,
    const IMUData &imu_data
) {
    ResetParam();

    if ( MaybeNewKeyFrame(cloud_data, laser_odom, gnss_pose, imu_data) ) {
        AddNodeAndEdge(gnss_pose);
        MaybeOptimized();
    }

    return true;
}

void LIOBackEnd::ResetParam() {
    has_new_key_frame_ = false;
    has_new_optimized_ = false;
}

bool LIOBackEnd::MaybeNewKeyFrame(
    const CloudData& cloud_data, 
    const PoseData& laser_odom, 
    const PoseData& gnss_odom,
    const IMUData &imu_data
) {
    static int count = 0;
    static VelocityData velocity_data;
    static PoseData last_laser_pose = laser_odom;
    static PoseData last_gnss_pose = gnss_odom;

    if (key_frames_deque_.size() == 0) {
        // init IMU pre-integrator:
        if ( imu_pre_integrator_ptr_ ) {
            imu_pre_integrator_ptr_->Init(imu_data);
        }

        // init odometer pre-integrator:
        if ( odo_pre_integrator_ptr_ ) {
            gnss_odom.GetVelocityData(velocity_data);
            odo_pre_integrator_ptr_->Init(velocity_data);
        }

        last_laser_pose = laser_odom;
        last_gnss_pose = gnss_odom;

        has_new_key_frame_ = true;
    }

    // whether the current scan is far away enough from last key frame:
    if (fabs(laser_odom.pose(0,3) - last_laser_pose.pose(0,3)) + 
        fabs(laser_odom.pose(1,3) - last_laser_pose.pose(1,3)) +
        fabs(laser_odom.pose(2,3) - last_laser_pose.pose(2,3)) > key_frame_distance_) {

        if ( imu_pre_integrator_ptr_ ) {
            imu_pre_integrator_ptr_->Reset(imu_data, imu_pre_integration_); 
        }

        if ( odo_pre_integrator_ptr_ ) {
            gnss_odom.GetVelocityData(velocity_data);
            odo_pre_integrator_ptr_->Reset(velocity_data, odo_pre_integration_); 
        }

        //
        // for IMU pre-integration debugging ONLY:
        // this is critical to IMU pre-integration verification
        //
        if ( 0 == (++count) % 10 ) {
            // display IMU pre-integration:
            // ShowIMUPreIntegrationResidual(last_gnss_pose, gnss_odom, imu_pre_integration_); 

            // display odometer pre-integration:

            // reset counter:
            count = 0;
        }

        last_laser_pose = laser_odom;
        last_gnss_pose = gnss_odom;
    
        has_new_key_frame_ = true;
    }

    // if so:
    if (has_new_key_frame_) {
        // a. first write new key scan to disk:
        std::string file_path = key_frames_path_ + "/key_frame_" + std::to_string(key_frames_deque_.size()) + ".pcd";
        pcl::io::savePCDFileBinary(file_path, *cloud_data.cloud_ptr);
        current_key_scan_.time = cloud_data.time;
        current_key_scan_.cloud_ptr.reset(
            new CloudData::CLOUD(*cloud_data.cloud_ptr)
        );

        current_key_gnss_.time = current_key_frame_.time = laser_odom.time;
        current_key_gnss_.index = current_key_frame_.index = key_frames_deque_.size();

        // b. create key frame index for lidar scan, relative pose measurement:
        current_key_frame_.pose = laser_odom.pose;

        // c. create key frame index for GNSS measurement, full LIO state:
        current_key_gnss_.pose = gnss_odom.pose;
        current_key_gnss_.vel.v = gnss_odom.vel.v;
        current_key_gnss_.vel.w = gnss_odom.vel.w;

        // add to cache for later evo evaluation:
        key_frames_deque_.push_back(current_key_frame_);
        key_gnss_deque_.push_back(current_key_gnss_);
    }

    return has_new_key_frame_;
}

bool LIOBackEnd::AddNodeAndEdge(const PoseData& gnss_data) {
    static KeyFrame last_key_frame_ = current_key_frame_;

    //
    // add node for new key frame pose:
    //
    // fix the pose of the first key frame for lidar only mapping:
    if (!graph_optimizer_config_.use_gnss && graph_optimizer_ptr_->GetNodeNum() == 0) {
        graph_optimizer_ptr_->AddPRVAGNode(current_key_frame_, true);
    } else {
        graph_optimizer_ptr_->AddPRVAGNode(current_key_gnss_, false);
    }

    //
    // add constraints:
    //
    // get num. of vertices:
    const int N = graph_optimizer_ptr_->GetNodeNum();
    // get vertex IDs:
    const int vertex_index_i = N - 2;
    const int vertex_index_j = N - 1;
    // a. lidar frontend / loop closure detection:
    if ( N > 1 ) {
        // get relative pose measurement:
        Eigen::Matrix4d relative_pose = (last_key_frame_.pose.inverse() * current_key_frame_.pose).cast<double>();
        // add constraint, lidar frontend / loop closure detection:
        graph_optimizer_ptr_->AddPRVAGRelativePoseEdge(
            vertex_index_i, vertex_index_j, 
            relative_pose, graph_optimizer_config_.odom_edge_noise
        );
    }

    // b. GNSS position:
    if ( graph_optimizer_config_.use_gnss ) {
        // get prior position measurement:
        Eigen::Vector3d pos = current_key_gnss_.pose.block<3, 1>(0, 3).cast<double>();
        // add constraint, GNSS position:
        graph_optimizer_ptr_->AddPRVAGPriorPosEdge(
            vertex_index_j, 
            pos, graph_optimizer_config_.gnss_noise
        );
    }

    // c. IMU pre-integration:
    if ( N > 1 && graph_optimizer_config_.use_imu_pre_integration ) {
        // add constraint, IMU pre-integraion:
        graph_optimizer_ptr_->AddPRVAGIMUPreIntegrationEdge(
            vertex_index_i, vertex_index_j,
            imu_pre_integration_
        );
    }

    // d. Odo pre-integration:
    if ( N > 1 && graph_optimizer_config_.use_odo_pre_integration ) {
        // add constraint, odo pre-integraion:
        graph_optimizer_ptr_->AddPRVAGOdoPreIntegrationEdge(
            vertex_index_i, vertex_index_j,
            odo_pre_integration_
        );
    }

    // move forward:
    last_key_frame_ = current_key_frame_;
    ++counter_.key_frame;

    return true;
}

bool LIOBackEnd::MaybeOptimized() {
    bool need_optimize = false; 

    if (
        counter_.HasEnoughKeyFrames(
            graph_optimizer_config_.optimization_step_size.key_frame
        ) ||
        counter_.HasEnoughLoopClosures(
            graph_optimizer_config_.optimization_step_size.loop_closure
        )
    ) {
        need_optimize = true;
    }
    
    if ( need_optimize && graph_optimizer_ptr_->Optimize() ) {
        has_new_optimized_ = true;

        return true;
    }
    
    return false;
}

bool LIOBackEnd::SavePose(std::ofstream& ofs, const Eigen::Matrix4f& pose) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            ofs << pose(i, j);
            
            if (i == 2 && j == 3) {
                ofs << std::endl;
            } else {
                ofs << " ";
            }
        }
    }

    return true;
}

bool LIOBackEnd::ForceOptimize() {
    if (graph_optimizer_ptr_->Optimize())
        has_new_optimized_ = true;

    return has_new_optimized_;
}

bool LIOBackEnd::SaveOptimizedPose() {
    static Eigen::Matrix4f current_pose = Eigen::Matrix4f::Identity();

    if (graph_optimizer_ptr_->GetNodeNum() == 0)
        return false;

    if (
        !FileManager::CreateFile(ground_truth_ofs_, trajectory_path_ + "/ground_truth.txt") || 
        !FileManager::CreateFile(laser_odom_ofs_, trajectory_path_ + "/laser_odom.txt") ||
        !FileManager::CreateFile(optimized_pose_ofs_, trajectory_path_ + "/optimized.txt")
    )
        return false;

    graph_optimizer_ptr_->GetOptimizedKeyFrame(optimized_key_frames_);

    // write GNSS/IMU pose and lidar odometry estimation as trajectory for evo evaluation:
    for (size_t i = 0; i < optimized_key_frames_.size(); ++i) {
        // a. ground truth, IMU/GNSS:
        current_pose = key_frames_deque_.at(i).pose;
        current_pose(2, 3) = 0.0f;
        SavePose(ground_truth_ofs_, current_pose);
        // b. lidar odometry:
        current_pose = key_gnss_deque_.at(i).pose;
        current_pose(2, 3) = 0.0f;
        SavePose(laser_odom_ofs_, current_pose);
        // c. optimized odometry:
        current_pose = optimized_key_frames_.at(i).pose;
        current_pose(2, 3) = 0.0f;
        SavePose(optimized_pose_ofs_, current_pose);
    }

    return true;
}

void LIOBackEnd::GetOptimizedKeyFrames(std::deque<KeyFrame>& key_frames_deque) {
    key_frames_deque.clear();

    key_frames_deque.insert(
        key_frames_deque.end(), 
        optimized_key_frames_.begin(), optimized_key_frames_.end()
    );
}

bool LIOBackEnd::HasNewKeyFrame() {
    return has_new_key_frame_;
}

bool LIOBackEnd::HasNewOptimized() {
    return has_new_optimized_;
}

void LIOBackEnd::GetLatestKeyScan(CloudData& key_scan) {
    key_scan.time = current_key_scan_.time;
    key_scan.cloud_ptr.reset(
        new CloudData::CLOUD(*current_key_scan_.cloud_ptr)
    );
}

void LIOBackEnd::GetLatestKeyFrame(KeyFrame& key_frame) {
    key_frame = current_key_frame_;
}

void LIOBackEnd::GetLatestKeyGNSS(KeyFrame& key_frame) {
    key_frame = current_key_gnss_;
}

void LIOBackEnd::ShowIMUPreIntegrationResidual(
    const PoseData &last_gnss_pose, const PoseData& curr_gnss_pose,
    const IMUPreIntegrator::IMUPreIntegration &imu_pre_integration
) {
    const double &T = imu_pre_integration.T_;
    const Eigen::Vector3d &g = imu_pre_integration.g_;

    Eigen::Vector3d r_p = last_gnss_pose.pose.block<3, 3>(0, 0).transpose().cast<double>() * (
        curr_gnss_pose.pose.block<3, 1>(0, 3).cast<double>() - last_gnss_pose.pose.block<3, 1>(0, 3).cast<double>() - 
        ( last_gnss_pose.pose.block<3, 3>(0, 0).cast<double>()*last_gnss_pose.vel.v.cast<double>() - 0.50 * g * T ) * T
    ) - imu_pre_integration.alpha_ij_;

    Sophus::SO3d prev_theta(Eigen::Quaterniond(last_gnss_pose.pose.block<3, 3>(0, 0).cast<double>()));
    Sophus::SO3d curr_theta(Eigen::Quaterniond(curr_gnss_pose.pose.block<3, 3>(0, 0).cast<double>()));
    Eigen::Vector3d r_q = (imu_pre_integration.theta_ij_.inverse()*prev_theta.inverse() * curr_theta).log();

    Eigen::Vector3d r_v = last_gnss_pose.pose.block<3, 3>(0, 0).transpose().cast<double>() * (
        curr_gnss_pose.pose.block<3, 3>(0, 0).cast<double>()*curr_gnss_pose.vel.v.cast<double>() - 
        last_gnss_pose.pose.block<3, 3>(0, 0).cast<double>()*last_gnss_pose.vel.v.cast<double>() + 
        g * T
    ) - imu_pre_integration.beta_ij_;

    LOG(INFO) << "IMU Pre-Integration Measurement: " << std::endl
                << "\tT: " << T << " --- " << curr_gnss_pose.time - last_gnss_pose.time << std::endl
                << "\talpha:" << std::endl
                << "\t\t" << r_p.x() << ", " << r_p.y() << ", " << r_p.z() << std::endl
                << "\ttheta:" << std::endl
                << "\t\t" << r_q.x() << ", " << r_q.y() << ", " << r_q.z() << std::endl
                << "\tbeta:" << std::endl
                << "\t\t" << r_v.x() << ", " << r_v.y() << ", " << r_v.z() << std::endl
                << "\tbias_accel:" 
                << imu_pre_integration.b_a_i_.x() << ", "
                << imu_pre_integration.b_a_i_.y() << ", " 
                << imu_pre_integration.b_a_i_.z()
                << std::endl
                << "\tbias_gyro:" 
                << imu_pre_integration.b_g_i_.x() << ", "
                << imu_pre_integration.b_g_i_.y() << ", " 
                << imu_pre_integration.b_g_i_.z()
                << std::endl
                << "\tcovariance:" << std::endl
                << "\t\talpha: "
                << imu_pre_integration.P_( 0,  0) << ", "
                << imu_pre_integration.P_( 1,  1) << ", " 
                << imu_pre_integration.P_( 2,  3)
                << std::endl
                << "\t\ttheta: "
                << imu_pre_integration.P_( 3,  3) << ", "
                << imu_pre_integration.P_( 4,  4) << ", " 
                << imu_pre_integration.P_( 5,  5)
                << std::endl
                << "\t\tbeta: "
                << imu_pre_integration.P_( 6,  6) << ", "
                << imu_pre_integration.P_( 7,  7) << ", " 
                << imu_pre_integration.P_( 8,  8)
                << std::endl
                << "\t\tbias_accel: "
                << imu_pre_integration.P_( 9,  9) << ", "
                << imu_pre_integration.P_(10, 10) << ", " 
                << imu_pre_integration.P_(11, 11)
                << std::endl
                << "\t\tbias_gyro: "
                << imu_pre_integration.P_(12, 12) << ", "
                << imu_pre_integration.P_(13, 13) << ", " 
                << imu_pre_integration.P_(14, 14)
                << std::endl
                /*
                << "\tJacobian:" << std::endl
                << "\t\td_alpha_d_b_a: " << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 0,  9) << ", " << imu_pre_integration.J_( 0, 10) << ", " << imu_pre_integration.J_( 0, 11) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 1,  9) << ", " << imu_pre_integration.J_( 1, 10) << ", " << imu_pre_integration.J_( 1, 11) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 2,  9) << ", " << imu_pre_integration.J_( 2, 10) << ", " << imu_pre_integration.J_( 2, 11) << std::endl
                << "\t\td_alpha_d_b_g: " << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 0, 12) << ", " << imu_pre_integration.J_( 0, 13) << ", " << imu_pre_integration.J_( 0, 14) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 1, 12) << ", " << imu_pre_integration.J_( 1, 13) << ", " << imu_pre_integration.J_( 1, 14) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 2, 12) << ", " << imu_pre_integration.J_( 2, 13) << ", " << imu_pre_integration.J_( 2, 14) << std::endl
                << "\t\td_theta_d_b_g: " << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 3, 12) << ", " << imu_pre_integration.J_( 3, 13) << ", " << imu_pre_integration.J_( 3, 14) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 4, 12) << ", " << imu_pre_integration.J_( 4, 13) << ", " << imu_pre_integration.J_( 4, 14) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 5, 12) << ", " << imu_pre_integration.J_( 5, 13) << ", " << imu_pre_integration.J_( 5, 14) << std::endl
                << "\t\td_beta_d_b_a: " << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 6,  9) << ", " << imu_pre_integration.J_( 6, 10) << ", " << imu_pre_integration.J_( 6, 11) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 7,  9) << ", " << imu_pre_integration.J_( 7, 10) << ", " << imu_pre_integration.J_( 7, 11) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 8,  9) << ", " << imu_pre_integration.J_( 8, 10) << ", " << imu_pre_integration.J_( 8, 11) << std::endl
                << "\t\td_beta_d_b_a: " << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 6, 12) << ", " << imu_pre_integration.J_( 6, 13) << ", " << imu_pre_integration.J_( 6, 14) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 7, 12) << ", " << imu_pre_integration.J_( 7, 13) << ", " << imu_pre_integration.J_( 7, 14) << std::endl
                << "\t\t\t" << imu_pre_integration.J_( 8, 12) << ", " << imu_pre_integration.J_( 8, 13) << ", " << imu_pre_integration.J_( 8, 14) << std::endl
                */
                << std::endl;
}

} // namespace lidar_localization