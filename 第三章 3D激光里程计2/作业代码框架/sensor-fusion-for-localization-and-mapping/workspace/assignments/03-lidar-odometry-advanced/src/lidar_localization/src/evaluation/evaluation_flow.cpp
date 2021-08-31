/*
 * @Description: evo evaluation facade
 * @Author: Ge Yao
 * @Date: 2021-01-30 22:38:22
 */
#include "lidar_localization/evaluation/evaluation_flow.hpp"

#include "glog/logging.h"

#include "lidar_localization/tools/file_manager.hpp"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

EvaluationFlow::EvaluationFlow(ros::NodeHandle& nh) {
    std::string config_file_path = WORK_SPACE_PATH + "/config/dataset/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    InitSubscribers(nh, config_node["measurements"]);

    gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/gnss_odom", "/map", "/velo_link", 100);
}

bool EvaluationFlow::Run() {
    if (!ReadData()) {
        return false;
    }
        
    if (!InitCalibration())  {
        return false;
    }


    if (!InitGNSS()) {      
        return false; 
    }

    while( HasData() ) {
        if (!ValidData()) {
            continue;
        }
        
        UpdateLaserOdometry();
        UpdateGNSSOdometry();

        SaveTrajectory();
    }

    return true;
}

bool EvaluationFlow::InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node) {
    laser_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(
        nh, 
        "/laser_odom_scan_to_map", 1000
    );

    lidar_to_imu_ptr_ = std::make_shared<TFListener>(
        nh, 
        config_node["imu"]["frame_id"].as<std::string>(), config_node["lidar"]["frame_id"].as<std::string>()
    );
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(
        nh, 
        config_node["imu"]["topic_name"].as<std::string>(), config_node["imu"]["queue_size"].as<int>()
    );
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(
        nh, 
        config_node["gnss"]["topic_name"].as<std::string>(), config_node["gnss"]["queue_size"].as<int>()
    );

    return true;
}

bool EvaluationFlow::ReadData() {
    static bool evaluator_inited = false;

    laser_odom_sub_ptr_->ParseData(laser_odom_data_buff_);

    static std::deque<IMUData> unsynced_imu_;
    static std::deque<GNSSData> unsynced_gnss_;

    imu_sub_ptr_->ParseData(unsynced_imu_);
    gnss_sub_ptr_->ParseData(unsynced_gnss_);

    if ( laser_odom_data_buff_.empty() ) {
        return false;
    }
        
    double laser_odom_time = laser_odom_data_buff_.front().time;

    bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, laser_odom_time);
    bool valid_gnss = GNSSData::SyncData(unsynced_gnss_, gnss_data_buff_, laser_odom_time);

    if ( !evaluator_inited ) {
        if (!valid_imu || !valid_gnss) {
            laser_odom_data_buff_.pop_front();
            return false;
        }
        evaluator_inited = true;
    }

    return true;
}

bool EvaluationFlow::InitCalibration() {
    static bool calibration_received = false;
    if (!calibration_received) {
        if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
            calibration_received = true;
        }
    }

    return calibration_received;
}

bool EvaluationFlow::InitGNSS() {
    static bool gnss_inited = false;
    
    if (!gnss_inited) {
        GNSSData gnss_data = gnss_data_buff_.front();
        gnss_data.InitOriginPosition();
        gnss_inited = true;
    }

    return gnss_inited;
}

bool EvaluationFlow::HasData() {
    if ( laser_odom_data_buff_.empty() )
        return false;
    if ( imu_data_buff_.empty() )
        return false;
    if ( gnss_data_buff_.empty() )
        return false;
    
    return true;
}

bool EvaluationFlow::ValidData() {
    current_laser_odom_data_ = laser_odom_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();

    double d_time = current_laser_odom_data_.time - current_imu_data_.time;
    if (d_time < -0.08) {
        laser_odom_data_buff_.pop_front();
        return false;
    }

    if (d_time > 0.08) {
        imu_data_buff_.pop_front();
        gnss_data_buff_.pop_front();
        return false;
    }

    laser_odom_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    gnss_data_buff_.pop_front();

    return true;
}

bool EvaluationFlow::UpdateLaserOdometry() {
    laser_odometry_ = current_laser_odom_data_.pose;

    return true;
}

bool EvaluationFlow::UpdateGNSSOdometry() {
    static bool is_synced = false;
    static Eigen::Matrix4f gnss_to_odom = Eigen::Matrix4f::Identity();

    gnss_odometry_ = Eigen::Matrix4f::Identity();

    current_gnss_data_.UpdateXYZ();
    gnss_odometry_(0,3) = current_gnss_data_.local_E;
    gnss_odometry_(1,3) = current_gnss_data_.local_N;
    gnss_odometry_(2,3) = current_gnss_data_.local_U;
    gnss_odometry_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();
    gnss_odometry_ *= lidar_to_imu_;

    // init transform from GNSS frame to laser odometry frame:
    if (!is_synced) {
        gnss_to_odom = laser_odometry_ * gnss_odometry_.inverse();
        is_synced = true;
    }

    gnss_odometry_ = gnss_to_odom * gnss_odometry_;

    gnss_pub_ptr_->Publish(gnss_odometry_);

    return true;
}

bool EvaluationFlow::SaveTrajectory() {
    static std::ofstream ground_truth, laser_odom;
    static bool is_file_created = false;

    if (!is_file_created) {
        if (!FileManager::CreateDirectory(WORK_SPACE_PATH + "/slam_data/trajectory"))
            return false;
        if (!FileManager::CreateFile(ground_truth, WORK_SPACE_PATH + "/slam_data/trajectory/ground_truth.txt"))
            return false;
        if (!FileManager::CreateFile(laser_odom, WORK_SPACE_PATH + "/slam_data/trajectory/laser_odom.txt"))
            return false;
        is_file_created = true;
    }

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            ground_truth << gnss_odometry_(i, j);
            laser_odom << laser_odometry_(i, j);
            if (i == 2 && j == 3) {
                ground_truth << std::endl;
                laser_odom << std::endl;
            } else {
                ground_truth << " ";
                laser_odom << " ";
            }
        }
    }

    return true;
}

} // namespace lidar_localization