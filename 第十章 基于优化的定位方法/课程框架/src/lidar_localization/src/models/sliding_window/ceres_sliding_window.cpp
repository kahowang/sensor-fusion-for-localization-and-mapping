/*
 * @Description: ceres sliding window optimizer, implementation
 * @Author: Ge Yao
 * @Date: 2021-01-03 14:53:21
 */

#include "lidar_localization/models/sliding_window/ceres_sliding_window.hpp"

#include <sophus/so3.hpp>

#include <chrono>

#include "glog/logging.h"

namespace lidar_localization {

CeresSlidingWindow::CeresSlidingWindow(
    const int N
) : kWindowSize(N) 
{   
    //
    // config optimizer:
    // 
    // a. loss function:
    config_.loss_function_ptr = std::make_unique<ceres::CauchyLoss>(1.0);

    // b. solver:
    config_.options.linear_solver_type = ceres::DENSE_SCHUR;
    // config_.options.use_explicit_schur_complement = true;
    config_.options.trust_region_strategy_type = ceres::DOGLEG;
    // config_.options.use_nonmonotonic_steps = true;
    config_.options.num_threads = 2;
    config_.options.max_num_iterations = 1000;
    config_.options.max_solver_time_in_seconds = 0.10;
    // config_.options.minimizer_progress_to_stdout = true;

    //
    // clear data buffer:
    // 
    optimized_key_frames_.clear();

    residual_blocks_.relative_pose.clear();
    residual_blocks_.map_matching_pose.clear();
    residual_blocks_.imu_pre_integration.clear();
}

CeresSlidingWindow::~CeresSlidingWindow() {
}

/**
 * @brief  add parameter block for LIO key frame
 * @param  lio_key_frame, LIO key frame with (pos, ori, vel, b_a and b_g)
 * @param  fixed, shall the param block be fixed to eliminate trajectory estimation ambiguity
 * @return true if success false otherwise
 */
void CeresSlidingWindow::AddPRVAGParam(
    const KeyFrame &lio_key_frame, const bool fixed
) {
    // create new key frame:
    OptimizedKeyFrame optimized_key_frame;

    // a. set timestamp:
    optimized_key_frame.time = lio_key_frame.time;

    // b. shall the param block be fixed:
    optimized_key_frame.fixed = fixed;

    // c. set init values:
    Eigen::Map<Eigen::Vector3d>     pos(optimized_key_frame.prvag + INDEX_P);
    Eigen::Map<Eigen::Vector3d> log_ori(optimized_key_frame.prvag + INDEX_R);
    Eigen::Map<Eigen::Vector3d>     vel(optimized_key_frame.prvag + INDEX_V);
    Eigen::Map<Eigen::Vector3d>     b_a(optimized_key_frame.prvag + INDEX_A);
    Eigen::Map<Eigen::Vector3d>     b_g(optimized_key_frame.prvag + INDEX_G);

    pos = lio_key_frame.pose.block<3, 1>(0, 3).cast<double>();

    Sophus::SO3d ori(
        Eigen::Quaterniond(lio_key_frame.pose.block<3, 3>(0, 0).cast<double>())
    );
    log_ori = ori.log();

    vel = ori * lio_key_frame.vel.v.cast<double>();
    b_a = lio_key_frame.bias.accel.cast<double>();
    b_g = lio_key_frame.bias.gyro.cast<double>();

    // add to data buffer:
    optimized_key_frames_.push_back(optimized_key_frame);
}

/**
 * @brief  add residual block for relative pose constraint from lidar frontend
 * @param  param_index_i, param block ID of previous key frame
 * @param  param_index_j, param block ID of current key frame
 * @param  relative_pose, relative pose measurement
 * @param  noise, relative pose measurement noise
 * @return void
 */
void CeresSlidingWindow::AddPRVAGRelativePoseFactor(
    const int param_index_i, const int param_index_j,
    const Eigen::Matrix4d &relative_pose, const Eigen::VectorXd &noise
) {
    // create new residual block:
    ResidualRelativePose residual_relative_pose;

    // a. set param block IDs:
    residual_relative_pose.param_index_i = param_index_i;
    residual_relative_pose.param_index_j = param_index_j;

    // b. set measurement
    residual_relative_pose.m = Eigen::VectorXd::Zero(6);
    // b.1. position:
    residual_relative_pose.m.block<3, 1>(INDEX_P, 0) = relative_pose.block<3, 1>(0, 3);
    // b.2. orientation, so3:
    residual_relative_pose.m.block<3, 1>(INDEX_R, 0) = Sophus::SO3d(
        Eigen::Quaterniond(relative_pose.block<3, 3>(0, 0).cast<double>())
    ).log();

    // c. set information matrix:
    residual_relative_pose.I = GetInformationMatrix(noise);

    // add to data buffer:
    residual_blocks_.relative_pose.push_back(residual_relative_pose);
}

/**
 * @brief  add residual block for prior pose constraint from map matching
 * @param  param_index, param block ID of current key frame
 * @param  pos, prior position measurement
 * @param  noise, prior position measurement noise
 * @return void
 */
void CeresSlidingWindow::AddPRVAGMapMatchingPoseFactor(
    const int param_index,
    const Eigen::Matrix4d &prior_pose, const Eigen::VectorXd &noise
) {
    // create new residual block:
    ResidualMapMatchingPose residual_map_matching_pose;

    // a. set param block ID:
    residual_map_matching_pose.param_index = param_index;

    // b. set measurement
    residual_map_matching_pose.m = Eigen::VectorXd::Zero(6);
    // b.1. position:
    residual_map_matching_pose.m.block<3, 1>(INDEX_P, 0) = prior_pose.block<3, 1>(0, 3);
    // b.2. orientation, so3:
    residual_map_matching_pose.m.block<3, 1>(INDEX_R, 0) = Sophus::SO3d(
        Eigen::Quaterniond(prior_pose.block<3, 3>(0, 0).cast<double>())
    ).log();

    // c. set information matrix:
    residual_map_matching_pose.I = GetInformationMatrix(noise);

    // add to data buffer:
    residual_blocks_.map_matching_pose.push_back(residual_map_matching_pose);
}

/**
 * @brief  add residual block for IMU pre-integration constraint from IMU measurement
 * @param  param_index_i, param block ID of previous key frame
 * @param  param_index_j, param block ID of current key frame
 * @param  imu_pre_integration, IMU pre-integration measurement
 * @return void
 */
void CeresSlidingWindow::AddPRVAGIMUPreIntegrationFactor(
    const int param_index_i, const int param_index_j,
    const IMUPreIntegrator::IMUPreIntegration &imu_pre_integration
) {
    // create new residual block:
    ResidualIMUPreIntegration residual_imu_pre_integration;

    // a. set param block IDs:
    residual_imu_pre_integration.param_index_i = param_index_i;
    residual_imu_pre_integration.param_index_j = param_index_j;

    // b. set measurement
    // b.1. integration interval:
    residual_imu_pre_integration.T = imu_pre_integration.GetT();
    // b.2. gravity constant:
    residual_imu_pre_integration.g = imu_pre_integration.GetGravity();
    // b.3. measurement:
    residual_imu_pre_integration.m = imu_pre_integration.GetMeasurement();
    // b.4. information:
    residual_imu_pre_integration.I = imu_pre_integration.GetInformation();
    /*
    LOG(INFO) << "IMU Pre-Integration Info.: " 
              << residual_imu_pre_integration.I(0,0) << ", "
              << residual_imu_pre_integration.I(1,1) << ", "
              << residual_imu_pre_integration.I(2,2) << ", " 
              << residual_imu_pre_integration.I(3,3) << ", " 
              << residual_imu_pre_integration.I(4,4) << ", " 
              << residual_imu_pre_integration.I(5,5) << ", "
              << std::endl;
    */
    // b.5. Jacobian:
    residual_imu_pre_integration.J = imu_pre_integration.GetJacobian();

    // add to data buffer:
    residual_blocks_.imu_pre_integration.push_back(residual_imu_pre_integration);
}

sliding_window::FactorPRVAGMapMatchingPose *CeresSlidingWindow::GetResMapMatchingPose(
    const CeresSlidingWindow::ResidualMapMatchingPose &res_map_matching_pose
) {
    sliding_window::FactorPRVAGMapMatchingPose *factor_map_matching_pose = new sliding_window::FactorPRVAGMapMatchingPose();
    
    factor_map_matching_pose->SetMeasurement(res_map_matching_pose.m);
    factor_map_matching_pose->SetInformation(res_map_matching_pose.I);

    return factor_map_matching_pose;
}

sliding_window::FactorPRVAGRelativePose *CeresSlidingWindow::GetResRelativePose(
    const CeresSlidingWindow::ResidualRelativePose &res_relative_pose
) {
    sliding_window::FactorPRVAGRelativePose *factor_relative_pose = new sliding_window::FactorPRVAGRelativePose();
    
    factor_relative_pose->SetMeasurement(res_relative_pose.m);
    factor_relative_pose->SetInformation(res_relative_pose.I);   

    return factor_relative_pose;
}

sliding_window::FactorPRVAGIMUPreIntegration *CeresSlidingWindow::GetResIMUPreIntegration(
    const ResidualIMUPreIntegration &res_imu_pre_integration
) {
    sliding_window::FactorPRVAGIMUPreIntegration *factor_imu_pre_integration = new sliding_window::FactorPRVAGIMUPreIntegration();

    factor_imu_pre_integration->SetT(res_imu_pre_integration.T);
    factor_imu_pre_integration->SetGravitiy(res_imu_pre_integration.g);
    factor_imu_pre_integration->SetMeasurement(res_imu_pre_integration.m);
    factor_imu_pre_integration->SetInformation(res_imu_pre_integration.I);
    factor_imu_pre_integration->SetJacobian(res_imu_pre_integration.J);   

    return factor_imu_pre_integration;
}

bool CeresSlidingWindow::Optimize() {
    static int optimization_count = 0;
    
    // get key frames count:
    const int N = GetNumParamBlocks();

    if ( 
        (kWindowSize + 1 <= N)
    ) {
        // TODO: create new sliding window optimization problem:
        ceres::Problem problem;

        // TODO: a. add parameter blocks:
        for ( int i = 1; i <= kWindowSize + 1; ++i) {
            auto &target_key_frame = optimized_key_frames_.at(N - i);

            ceres::LocalParameterization *local_parameterization = new sliding_window::ParamPRVAG();

            // TODO: add parameter block:
        }

        // TODO: add residual blocks:
        // b.1. marginalization constraint:
        if (
            !residual_blocks_.map_matching_pose.empty() && 
            !residual_blocks_.relative_pose.empty() && 
            !residual_blocks_.imu_pre_integration.empty()
        ) {
            auto &key_frame_m = optimized_key_frames_.at(N - kWindowSize - 1);
            auto &key_frame_r = optimized_key_frames_.at(N - kWindowSize - 0);

            const ceres::CostFunction *factor_map_matching_pose = GetResMapMatchingPose(
                residual_blocks_.map_matching_pose.front()
            );
            const ceres::CostFunction *factor_relative_pose = GetResRelativePose(
                residual_blocks_.relative_pose.front()
            );
            const ceres::CostFunction *factor_imu_pre_integration = GetResIMUPreIntegration(
                residual_blocks_.imu_pre_integration.front()
            );

            sliding_window::FactorPRVAGMarginalization *factor_marginalization = new sliding_window::FactorPRVAGMarginalization();

            factor_marginalization->SetResMapMatchingPose(
                factor_map_matching_pose, 
                std::vector<double *>{key_frame_m.prvag}
            );
            factor_marginalization->SetResRelativePose(
                factor_relative_pose,
                std::vector<double *>{key_frame_m.prvag, key_frame_r.prvag}
            );
            factor_marginalization->SetResIMUPreIntegration(
                factor_imu_pre_integration,
                std::vector<double *>{key_frame_m.prvag, key_frame_r.prvag}
            );
            factor_marginalization->Marginalize(key_frame_r.prvag);

            // add marginalization factor into sliding window
            problem.AddResidualBlock(
                factor_marginalization,
                NULL,
                key_frame_r.prvag
            );

            residual_blocks_.map_matching_pose.pop_front();
            residual_blocks_.relative_pose.pop_front();
            residual_blocks_.imu_pre_integration.pop_front();
        }

        // TODO: b.2. map matching pose constraint:
        if ( !residual_blocks_.map_matching_pose.empty() ) {
            for ( const auto &residual_map_matching_pose: residual_blocks_.map_matching_pose ) {
                auto &key_frame = optimized_key_frames_.at(residual_map_matching_pose.param_index);

                sliding_window::FactorPRVAGMapMatchingPose *factor_map_matching_pose = GetResMapMatchingPose(
                    residual_map_matching_pose
                );

                // TODO: add map matching factor into sliding window
            }            
        }

        // TODO: b.3. relative pose constraint:
        if ( !residual_blocks_.relative_pose.empty() ) {
            for ( const auto &residual_relative_pose: residual_blocks_.relative_pose ) {
                auto &key_frame_i = optimized_key_frames_.at(residual_relative_pose.param_index_i);
                auto &key_frame_j = optimized_key_frames_.at(residual_relative_pose.param_index_j);

                sliding_window::FactorPRVAGRelativePose *factor_relative_pose = GetResRelativePose(
                    residual_relative_pose
                );

                // TODO: add relative pose factor into sliding window
            }
        }

        // TODO: b.4. IMU pre-integration constraint
        if ( !residual_blocks_.imu_pre_integration.empty() ) {
            for ( const auto &residual_imu_pre_integration: residual_blocks_.imu_pre_integration ) {
                auto &key_frame_i = optimized_key_frames_.at(residual_imu_pre_integration.param_index_i);
                auto &key_frame_j = optimized_key_frames_.at(residual_imu_pre_integration.param_index_j);

                sliding_window::FactorPRVAGIMUPreIntegration *factor_imu_pre_integration = GetResIMUPreIntegration(
                    residual_imu_pre_integration
                );

                // TODO: add IMU factor into sliding window
            }
        }

        // solve:
        ceres::Solver::Summary summary;

        auto start = std::chrono::steady_clock::now();
        ceres::Solve(config_.options, &problem, &summary);
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = end-start;

        // prompt:
        LOG(INFO) << "------ Finish Iteration " << ++optimization_count << " of Sliding Window Optimization -------" << std::endl
                  << "Time Used: " << time_used.count() << " seconds." << std::endl
                  << "Cost Reduced: " << summary.initial_cost - summary.final_cost << std::endl
                  << summary.BriefReport() << std::endl
                  << std::endl;
        
        return true;
    }

    return false;
}

int CeresSlidingWindow::GetNumParamBlocks() {
    return static_cast<int>(optimized_key_frames_.size());
}

/**
 * @brief  get optimized odometry estimation
 * @param  optimized_key_frame, output latest optimized key frame
 * @return true if success false otherwise
 */
bool CeresSlidingWindow::GetLatestOptimizedKeyFrame(KeyFrame &optimized_key_frame) {
    const int N = GetNumParamBlocks();
    if ( 0 == N ) return false;

    const auto &latest_optimized_key_frame = optimized_key_frames_.back();

    optimized_key_frame = KeyFrame(
        N - 1, latest_optimized_key_frame.time, latest_optimized_key_frame.prvag
    );

    return true;
}

/**
 * @brief  get optimized LIO key frame state estimation
 * @param  optimized_key_frames, output optimized LIO key frames
 * @return true if success false otherwise
 */
bool CeresSlidingWindow::GetOptimizedKeyFrames(std::deque<KeyFrame> &optimized_key_frames) {
    optimized_key_frames.clear();

    const int N = GetNumParamBlocks();
    if ( 0 == N ) return false;

    for (int param_id = 0; param_id < N; param_id++) {
        const auto &optimized_key_frame = optimized_key_frames_.at(param_id);

        optimized_key_frames.emplace_back( param_id, optimized_key_frame.time, optimized_key_frame.prvag );
    }

    return true;
}

/**
 * @brief  create information matrix from measurement noise specification
 * @param  noise, measurement noise covariances
 * @return information matrix as square Eigen::MatrixXd
 */
Eigen::MatrixXd CeresSlidingWindow::GetInformationMatrix(Eigen::VectorXd noise) {
    Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(
        noise.rows(), noise.rows()
    );

    for (int i = 0; i < noise.rows(); i++) {
        information_matrix(i, i) /= noise(i);
    }

    return information_matrix;
}

} // namespace graph_ptr_optimization
