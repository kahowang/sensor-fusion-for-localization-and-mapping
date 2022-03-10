/*
 * @Description: LIO mapping backend workflow, implementation
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
 
#include "lidar_localization/mapping/back_end/lio_back_end_flow.hpp"

#include "glog/logging.h"

#include "lidar_localization/tools/file_manager.hpp"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

LIOBackEndFlow::LIOBackEndFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string odom_topic) {
    //
    // subscribers:
    //
    // a. lidar scan, key frame measurement:
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, cloud_topic, 100000);
    // b. lidar odometry:
    laser_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, odom_topic, 100000);
    // c. GNSS position:
    gnss_pose_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);
    // d. loop closure detection:
    loop_pose_sub_ptr_ = std::make_shared<LoopPoseSubscriber>(nh, "/loop_pose", 100000);
    // e. IMU measurement, for pre-integration:
    imu_raw_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu/extract", 1000000);
    imu_synced_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/synced_imu", 100000);
    // f. odometer measurement, for pre-integration:
    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel/extract", 1000000);

    transformed_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/transformed_odom", "/map", "/lidar", 100);
    key_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/key_scan", "/velo_link", 100);
    key_frame_pub_ptr_ = std::make_shared<KeyFramePublisher>(nh, "/key_frame", "/map", 100);
    key_gnss_pub_ptr_ = std::make_shared<KeyFramePublisher>(nh, "/key_gnss", "/map", 100);
    key_frames_pub_ptr_ = std::make_shared<KeyFramesPublisher>(nh, "/optimized_key_frames", "/map", 100);

    back_end_ptr_ = std::make_shared<LIOBackEnd>();
}

bool LIOBackEndFlow::Run() {
    // load messages into buffer:
    if (!ReadData())
        return false;
    
    // add loop poses for graph optimization:
    InsertLoopClosurePose();

    while(HasData()) {
        // make sure undistorted Velodyne measurement -- lidar pose in map frame -- lidar odometry are synced:
        if (!ValidData())
            continue;

        UpdateBackEnd();

        PublishData();
    }

    return true;
}

bool LIOBackEndFlow::ForceOptimize() {
    static std::deque<KeyFrame> optimized_key_frames;

    back_end_ptr_->ForceOptimize();

    if ( back_end_ptr_->HasNewOptimized() ) {
        back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames);
        key_frames_pub_ptr_->Publish(optimized_key_frames);
    }
    return true;
}

bool LIOBackEndFlow::SaveOptimizedOdometry() {
    back_end_ptr_ -> SaveOptimizedPose();

    return true;
}

bool LIOBackEndFlow::ReadData() {
    // a. lidar scan, key frame measurement:
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    // b. lidar odometry:
    laser_odom_sub_ptr_->ParseData(laser_odom_data_buff_);
    // c. GNSS position:
    gnss_pose_sub_ptr_->ParseData(gnss_pose_data_buff_);
    // d. loop closure detection:
    loop_pose_sub_ptr_->ParseData(loop_pose_data_buff_);
    // e. IMU measurement, for pre-integration:
    imu_raw_sub_ptr_->ParseData(imu_raw_data_buff_);
    imu_synced_sub_ptr_->ParseData(imu_synced_data_buff_);
    // f. odometer measurement, for pre-integration:
    velocity_sub_ptr_->ParseData(velocity_data_buff_);

    return true;
}

/**
 * @brief  add loop closure for backend optimization
 * @param  void
 * @return true if success false otherwise
 */
bool LIOBackEndFlow::InsertLoopClosurePose() {
    while (loop_pose_data_buff_.size() > 0) {
        back_end_ptr_->InsertLoopPose(loop_pose_data_buff_.front());
        loop_pose_data_buff_.pop_front();
    }

    return true;
}

bool LIOBackEndFlow::HasData() {
    if (
        cloud_data_buff_.empty() ||
        laser_odom_data_buff_.empty() ||
        gnss_pose_data_buff_.empty() ||
        imu_synced_data_buff_.empty() 
    ) {
        return false;
    }

    return true;
}

bool LIOBackEndFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_laser_odom_data_ = laser_odom_data_buff_.front();
    current_gnss_pose_data_ = gnss_pose_data_buff_.front();
    current_imu_data_ = imu_synced_data_buff_.front();

    double diff_laser_time = current_cloud_data_.time - current_laser_odom_data_.time;
    double diff_gnss_time = current_cloud_data_.time - current_gnss_pose_data_.time;
    double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;

    if ( diff_laser_time < -0.05 || diff_gnss_time < -0.05 || diff_imu_time < -0.05 ) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if ( diff_laser_time > 0.05 ) {
        laser_odom_data_buff_.pop_front();
        return false;
    }

    if ( diff_gnss_time > 0.05 ) {
        gnss_pose_data_buff_.pop_front();
        return false;
    }

    if ( diff_imu_time > 0.05 ) {
        imu_synced_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    laser_odom_data_buff_.pop_front();
    gnss_pose_data_buff_.pop_front();
    imu_synced_data_buff_.pop_front();

    return true;
}

bool LIOBackEndFlow::UpdateIMUPreIntegration(void) {
    while (
        !imu_raw_data_buff_.empty() && 
        imu_raw_data_buff_.front().time < current_imu_data_.time && 
        back_end_ptr_->UpdateIMUPreIntegration(imu_raw_data_buff_.front())
    ) {
        imu_raw_data_buff_.pop_front();
    }

    return true;
}

bool LIOBackEndFlow::UpdateOdoPreIntegration(void) {
    while (
        !velocity_data_buff_.empty() && 
        velocity_data_buff_.front().time < current_imu_data_.time && 
        back_end_ptr_->UpdateOdoPreIntegration(velocity_data_buff_.front())
    ) {
        velocity_data_buff_.pop_front();
    }

    return true;
}

bool LIOBackEndFlow::UpdateBackEnd() {
    static bool odometry_inited = false;
    static Eigen::Matrix4f odom_init_pose = Eigen::Matrix4f::Identity();

    if (!odometry_inited) {
        // the origin of lidar odometry frame in map frame as init pose:
        odom_init_pose = current_gnss_pose_data_.pose * current_laser_odom_data_.pose.inverse();

        odometry_inited = true;
    }
    
    // update IMU pre-integration:
    UpdateIMUPreIntegration();
    
    // update odo pre-integration:
    UpdateOdoPreIntegration();
    
    // current lidar odometry in map frame:
    current_laser_odom_data_.pose = odom_init_pose * current_laser_odom_data_.pose;

    // optimization is carried out in map frame:
    return back_end_ptr_->Update(
        current_cloud_data_, 
        current_laser_odom_data_, 
        current_gnss_pose_data_,
        current_imu_data_
    );
}

bool LIOBackEndFlow::PublishData() {
    transformed_odom_pub_ptr_->Publish(current_laser_odom_data_.pose, current_laser_odom_data_.time);

    if (back_end_ptr_->HasNewKeyFrame()) {
        CloudData key_scan;

        back_end_ptr_->GetLatestKeyScan(key_scan);
        key_scan_pub_ptr_->Publish(key_scan.cloud_ptr, key_scan.time);
        
        KeyFrame key_frame;

        back_end_ptr_->GetLatestKeyFrame(key_frame);
        key_frame_pub_ptr_->Publish(key_frame);

        back_end_ptr_->GetLatestKeyGNSS(key_frame);
        key_gnss_pub_ptr_->Publish(key_frame);
    }

    if (back_end_ptr_->HasNewOptimized()) {
        std::deque<KeyFrame> optimized_key_frames;
        back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames);
        key_frames_pub_ptr_->Publish(optimized_key_frames);
    }

    return true;
}

} // namespace lidar_localization