/*
 * @Description: LIO mapping backend workflow, interface
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
#ifndef LIDAR_LOCALIZATION_MAPPING_BACK_END_LIO_BACK_END_FLOW_HPP_
#define LIDAR_LOCALIZATION_MAPPING_BACK_END_LIO_BACK_END_FLOW_HPP_

#include <ros/ros.h>

//
// subscribers:
//
// a. lidar scan, key frame measurement:
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
// b. lidar odometry & GNSS position:
#include "lidar_localization/subscriber/odometry_subscriber.hpp"
// c. loop closure detection:
#include "lidar_localization/subscriber/loop_pose_subscriber.hpp"
// d. IMU measurement, for pre-integration:
#include "lidar_localization/subscriber/imu_subscriber.hpp"
// e. odometer measurement, for pre-integration:
#include "lidar_localization/subscriber/velocity_subscriber.hpp"

#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/key_frame_publisher.hpp"
#include "lidar_localization/publisher/key_frames_publisher.hpp"

#include "lidar_localization/mapping/back_end/lio_back_end.hpp"

namespace lidar_localization {

class LIOBackEndFlow {
public:
    LIOBackEndFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string odom_topic);

    bool Run();
    bool ForceOptimize();
    bool SaveOptimizedOdometry();
    
  private:
    bool ReadData();
    bool InsertLoopClosurePose();
    bool HasData();
    bool ValidData();
    bool UpdateIMUPreIntegration(void);
    bool UpdateOdoPreIntegration(void);
    bool UpdateBackEnd();
    bool PublishData();

  private:
    //
    // subscribers:
    //
    // a. lidar scan, key frame measurement:
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::deque<CloudData> cloud_data_buff_;
    // b. lidar odometry:
    std::shared_ptr<OdometrySubscriber> laser_odom_sub_ptr_;
    std::deque<PoseData> laser_odom_data_buff_;
    // c. GNSS position:
    std::shared_ptr<OdometrySubscriber> gnss_pose_sub_ptr_;
    std::deque<PoseData> gnss_pose_data_buff_;
    // d. loop closure detection:
    std::shared_ptr<LoopPoseSubscriber> loop_pose_sub_ptr_;
    std::deque<LoopPose> loop_pose_data_buff_;
    // e. IMU measurement, for pre-integration:
    std::shared_ptr<IMUSubscriber> imu_raw_sub_ptr_;
    std::deque<IMUData> imu_raw_data_buff_;
    std::shared_ptr<IMUSubscriber> imu_synced_sub_ptr_;
    std::deque<IMUData> imu_synced_data_buff_;
    // f. odometer measurement, for pre-integration:
    std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
    std::deque<VelocityData> velocity_data_buff_;

    std::shared_ptr<OdometryPublisher> transformed_odom_pub_ptr_;
    std::shared_ptr<CloudPublisher> key_scan_pub_ptr_;
    std::shared_ptr<KeyFramePublisher> key_frame_pub_ptr_;
    std::shared_ptr<KeyFramePublisher> key_gnss_pub_ptr_;
    std::shared_ptr<KeyFramesPublisher> key_frames_pub_ptr_;
    std::shared_ptr<LIOBackEnd> back_end_ptr_;

    CloudData current_cloud_data_;
    PoseData current_laser_odom_data_;
    PoseData current_gnss_pose_data_;
    IMUData current_imu_data_;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MAPPING_BACK_END_LIO_BACK_END_FLOW_HPP_