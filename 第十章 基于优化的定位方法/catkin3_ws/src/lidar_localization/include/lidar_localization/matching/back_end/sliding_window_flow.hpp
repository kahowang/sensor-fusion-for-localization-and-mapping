/*
 * @Description: LIO localization backend workflow, interface
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
#ifndef LIDAR_LOCALIZATION_MATCHING_BACK_END_SLIDING_WINDOW_FLOW_HPP_
#define LIDAR_LOCALIZATION_MATCHING_BACK_END_SLIDING_WINDOW_FLOW_HPP_

#include <ros/ros.h>

//
// subscribers:
//
// a. lidar odometry, map matching pose & GNSS reference position:
#include "lidar_localization/subscriber/odometry_subscriber.hpp"
// b. IMU measurement, for pre-integration:
#include "lidar_localization/subscriber/imu_subscriber.hpp"

#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/key_frame_publisher.hpp"
#include "lidar_localization/publisher/key_frames_publisher.hpp"
#include "lidar_localization/publisher/tf_broadcaster.hpp"

#include "lidar_localization/matching/back_end/sliding_window.hpp"


namespace lidar_localization {

class SlidingWindowFlow {
public:
    SlidingWindowFlow(ros::NodeHandle& nh);

    bool Run();
    bool SaveOptimizedTrajectory();
    
  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool UpdateIMUPreIntegration(void);
    bool UpdateBackEnd();
    bool PublishData();

  private:
    //
    // subscribers:
    //
    // a. lidar odometry:
    std::shared_ptr<OdometrySubscriber> laser_odom_sub_ptr_;
    std::deque<PoseData> laser_odom_data_buff_;
    // b. map matching odometry:
    std::shared_ptr<OdometrySubscriber> map_matching_odom_sub_ptr_;
    std::deque<PoseData> map_matching_odom_data_buff_;
    // c. IMU measurement, for pre-integration:
    std::shared_ptr<IMUSubscriber> imu_raw_sub_ptr_;
    std::deque<IMUData> imu_raw_data_buff_;
    std::shared_ptr<IMUSubscriber> imu_synced_sub_ptr_;
    std::deque<IMUData> imu_synced_data_buff_;
    // d. GNSS position:
    std::shared_ptr<OdometrySubscriber> gnss_pose_sub_ptr_;
    std::deque<PoseData> gnss_pose_data_buff_;

    //
    // publishers:
    //
    std::shared_ptr<KeyFramePublisher> key_frame_pub_ptr_;
    std::shared_ptr<KeyFramePublisher> key_gnss_pub_ptr_;
    std::shared_ptr<OdometryPublisher> optimized_odom_pub_ptr_;
    std::shared_ptr<KeyFramesPublisher> optimized_trajectory_pub_ptr_;
    std::shared_ptr<TFBroadCaster> laser_tf_pub_ptr_;

    //
    // backend:
    //
    std::shared_ptr<SlidingWindow> sliding_window_ptr_;

    //
    // synced data:
    //
    PoseData current_laser_odom_data_;
    PoseData current_map_matching_odom_data_;
    IMUData current_imu_data_;
    PoseData current_gnss_pose_data_;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MATCHING_BACK_END_SLIDING_WINDOW_FLOW_HPP_