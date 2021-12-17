/*
 * @Description: IMU/GNSS measurement preprocess for ESKF observability analysis
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */

#ifndef LIDAR_LOCALIZATION_DATA_PRETREAT_ESKF_PREPROCESS_FLOW_HPP_
#define LIDAR_LOCALIZATION_DATA_PRETREAT_ESKF_PREPROCESS_FLOW_HPP_

// ROS common:
#include <ros/ros.h>

// subscriber:
#include "lidar_localization/subscriber/imu_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include "lidar_localization/subscriber/velocity_subscriber.hpp"
#include "lidar_localization/subscriber/odometry_subscriber.hpp"
// publisher:
#include "lidar_localization/publisher/imu_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"

namespace lidar_localization {

class ESKFPreprocessFlow {
  public:
    ESKFPreprocessFlow(
      ros::NodeHandle& nh
    );
    bool Run();

  private:
    bool ReadData();
    bool InitGNSS();
    bool HasData();
    bool ValidData();
    bool TransformData();
    bool PublishData();

  private:
    // subscriber:
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
    std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> ref_pose_sub_ptr_;
    // publisher:
    std::shared_ptr<IMUPublisher> imu_pub_ptr_;
    std::shared_ptr<OdometryPublisher> gnss_pose_pub_ptr_;
    std::shared_ptr<OdometryPublisher> ref_pose_pub_ptr_;

    std::deque<IMUData> imu_data_buff_;
    std::deque<GNSSData> gnss_data_buff_;
    std::deque<VelocityData> velocity_data_buff_;
    std::deque<PoseData> ref_pose_data_buff_;

    IMUData current_imu_data_;
    GNSSData current_gnss_data_;
    VelocityData current_velocity_data_;
    PoseData current_ref_pose_data_;

    Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f ref_pose_ = Eigen::Matrix4f::Identity();
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_DATA_PRETREAT_ESKF_PREPROCESS_FLOW_HPP_