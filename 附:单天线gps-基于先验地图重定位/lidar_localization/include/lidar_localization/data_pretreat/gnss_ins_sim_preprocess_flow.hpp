/*
 * @Description: GNSS-INS-Sim measurement preprocessing workflow
 * @Author: Ge Yao
 * @Date: 2020-11-21 15:39:24
 */

#ifndef LIDAR_LOCALIZATION_DATA_PRETREAT_GNSS_INS_SIM_PREPROCESS_FLOW_HPP_
#define LIDAR_LOCALIZATION_DATA_PRETREAT_GNSS_INS_SIM_PREPROCESS_FLOW_HPP_

// ROS common:
#include <ros/ros.h>

// subscriber:
#include "lidar_localization/subscriber/imu_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include "lidar_localization/subscriber/velocity_subscriber.hpp"
#include "lidar_localization/subscriber/magnetic_field_subscriber.hpp"
#include "lidar_localization/subscriber/odometry_subscriber.hpp"

// publisher:
// a. IMU measurement:
#include "lidar_localization/publisher/imu_publisher.hpp"
// b. synced GNSS-odo measurement:
#include "lidar_localization/publisher/pos_vel_mag_publisher.hpp"
// c. reference trajectory:
#include "lidar_localization/publisher/odometry_publisher.hpp"

namespace lidar_localization {

class GNSSINSSimPreprocessFlow {
  public:
    GNSSINSSimPreprocessFlow(
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
    std::shared_ptr<VelocitySubscriber> odo_sub_ptr_;
    std::shared_ptr<MagneticFieldSubscriber> mag_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> ref_pose_sub_ptr_;
    // publisher:
    std::shared_ptr<IMUPublisher> imu_pub_ptr_;
    std::shared_ptr<PosVelMagPublisher> pos_vel_mag_pub_ptr_;
    std::shared_ptr<OdometryPublisher> gnss_pose_pub_ptr_;
    std::shared_ptr<OdometryPublisher> ref_pose_pub_ptr_;

    std::deque<IMUData> imu_data_buff_;
    std::deque<GNSSData> gnss_data_buff_;
    std::deque<VelocityData> odo_data_buff_;
    std::deque<MagneticFieldData> mag_data_buff_;
    std::deque<PoseData> ref_pose_data_buff_;

    IMUData current_imu_data_;
    GNSSData current_gnss_data_;
    VelocityData current_odo_data_;
    MagneticFieldData current_mag_data_;
    PoseData current_ref_pose_data_;

    PosVelMagData pos_vel_mag_;
    Eigen::Matrix4f gnss_pose_;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_DATA_PRETREAT_GNSS_INS_SIM_PREPROCESS_FLOW_HPP_