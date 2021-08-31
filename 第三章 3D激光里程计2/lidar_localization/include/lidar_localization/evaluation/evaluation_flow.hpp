/*
 * @Description: evo evaluation facade
 * @Author: Ge Yao
 * @Date: 2021-05-01 07:38:22
 */
#ifndef LIDAR_LOCALIZATION_EVALUATION_FLOW_HPP_
#define LIDAR_LOCALIZATION_EVALUATION_FLOW_HPP_

#include <memory>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include "lidar_localization/subscriber/odometry_subscriber.hpp"

#include "lidar_localization/tf_listener/tf_listener.hpp"
#include "lidar_localization/subscriber/imu_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"

#include "lidar_localization/publisher/odometry_publisher.hpp"

namespace lidar_localization {

class EvaluationFlow {
  public:
    EvaluationFlow(ros::NodeHandle& nh);

    bool Run();
    bool SaveMap();
    bool PublishGlobalMap();

  private:
    bool InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node);

    bool ReadData();
    bool InitCalibration();
    bool InitGNSS();
    bool HasData();
    bool ValidData();
    bool UpdateLaserOdometry();
    bool UpdateGNSSOdometry();
    bool SaveTrajectory();

  private:
    std::shared_ptr<OdometrySubscriber> laser_odom_sub_ptr_;
    std::deque<PoseData> laser_odom_data_buff_;
    PoseData current_laser_odom_data_;

    std::shared_ptr<TFListener> lidar_to_imu_ptr_;
    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();

    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::deque<IMUData> imu_data_buff_;
    IMUData current_imu_data_;

    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
    std::deque<GNSSData> gnss_data_buff_;
    GNSSData current_gnss_data_;

    std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;

    Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f gnss_odometry_ = Eigen::Matrix4f::Identity();
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_EVALUATION_FLOW_HPP_