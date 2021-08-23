/*
 * @Description: frone-end workflow
 * @Author: Ge Yao
 * @Date: 2021-04-03
 */
#ifndef LIDAR_LOCALIZATION_FRONT_END_FRONT_END_FLOW_HPP_
#define LIDAR_LOCALIZATION_FRONT_END_FRONT_END_FLOW_HPP_

#include <memory>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

//
// TF tree:
//
#include "lidar_localization/subscriber/tf_listener.hpp"
#include "lidar_localization/publisher/tf_broadcaster.hpp"

//
// subscribers:
//
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/imu_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"

//
// publishers:
//
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"

//
// algorithm core:
//
#include "lidar_localization/front_end/front_end.hpp"

namespace lidar_localization {

class FrontEndFlow {
  public:
    FrontEndFlow(ros::NodeHandle& nh);

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
    bool UpdateGNSSOdometry();
    bool UpdateLaserOdometry();
    bool PublishData();
    bool SaveTrajectory();

  private:
    std::shared_ptr<TFListener> lidar_to_imu_tf_sub_ptr_;
    std::shared_ptr<TFBroadCaster> lidar_to_map_tf_pub_ptr_;

    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;

    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
    std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
    std::shared_ptr<CloudPublisher> global_map_pub_ptr_;

    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
    std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;

    std::shared_ptr<FrontEnd> front_end_ptr_;

    std::deque<CloudData> cloud_data_buff_;
    std::deque<IMUData> imu_data_buff_;
    std::deque<GNSSData> gnss_data_buff_;

    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();
    CloudData current_cloud_data_;
    IMUData current_imu_data_;
    GNSSData current_gnss_data_;

    CloudData::CLOUD_PTR local_map_ptr_;
    CloudData::CLOUD_PTR global_map_ptr_;
    CloudData::CLOUD_PTR current_scan_ptr_;

    Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f gnss_odometry_ = Eigen::Matrix4f::Identity();
};

}

#endif