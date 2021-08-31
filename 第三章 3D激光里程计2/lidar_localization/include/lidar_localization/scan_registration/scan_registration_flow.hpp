/*
 * @Description: scan registration facade
 * @Author: Ge Yao
 * @Date: 2021-01-30 22:38:22
 */
#ifndef LIDAR_LOCALIZATION_SCAN_REGISTRATION_FLOW_HPP_
#define LIDAR_LOCALIZATION_SCAN_REGISTRATION_FLOW_HPP_

#include <memory>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/publisher/cloud_publisher.hpp"

#include "lidar_localization/scan_registration/scan_registration.hpp"

namespace lidar_localization {

class ScanRegistrationFlow {
  public:
    ScanRegistrationFlow(ros::NodeHandle& nh);

    bool Run();

  private:
    bool InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node);
    bool InitPublishers(ros::NodeHandle& nh, const YAML::Node& config_node);

    bool ReadData(void);
    bool HasData(void);
    bool ValidData(void);
    bool UpdateData(void);
    bool PublishData(void);
    
  private:
    // input: velodyne measurements
    std::unique_ptr<CloudSubscriber> cloud_sub_ptr_{nullptr};
    std::deque<CloudData> cloud_data_buff_;
    CloudData current_cloud_data_;

    // scan registration implementation:
    std::unique_ptr<ScanRegistration> scan_registration_ptr_{nullptr};
    std::unique_ptr<CloudPublisher> filtered_cloud_pub_ptr_{nullptr};
    CloudData::CLOUD_PTR filtered_cloud_data_;

    // outputs: registered scans
    std::unique_ptr<CloudPublisher> corner_points_sharp_pub_ptr_{nullptr};
    CloudData::CLOUD_PTR corner_sharp_;
    std::unique_ptr<CloudPublisher> corner_points_less_sharp_pub_ptr_{nullptr};
    CloudData::CLOUD_PTR corner_less_sharp_;
    std::unique_ptr<CloudPublisher> surf_points_flat_pub_ptr_{nullptr};
    CloudData::CLOUD_PTR surf_flat_;
    std::unique_ptr<CloudPublisher> surf_points_less_flat_pub_ptr_{nullptr};
    CloudData::CLOUD_PTR surf_less_flat_;
    std::unique_ptr<CloudPublisher> removed_points_pub_ptr_{nullptr};
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_SCAN_REGISTRATION_FLOW_HPP_