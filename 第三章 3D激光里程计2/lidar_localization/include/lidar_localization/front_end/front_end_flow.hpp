/*
 * @Description: LOAM frontend facade
 * @Author: Ge Yao
 * @Date: 2021-01-30 22:38:22
 */
#ifndef LIDAR_LOCALIZATION_FRONT_END_FLOW_HPP_
#define LIDAR_LOCALIZATION_FRONT_END_FLOW_HPP_

#include <memory>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include "lidar_localization/subscriber/cloud_subscriber.hpp"

#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"

#include "lidar_localization/front_end/front_end.hpp"

namespace lidar_localization {

class FrontEndFlow {
  public:
    FrontEndFlow(ros::NodeHandle& nh);

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
    // inputs: registered scans
    std::unique_ptr<CloudSubscriber> filtered_cloud_sub_ptr_{nullptr};
    std::deque<CloudData> filtered_cloud_buff_;
    CloudData filtered_cloud_;

    std::unique_ptr<CloudSubscriber> corner_points_sharp_sub_ptr_{nullptr};
    std::deque<CloudData> corner_points_sharp_buff_;
    CloudData corner_points_sharp_;

    std::unique_ptr<CloudSubscriber> corner_points_less_sharp_sub_ptr_{nullptr};
    std::deque<CloudData> corner_points_less_sharp_buff_;
    CloudData corner_points_less_sharp_;

    std::unique_ptr<CloudSubscriber> surf_points_flat_sub_ptr_{nullptr};
    std::deque<CloudData> surf_points_flat_buff_;
    CloudData surf_points_flat_;

    std::unique_ptr<CloudSubscriber> surf_points_less_flat_sub_ptr_{nullptr};
    std::deque<CloudData> surf_points_less_flat_buff_;
    CloudData surf_points_less_flat_;

    // LOAM front end implementation:
    std::unique_ptr<FrontEnd> front_end_ptr_{nullptr};

    // outputs:
    std::unique_ptr<OdometryPublisher> odom_scan_to_scan_pub_ptr_;
    Eigen::Matrix4f odometry_ = Eigen::Matrix4f::Identity();
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_FRONT_END_FLOW_HPP_