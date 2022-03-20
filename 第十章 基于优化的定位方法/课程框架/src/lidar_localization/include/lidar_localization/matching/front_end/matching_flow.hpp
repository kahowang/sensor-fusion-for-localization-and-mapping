/*
 * @Description: LIO localization frontend workflow, interface
 * @Author: Ge Yao
 * @Date: 2021-01-02 10:47:23
 */
#ifndef LIDAR_LOCALIZATION_MATCHING_FRONT_END_MATCHING_FLOW_HPP_
#define LIDAR_LOCALIZATION_MATCHING_FRONT_END_MATCHING_FLOW_HPP_

#include <ros/ros.h>

// subscriber
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/odometry_subscriber.hpp"

// publisher
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/publisher/tf_broadcaster.hpp"

// matching
#include "lidar_localization/matching/front_end/matching.hpp"

namespace lidar_localization {

class MatchingFlow {
  public:
    MatchingFlow(ros::NodeHandle& nh);

    bool Run();

  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool UpdateMatching();
    bool PublishData();

  private:
    //
    // subscribers:
    //  
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> gnss_sub_ptr_;

    //
    // publishers:
    //
    std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
    std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
    std::shared_ptr<CloudPublisher> current_scan_pub_ptr_;

    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
    std::shared_ptr<OdometryPublisher> map_matching_odom_pub_ptr_;

    // matching
    std::shared_ptr<Matching> matching_ptr_;

    std::deque<CloudData> cloud_data_buff_;
    std::deque<PoseData> gnss_data_buff_;

    CloudData current_cloud_data_;
    PoseData current_gnss_data_;

    Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f map_matching_odometry_ = Eigen::Matrix4f::Identity();
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MATCHING_FRONT_END_MATCHING_FLOW_HPP_