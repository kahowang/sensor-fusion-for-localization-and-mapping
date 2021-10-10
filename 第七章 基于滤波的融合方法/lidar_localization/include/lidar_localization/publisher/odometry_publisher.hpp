/*
 * @Description: odometry 信息发布
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:05:47
 */
#ifndef LIDAR_LOCALIZATION_PUBLISHER_ODOMETRY_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_ODOMETRY_PUBLISHER_HPP_

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "lidar_localization/sensor_data/velocity_data.hpp"

namespace lidar_localization {
class OdometryPublisher {
  public:
    OdometryPublisher(ros::NodeHandle& nh, 
                      std::string topic_name, 
                      std::string base_frame_id,
                      std::string child_frame_id,
                      int buff_size);
    OdometryPublisher() = default;

    void Publish(const Eigen::Matrix4f& transform_matrix, double time);
    void Publish(const Eigen::Matrix4f& transform_matrix);
    void Publish(const Eigen::Matrix4f& transform_matrix, const VelocityData &velocity_data, double time);
    void Publish(const Eigen::Matrix4f& transform_matrix, const VelocityData &velocity_data);
    void Publish(const Eigen::Matrix4f& transform_matrix, const Eigen::Vector3f& vel, double time);
    void Publish(const Eigen::Matrix4f& transform_matrix, const Eigen::Vector3f& vel);

    bool HasSubscribers();

  private:
    void PublishData(
      const Eigen::Matrix4f& transform_matrix, 
      const VelocityData &velocity_data, 
      ros::Time time
    );

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;

    VelocityData velocity_data_;
    nav_msgs::Odometry odometry_;
};
}
#endif