/*
 * @Description: Subscribe to ROS odometry message
 * @Author: Ge Yao
 * @Date: 2020-11-10 14:25:03
 */
#ifndef IMU_INTEGRATION_ODOM_SUBSCRIBER_HPP_
#define IMU_INTEGRATION_ODOM_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "imu_integration/sensor_data/odom_data.hpp"

namespace imu_integration {

class OdomSubscriber {
  public:
    OdomSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    OdomSubscriber() = default;
    void ParseData(std::deque<OdomData>& odom_data);

  private:
    void msg_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<OdomData> odom_data_;

    std::mutex buff_mutex_; 
};

} // namespace imu_integration

#endif