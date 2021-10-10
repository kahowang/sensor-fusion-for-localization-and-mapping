/*
 * @Description: Subscribe to ROS odometry message
 * @Author: Ge Yao
 * @Date: 2020-11-10 14:25:03
 */
#include "imu_integration/subscriber/odom_subscriber.hpp"
#include "glog/logging.h"
#include <eigen3/Eigen/src/Geometry/RotationBase.h>

namespace imu_integration {

OdomSubscriber::OdomSubscriber(
  ros::NodeHandle& nh, 
  std::string topic_name, 
  size_t buff_size
) :nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size, &OdomSubscriber::msg_callback, this);
}

void OdomSubscriber::msg_callback(
  const nav_msgs::OdometryConstPtr& odom_msg_ptr
) {
    buff_mutex_.lock();

    // convert ROS IMU to GeographicLib compatible GNSS message:
    OdomData odom_data;
    odom_data.time = odom_msg_ptr->header.stamp.toSec();

    Eigen::Quaterniond q(
      odom_msg_ptr->pose.pose.orientation.w,
      odom_msg_ptr->pose.pose.orientation.x,
      odom_msg_ptr->pose.pose.orientation.y,
      odom_msg_ptr->pose.pose.orientation.z
    );
    Eigen::Vector3d t(
      odom_msg_ptr->pose.pose.position.x,
      odom_msg_ptr->pose.pose.position.y,
      odom_msg_ptr->pose.pose.position.z      
    );

    odom_data.pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    odom_data.pose.block<3, 1>(0, 3) = t;

    odom_data.vel = Eigen::Vector3d(
      odom_msg_ptr->twist.twist.linear.x,
      odom_msg_ptr->twist.twist.linear.y,
      odom_msg_ptr->twist.twist.linear.z 
    );

    // add new message to buffer:
    odom_data_.push_back(odom_data);
    
    buff_mutex_.unlock();
}

void OdomSubscriber::ParseData(
  std::deque<OdomData>& odom_data
) {
    buff_mutex_.lock();

    // pipe all available measurements to output buffer:
    if (odom_data_.size() > 0) {
        odom_data.insert(odom_data.end(), odom_data_.begin(), odom_data_.end());
        odom_data_.clear();
    }

    buff_mutex_.unlock();
}

} // namespace imu_integration