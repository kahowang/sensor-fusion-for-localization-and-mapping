/*
 * @Description: Subscribe to ROS IMU message
 * @Author: Ge Yao
 * @Date: 2020-11-10 14:25:03
 */
#include "imu_integration/subscriber/imu_subscriber.hpp"
#include "glog/logging.h"

namespace imu_integration {

IMUSubscriber::IMUSubscriber(
  ros::NodeHandle& nh, 
  std::string topic_name, 
  size_t buff_size
) :nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size, &IMUSubscriber::msg_callback, this);
}

void IMUSubscriber::msg_callback(
  const sensor_msgs::ImuConstPtr& imu_msg_ptr
) {
    buff_mutex_.lock();

    // convert ROS IMU to GeographicLib compatible GNSS message:
    IMUData imu_data;
    imu_data.time = imu_msg_ptr->header.stamp.toSec();

    imu_data.linear_acceleration = Eigen::Vector3d(
      imu_msg_ptr->linear_acceleration.x,
      imu_msg_ptr->linear_acceleration.y,
      imu_msg_ptr->linear_acceleration.z
    );

    imu_data.angular_velocity = Eigen::Vector3d(
      imu_msg_ptr->angular_velocity.x,
      imu_msg_ptr->angular_velocity.y,
      imu_msg_ptr->angular_velocity.z
    );

    // add new message to buffer:
    imu_data_.push_back(imu_data);
    
    buff_mutex_.unlock();
}

void IMUSubscriber::ParseData(
  std::deque<IMUData>& imu_data
) {
    buff_mutex_.lock();

    // pipe all available measurements to output buffer:
    if (imu_data_.size() > 0) {
        imu_data.insert(imu_data.end(), imu_data_.begin(), imu_data_.end());
        imu_data_.clear();
    }

    buff_mutex_.unlock();
}

} // namespace imu_integration