/*
 * @Description: Subscribe to ROS IMU message
 * @Author: Ge Yao
 * @Date: 2020-11-10 14:25:03
 */
#ifndef IMU_INTEGRATION_IMU_SUBSCRIBER_HPP_
#define IMU_INTEGRATION_IMU_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "imu_integration/sensor_data/imu_data.hpp"

namespace imu_integration {

class IMUSubscriber {
  public:
    IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    IMUSubscriber() = default;
    void ParseData(std::deque<IMUData>& imu_data);

  private:
    void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<IMUData> imu_data_;

    std::mutex buff_mutex_; 
};

} // namespace imu_integration

#endif