/*
 * @Description: 在ros中发布IMU数据
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */
#ifndef LIDAR_LOCALIZATION_PUBLISHER_IMU_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_IMU_PUBLISHER_HPP_

#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "lidar_localization/sensor_data/imu_data.hpp"

namespace lidar_localization {
class IMUPublisher {
  public:
    IMUPublisher(
      ros::NodeHandle& nh,
      std::string topic_name,
      std::string frame_id,
      size_t buff_size
    );
    IMUPublisher() = default;

    void Publish(const IMUData &imu_data, double time);
    void Publish(const IMUData &imu_data);

    bool HasSubscribers(void);

  private:
    void PublishData(const IMUData &imu_data, ros::Time time);

    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;

    sensor_msgs::Imu imu_;
};
} 
#endif