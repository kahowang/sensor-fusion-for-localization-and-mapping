/*
 * @Description: 订阅激光点云信息，并解析数据
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */

#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_LIDAR_MEASUREMENT_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_LIDAR_MEASUREMENT_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <lidar_localization/LidarMeasurement.h>

#include "lidar_localization/sensor_data/lidar_measurement_data.hpp"

namespace lidar_localization {

class LidarMeasurementSubscriber {
  public:
    LidarMeasurementSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    LidarMeasurementSubscriber() = default;
    void ParseData(std::deque<LidarMeasurementData>& deque_cloud_data);

  private:
    void ParseCloudData(
      const sensor_msgs::PointCloud2 &point_cloud_msg,
      CloudData &point_cloud_data
    );
    void ParseIMUData(
      const sensor_msgs::Imu &imu_msg,
      IMUData &imu_data
    );
    void ParsePoseData(
      const nav_msgs::Odometry &gnss_odometry_msg,
      PoseData &gnss_odometry_data
    );
    void msg_callback(const LidarMeasurement::ConstPtr& synced_cloud_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<LidarMeasurementData> new_cloud_data_;

    std::mutex buff_mutex_;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_SUBSCRIBER_LIDAR_MEASUREMENT_SUBSCRIBER_HPP_