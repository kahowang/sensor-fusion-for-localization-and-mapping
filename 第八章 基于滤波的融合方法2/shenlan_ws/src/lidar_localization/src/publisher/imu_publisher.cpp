/*
 * @Description: 通过ros发布点云
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */
#include "lidar_localization/publisher/imu_publisher.hpp"
#include "glog/logging.h"

namespace lidar_localization {

IMUPublisher::IMUPublisher(
    ros::NodeHandle& nh,
    std::string topic_name,
    std::string frame_id,
    size_t buff_size
)
    :nh_(nh), frame_id_(frame_id) {
    publisher_ = nh_.advertise<sensor_msgs::Imu>(topic_name, buff_size);

    imu_.header.frame_id = frame_id_;
}

void IMUPublisher::Publish(const IMUData &imu_data, double time) {
    ros::Time ros_time(time);
    PublishData(imu_data, ros_time);
}

void IMUPublisher::Publish(const IMUData &imu_data) {
    ros::Time time = ros::Time::now();
    PublishData(imu_data, time);
}

void IMUPublisher::PublishData(const IMUData &imu_data, ros::Time time) {
    imu_.header.stamp = time;

    // set orientation:
    imu_.orientation.w = imu_data.orientation.w;
    imu_.orientation.x = imu_data.orientation.x;
    imu_.orientation.y = imu_data.orientation.y;
    imu_.orientation.z = imu_data.orientation.z;

    // set angular velocity:
    imu_.angular_velocity.x = imu_data.angular_velocity.x;
    imu_.angular_velocity.y = imu_data.angular_velocity.y;
    imu_.angular_velocity.z = imu_data.angular_velocity.z;

    // set linear acceleration:
    imu_.linear_acceleration.x = imu_data.linear_acceleration.x;
    imu_.linear_acceleration.y = imu_data.linear_acceleration.y;
    imu_.linear_acceleration.z = imu_data.linear_acceleration.z;

    publisher_.publish(imu_);
}

bool IMUPublisher::HasSubscribers(void) {
    return publisher_.getNumSubscribers() != 0;
}

} // namespace lidar_localization