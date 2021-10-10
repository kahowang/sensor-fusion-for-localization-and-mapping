/*
 * @Description: Subscribe to & parse synced lidar measurement
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */

#include "lidar_localization/subscriber/lidar_measurement_subscriber.hpp"

#include "glog/logging.h"

namespace lidar_localization {

LidarMeasurementSubscriber::LidarMeasurementSubscriber(
    ros::NodeHandle& nh, 
    std::string topic_name, 
    size_t buff_size
)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &LidarMeasurementSubscriber::msg_callback, this);
}

void LidarMeasurementSubscriber::ParseData(
    std::deque<LidarMeasurementData>& cloud_data_buff
) {
    buff_mutex_.lock();

    // pipe all available measurements to output buffer:
    if (new_cloud_data_.size() > 0) {
        cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
        new_cloud_data_.clear();
    }
    
    buff_mutex_.unlock();
}

void LidarMeasurementSubscriber::ParseCloudData(
    const sensor_msgs::PointCloud2 &point_cloud_msg,
    CloudData &point_cloud_data
) {
    point_cloud_data.time = point_cloud_msg.header.stamp.toSec();
    pcl::fromROSMsg(point_cloud_msg, *(point_cloud_data.cloud_ptr));
}

void LidarMeasurementSubscriber::ParseIMUData(
    const sensor_msgs::Imu &imu_msg,
    IMUData &imu_data
) {
    imu_data.time = imu_msg.header.stamp.toSec();

    imu_data.linear_acceleration.x = imu_msg.linear_acceleration.x;
    imu_data.linear_acceleration.y = imu_msg.linear_acceleration.y;
    imu_data.linear_acceleration.z = imu_msg.linear_acceleration.z;

    imu_data.angular_velocity.x = imu_msg.angular_velocity.x;
    imu_data.angular_velocity.y = imu_msg.angular_velocity.y;
    imu_data.angular_velocity.z = imu_msg.angular_velocity.z;

    imu_data.orientation.x = imu_msg.orientation.x;
    imu_data.orientation.y = imu_msg.orientation.y;
    imu_data.orientation.z = imu_msg.orientation.z;
    imu_data.orientation.w = imu_msg.orientation.w;
}

void LidarMeasurementSubscriber::ParsePoseData(
    const nav_msgs::Odometry &gnss_odometry_msg,
    PoseData &gnss_odometry_data
) {
    gnss_odometry_data.time = gnss_odometry_msg.header.stamp.toSec();

    // set the position:
    gnss_odometry_data.pose(0,3) = gnss_odometry_msg.pose.pose.position.x;
    gnss_odometry_data.pose(1,3) = gnss_odometry_msg.pose.pose.position.y;
    gnss_odometry_data.pose(2,3) = gnss_odometry_msg.pose.pose.position.z;

    // set the orientation:
    Eigen::Quaternionf q;
    q.x() = gnss_odometry_msg.pose.pose.orientation.x;
    q.y() = gnss_odometry_msg.pose.pose.orientation.y;
    q.z() = gnss_odometry_msg.pose.pose.orientation.z;
    q.w() = gnss_odometry_msg.pose.pose.orientation.w;
    gnss_odometry_data.pose.block<3,3>(0,0) = q.matrix();

    // set the linear velocity:
    gnss_odometry_data.vel.x() = gnss_odometry_msg.twist.twist.linear.x;
    gnss_odometry_data.vel.y() = gnss_odometry_msg.twist.twist.linear.y;
    gnss_odometry_data.vel.z() = gnss_odometry_msg.twist.twist.linear.z;
}

void LidarMeasurementSubscriber::msg_callback(
    const LidarMeasurement::ConstPtr& synced_cloud_msg_ptr
) {
    buff_mutex_.lock();

    LidarMeasurementData synced_cloud_data;
    
    // parse header:
    synced_cloud_data.time = synced_cloud_msg_ptr->header.stamp.toSec();

    // a. parse lidar measurement:
    ParseCloudData(synced_cloud_msg_ptr->point_cloud, synced_cloud_data.point_cloud);

    // b. parse IMU measurement:
    ParseIMUData(synced_cloud_msg_ptr->imu, synced_cloud_data.imu);

    // c. parse GNSS odometry:
    ParsePoseData(synced_cloud_msg_ptr->gnss_odometry, synced_cloud_data.gnss_odometry);

    // add new message to buffer:
    new_cloud_data_.push_back(synced_cloud_data);

    buff_mutex_.unlock();
}

} // namespace lidar_localization