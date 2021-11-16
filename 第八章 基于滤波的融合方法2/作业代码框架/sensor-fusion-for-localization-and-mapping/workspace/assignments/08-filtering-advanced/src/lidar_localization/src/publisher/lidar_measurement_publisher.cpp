/*
 * @Description: Publish synced Lidar-IMU-GNSS measurement
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */

#include "lidar_localization/publisher/lidar_measurement_publisher.hpp"
#include "glog/logging.h"

namespace lidar_localization {

LidarMeasurementPublisher::LidarMeasurementPublisher(
    ros::NodeHandle& nh,
    std::string topic_name,
    std::string lidar_frame_id,
    std::string imu_frame_id,
    std::string gnss_odometry_pos_frame_id,
    std::string gnss_odometry_vel_frame_id,
    size_t buff_size
) : nh_(nh), 
    lidar_frame_id_(lidar_frame_id),
    imu_frame_id_(imu_frame_id),
    gnss_odometry_pos_frame_id_(gnss_odometry_pos_frame_id), 
    gnss_odometry_vel_frame_id_(gnss_odometry_vel_frame_id)
{
    // set frame ids:
    lidar_measurement_.header.frame_id = lidar_frame_id;
    lidar_measurement_.point_cloud.header.frame_id = lidar_frame_id_;
    lidar_measurement_.imu.header.frame_id = imu_frame_id_;
    lidar_measurement_.gnss_odometry.header.frame_id = gnss_odometry_pos_frame_id_;
    lidar_measurement_.gnss_odometry.child_frame_id = gnss_odometry_vel_frame_id_;

    publisher_ = nh_.advertise<lidar_localization::LidarMeasurement>(topic_name, buff_size);
}

void LidarMeasurementPublisher::Publish(
    double time,
    const CloudData::CLOUD_PTR& cloud_data_ptr,
    const IMUData &imu_data,
    const Eigen::Matrix4f& transform_matrix, 
    const VelocityData &velocity_data 
) {
    ros::Time ros_time(time);
    PublishData(
        ros_time,
        cloud_data_ptr, 
        imu_data,
        transform_matrix,
        velocity_data
    );
}

void LidarMeasurementPublisher::Publish(
    const CloudData::CLOUD_PTR& cloud_data_ptr,
    const IMUData &imu_data,
    const Eigen::Matrix4f& transform_matrix, 
    const VelocityData &velocity_data
) {
    ros::Time time = ros::Time::now();
    PublishData(
        time,
        cloud_data_ptr, 
        imu_data,
        transform_matrix,
        velocity_data
    );
}

bool LidarMeasurementPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}

void LidarMeasurementPublisher::SetPointCloud(
    const ros::Time &time,
    const CloudData::CLOUD_PTR &cloud_data_ptr,
    sensor_msgs::PointCloud2 &point_cloud
) {
    // set point cloud:
    pcl::toROSMsg(*cloud_data_ptr, point_cloud);
    
    // set header:
    point_cloud.header.stamp = time;
}

void LidarMeasurementPublisher::SetIMU(
    const ros::Time &time,
    const IMUData &imu_data,
    sensor_msgs::Imu &imu
) {
    // set header:
    imu.header.stamp = time;

    // set orientation:
    imu.orientation.w = imu_data.orientation.w;
    imu.orientation.x = imu_data.orientation.x;
    imu.orientation.y = imu_data.orientation.y;
    imu.orientation.z = imu_data.orientation.z;

    // set angular velocity:
    imu.angular_velocity.x = imu_data.angular_velocity.x;
    imu.angular_velocity.y = imu_data.angular_velocity.y;
    imu.angular_velocity.z = imu_data.angular_velocity.z;

    // set linear acceleration:
    imu.linear_acceleration.x = imu_data.linear_acceleration.x;
    imu.linear_acceleration.y = imu_data.linear_acceleration.y;
    imu.linear_acceleration.z = imu_data.linear_acceleration.z;
}

void LidarMeasurementPublisher::SetGNSSOdometry(
    const ros::Time &time,
    const Eigen::Matrix4f& transform_matrix,
    const VelocityData &velocity_data, 
    nav_msgs::Odometry &gnss_odometry 
) {
    // set header:
    gnss_odometry.header.stamp = time;

    // set the pose
    gnss_odometry.pose.pose.position.x = transform_matrix(0,3);
    gnss_odometry.pose.pose.position.y = transform_matrix(1,3);
    gnss_odometry.pose.pose.position.z = transform_matrix(2,3);

    Eigen::Quaternionf q;
    q = transform_matrix.block<3,3>(0,0);
    gnss_odometry.pose.pose.orientation.x = q.x();
    gnss_odometry.pose.pose.orientation.y = q.y();
    gnss_odometry.pose.pose.orientation.z = q.z();
    gnss_odometry.pose.pose.orientation.w = q.w();

    // set the twist:
    gnss_odometry.twist.twist.linear.x = velocity_data.linear_velocity.x;
    gnss_odometry.twist.twist.linear.y = velocity_data.linear_velocity.y;
    gnss_odometry.twist.twist.linear.z = velocity_data.linear_velocity.z;

    gnss_odometry.twist.twist.angular.x = velocity_data.angular_velocity.x;
    gnss_odometry.twist.twist.angular.y = velocity_data.angular_velocity.y;
    gnss_odometry.twist.twist.angular.z = velocity_data.angular_velocity.z;
}

void LidarMeasurementPublisher::PublishData(
    const ros::Time &time,
    const CloudData::CLOUD_PTR &cloud_data_ptr,
    const IMUData &imu_data, 
    const Eigen::Matrix4f &transform_matrix, 
    const VelocityData &velocity_data
) {
    // set header:
    lidar_measurement_.header.stamp = time;
    
    // a. set lidar measurement:
    SetPointCloud(time, cloud_data_ptr, lidar_measurement_.point_cloud);

    // b. set IMU measurement:
    SetIMU(time, imu_data, lidar_measurement_.imu);

    // c. set GNSS odometry:
    SetGNSSOdometry(time, transform_matrix, velocity_data, lidar_measurement_.gnss_odometry);

    // publish synced lidar measurement:
    publisher_.publish(lidar_measurement_);
}

} // namespace lidar_localization