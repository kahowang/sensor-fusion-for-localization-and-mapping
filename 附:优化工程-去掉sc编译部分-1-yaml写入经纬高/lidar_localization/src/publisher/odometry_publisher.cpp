/*
 * @Description: 里程计信息发布
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:11:44
 */
#include "lidar_localization/publisher/odometry_publisher.hpp"

namespace lidar_localization {
OdometryPublisher::OdometryPublisher(ros::NodeHandle& nh, 
                                     std::string topic_name, 
                                     std::string base_frame_id,
                                     std::string child_frame_id,
                                     int buff_size)
    :nh_(nh) {

    publisher_ = nh_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
    odometry_.header.frame_id = base_frame_id;
    odometry_.child_frame_id = child_frame_id;
}

void OdometryPublisher::Publish(
    const Eigen::Matrix4f& transform_matrix, 
    double time
) {
    ros::Time ros_time(time);
    PublishData(transform_matrix, velocity_data_, ros_time);
}

void OdometryPublisher::Publish(
    const Eigen::Matrix4f& transform_matrix
) {
    PublishData(transform_matrix, velocity_data_, ros::Time::now());
}

void OdometryPublisher::Publish(
    const Eigen::Matrix4f& transform_matrix, 
    const VelocityData &velocity_data, 
    double time
) {
    ros::Time ros_time(time);
    PublishData(transform_matrix, velocity_data, ros_time);
}

void OdometryPublisher::Publish(
    const Eigen::Matrix4f& transform_matrix, 
    const VelocityData &velocity_data
) {
    PublishData(transform_matrix, velocity_data, ros::Time::now());
}

void OdometryPublisher::Publish(
    const Eigen::Matrix4f& transform_matrix, 
    const Eigen::Vector3f& vel, 
    double time
) {
    ros::Time ros_time(time);

    velocity_data_.linear_velocity.x = vel.x();
    velocity_data_.linear_velocity.y = vel.y();
    velocity_data_.linear_velocity.z = vel.z();
    
    PublishData(transform_matrix, velocity_data_, ros_time);

    velocity_data_.linear_velocity.x = velocity_data_.linear_velocity.y = velocity_data_.linear_velocity.z = 0.0;
}

void OdometryPublisher::Publish(
    const Eigen::Matrix4f& transform_matrix, 
    const Eigen::Vector3f& vel
) {
    velocity_data_.linear_velocity.x = vel.x();
    velocity_data_.linear_velocity.y = vel.y();
    velocity_data_.linear_velocity.z = vel.z();

    PublishData(transform_matrix, velocity_data_, ros::Time::now());

    velocity_data_.linear_velocity.x = velocity_data_.linear_velocity.y = velocity_data_.linear_velocity.z = 0.0;
}

void OdometryPublisher::PublishData(
    const Eigen::Matrix4f& transform_matrix,
    const VelocityData &velocity_data,  
    ros::Time time
) {
    odometry_.header.stamp = time;

    // set the pose
    odometry_.pose.pose.position.x = transform_matrix(0,3);
    odometry_.pose.pose.position.y = transform_matrix(1,3);
    odometry_.pose.pose.position.z = transform_matrix(2,3);

    Eigen::Quaternionf q;
    q = transform_matrix.block<3,3>(0,0);
    odometry_.pose.pose.orientation.x = q.x();
    odometry_.pose.pose.orientation.y = q.y();
    odometry_.pose.pose.orientation.z = q.z();
    odometry_.pose.pose.orientation.w = q.w();

    // set the twist:
    odometry_.twist.twist.linear.x = velocity_data.linear_velocity.x;
    odometry_.twist.twist.linear.y = velocity_data.linear_velocity.y;
    odometry_.twist.twist.linear.z = velocity_data.linear_velocity.z;

    odometry_.twist.twist.angular.x = velocity_data.angular_velocity.x;
    odometry_.twist.twist.angular.y = velocity_data.angular_velocity.y;
    odometry_.twist.twist.angular.z = velocity_data.angular_velocity.z;

    publisher_.publish(odometry_);
}

bool OdometryPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
}