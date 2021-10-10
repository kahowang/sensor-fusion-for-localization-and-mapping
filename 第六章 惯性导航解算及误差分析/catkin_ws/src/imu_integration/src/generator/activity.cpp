/*
 * @Description: IMU measurement generation activity
 * @Author: Ge Yao
 * @Date: 2020-11-10 14:25:03
 */
#include "imu_integration/generator/node_constants.hpp"
#include "imu_integration/generator/activity.hpp"
#include "glog/logging.h"

#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <math.h>

namespace imu_integration {

namespace generator {

Activity::Activity(void) 
    : private_nh_("~"), 
    // standard normal distribution:
    normal_distribution_(0.0, 1.0),
    // gravity acceleration:
    G_(0, 0, -9.81),
    // angular velocity bias:
    angular_vel_bias_(0.0, 0.0, 0.0),
    // linear acceleration bias:
    linear_acc_bias_(0.0, 0.0, 0.0)
{}

void Activity::Init(void) {
    // parse IMU config:
    private_nh_.param("imu/device_name", imu_config_.device_name, std::string("GNSS_INS_SIM_IMU"));
    private_nh_.param("imu/topic_name", imu_config_.topic_name, std::string("/sim/sensor/imu"));
    private_nh_.param("imu/frame_id", imu_config_.frame_id, std::string("ENU"));

    // a. gravity constant:
    private_nh_.param("imu/gravity/x", imu_config_.gravity.x,  0.0);
    private_nh_.param("imu/gravity/y", imu_config_.gravity.y,  0.0);
    private_nh_.param("imu/gravity/z", imu_config_.gravity.z, -9.81);
    G_.x() = imu_config_.gravity.x;
    G_.y() = imu_config_.gravity.y;
    G_.z() = imu_config_.gravity.z;

    // b. angular velocity bias:
    private_nh_.param("imu/bias/angular_velocity/x", imu_config_.bias.angular_velocity.x,  0.0);
    private_nh_.param("imu/bias/angular_velocity/y", imu_config_.bias.angular_velocity.y,  0.0);
    private_nh_.param("imu/bias/angular_velocity/z", imu_config_.bias.angular_velocity.z,  0.0);
    angular_vel_bias_.x() = imu_config_.bias.angular_velocity.x;
    angular_vel_bias_.y() = imu_config_.bias.angular_velocity.y;
    angular_vel_bias_.z() = imu_config_.bias.angular_velocity.z;

    // c. linear acceleration bias:
    private_nh_.param("imu/bias/linear_acceleration/x", imu_config_.bias.linear_acceleration.x,  0.0);
    private_nh_.param("imu/bias/linear_acceleration/y", imu_config_.bias.linear_acceleration.y,  0.0);
    private_nh_.param("imu/bias/linear_acceleration/z", imu_config_.bias.linear_acceleration.z,  0.0);
    linear_acc_bias_.x() = imu_config_.bias.linear_acceleration.x;
    linear_acc_bias_.y() = imu_config_.bias.linear_acceleration.y;
    linear_acc_bias_.z() = imu_config_.bias.linear_acceleration.z;

    // d. angular velocity random noise:
    private_nh_.param("imu/gyro/sigma_bias", imu_config_.gyro_bias_stddev, 5e-5);
    private_nh_.param("imu/gyro/sigma_noise", imu_config_.gyro_noise_stddev, 0.015);

    // e. linear acceleration random noise:
    private_nh_.param("imu/acc/sigma_bias", imu_config_.acc_bias_stddev, 5e-4);
    private_nh_.param("imu/acc/sigma_noise", imu_config_.acc_noise_stddev, 0.019);

    // parse odom config:
    private_nh_.param("pose/frame_id", odom_config_.frame_id, std::string("inertial"));
    private_nh_.param("pose/topic_name", odom_config_.topic_name.ground_truth, std::string("/pose/ground_truth"));

    // init publishers:
    pub_imu_ = private_nh_.advertise<sensor_msgs::Imu>(imu_config_.topic_name, 500);
    pub_odom_ = private_nh_.advertise<nav_msgs::Odometry>(odom_config_.topic_name.ground_truth, 500);

    // init timestamp:
    timestamp_ = ros::Time::now();
}

void Activity::Run(void) {
    // update timestamp:
    ros::Time timestamp = ros::Time::now();
    double delta_t = timestamp.toSec() - timestamp_.toSec();
    timestamp_ = timestamp;

    // get ground truth from motion equation:
    GetGroundTruth();
    // update bias & add measurement noises:
    AddNoise(delta_t);
    // convert to ROS messages:
    SetIMUMessage();
    SetOdometryMessage();
    // publish ROS messages:
    PublishMessages();
}

void Activity::GetGroundTruth(void) {
    // acceleration:
    double timestamp_in_sec = timestamp_.toSec();
    double sin_w_xy_t = sin(kOmegaXY*timestamp_in_sec);
    double cos_w_xy_t = cos(kOmegaXY*timestamp_in_sec);
    double sin_w_z_t = sin(kOmegaZ*timestamp_in_sec);
    double cos_w_z_t = cos(kOmegaZ*timestamp_in_sec);
    double rho_x_w_xy = kRhoX*kOmegaXY;
    double rho_y_w_xy = kRhoY*kOmegaXY;
    double rho_z_w_z = kRhoZ*kOmegaZ;

    Eigen::Vector3d p(
        kRhoX*cos_w_xy_t, 
        kRhoY*sin_w_xy_t, 
        kRhoZ*sin_w_z_t
    );
    Eigen::Vector3d v(
        -rho_x_w_xy*sin_w_xy_t, 
         rho_y_w_xy*cos_w_xy_t, 
         rho_z_w_z*cos_w_z_t
    );
    Eigen::Vector3d a(
        -rho_x_w_xy*kOmegaXY*cos_w_xy_t, 
        -rho_y_w_xy*kOmegaXY*sin_w_xy_t, 
        -rho_z_w_z*kOmegaZ*sin_w_z_t
    );

    // angular velocity:
    double sin_t = sin(timestamp_in_sec);
    double cos_t = cos(timestamp_in_sec);

    Eigen::Vector3d euler_angles(
        kRoll*cos_t,
        kPitch*sin_t,
        kYaw*timestamp_in_sec
    );

    Eigen::Vector3d euler_angle_rates(
        -kRoll*sin_t,
        kPitch*cos_t,
        kYaw
    );

    // transform to body frame:
    R_gt_ = EulerAnglesToRotation(euler_angles);
    t_gt_ = p;
    v_gt_ = v;
    // a. angular velocity:
    angular_vel_ = EulerAngleRatesToBodyAngleRates(euler_angles, euler_angle_rates);
    // b. linear acceleration:
    linear_acc_ = R_gt_.transpose() * (a + G_);
}

void Activity::AddNoise(double delta_t) {
    // TODO: add params to class attributes:
    double sqrt_delta_t = sqrt(delta_t);

    // a. update bias:
    angular_vel_bias_ += GetGaussianNoise(imu_config_.gyro_bias_stddev * sqrt_delta_t);
    linear_acc_bias_ += GetGaussianNoise(imu_config_.acc_bias_stddev * sqrt_delta_t);

    // b. get measurement noise:
    Eigen::Vector3d angular_vel_noise = GetGaussianNoise(imu_config_.gyro_noise_stddev / sqrt_delta_t);
    Eigen::Vector3d linear_acc_noise = GetGaussianNoise(imu_config_.acc_noise_stddev / sqrt_delta_t);

    // apply to measurement:
    angular_vel_ += angular_vel_bias_ + angular_vel_noise;
    linear_acc_ += linear_acc_bias_ + linear_acc_noise;
}

void Activity::SetIMUMessage(void) {
    // a. set header:
    message_imu_.header.stamp = timestamp_;
    message_imu_.header.frame_id = imu_config_.frame_id;

    // b. set orientation:
    Eigen::Quaterniond q(R_gt_);
    message_imu_.orientation.x = q.x();
    message_imu_.orientation.y = q.y();
    message_imu_.orientation.z = q.z();
    message_imu_.orientation.w = q.w();
    // c. set angular velocity:
    message_imu_.angular_velocity.x = angular_vel_.x(); 
    message_imu_.angular_velocity.y = angular_vel_.y(); 
    message_imu_.angular_velocity.z = angular_vel_.z();
    // d. set linear acceleration:
    message_imu_.linear_acceleration.x = linear_acc_.x(); 
    message_imu_.linear_acceleration.y = linear_acc_.y();
    message_imu_.linear_acceleration.z = linear_acc_.z();
}

void Activity::SetOdometryMessage(void) {
    // a. set header:
    message_odom_.header.stamp = timestamp_;
    message_odom_.header.frame_id = odom_config_.frame_id;
    
    // b. set child frame id:
    message_odom_.child_frame_id = odom_config_.frame_id;

    // b. set orientation:
    Eigen::Quaterniond q(R_gt_);
    message_odom_.pose.pose.orientation.x = q.x();
    message_odom_.pose.pose.orientation.y = q.y();
    message_odom_.pose.pose.orientation.z = q.z();
    message_odom_.pose.pose.orientation.w = q.w();

    // c. set position:
    message_odom_.pose.pose.position.x = t_gt_.x();
    message_odom_.pose.pose.position.y = t_gt_.y();
    message_odom_.pose.pose.position.z = t_gt_.z();  

    // d. set velocity:
    message_odom_.twist.twist.linear.x = v_gt_.x();
    message_odom_.twist.twist.linear.y = v_gt_.y();
    message_odom_.twist.twist.linear.z = v_gt_.z(); 
}

void Activity::PublishMessages(void) {
    pub_imu_.publish(message_imu_);
    pub_odom_.publish(message_odom_);
}

Eigen::Vector3d Activity::GetGaussianNoise(double stddev) {
    return stddev * Eigen::Vector3d(
        normal_distribution_(normal_generator_),
        normal_distribution_(normal_generator_),
        normal_distribution_(normal_generator_)
    );
}

Eigen::Matrix3d Activity::EulerAnglesToRotation(
    const Eigen::Vector3d &euler_angles
) {
    // parse Euler angles:
    double roll = euler_angles.x();
    double pitch = euler_angles.y();
    double yaw = euler_angles.z();

    double cr =  cos(roll); double sr =  sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);
    double cy =   cos(yaw); double sy =   sin(yaw);

    Eigen::Matrix3d R_ib;
    
    R_ib << 
        cy*cp,  cy*sp*sr - sy*cr,   sy*sr + cy* cr*sp,
        sy*cp, cy *cr + sy*sr*sp,    sp*sy*cr - cy*sr,
          -sp,             cp*sr,               cp*cr;

    return R_ib;
}

Eigen::Vector3d Activity::EulerAngleRatesToBodyAngleRates(
    const Eigen::Vector3d &euler_angles, 
    const Eigen::Vector3d &euler_angle_rates
) {
    // parse euler angles:
    double roll = euler_angles(0);
    double pitch = euler_angles(1);

    double cr =  cos(roll); double sr =  sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);

    Eigen::Matrix3d R;

    R <<  
        1,     0,    - sp,
        0,    cr,   sr*cp,
        0,   -sr,   cr*cp;

    return R * euler_angle_rates;
}

}  // namespace generator

}  // namespace imu_integration