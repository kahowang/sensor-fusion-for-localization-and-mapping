/*
 * @Description: IMU measurement generation activity
 * @Author: Ge Yao
 * @Date: 2020-11-10 14:25:03
 */
#ifndef IMU_INTEGRATION_GENERATOR_ACTIVITY_HPP_
#define IMU_INTEGRATION_GENERATOR_ACTIVITY_HPP_

#include <random>
#include <string>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include "imu_integration/config/config.hpp"

namespace imu_integration {

namespace generator {

class Activity {
public:
    Activity();
    void Init(void);
    void Run(void);
private:
    // get groud truth from motion equation:
    void GetGroundTruth(void);
    // random walk & measurement noise generation:
    void AddNoise(double delta_t);
    // convert to ROS messages:
    void SetIMUMessage(void);
    void SetOdometryMessage(void);
    // publish:
    void PublishMessages(void);

    // utilities:
    Eigen::Vector3d GetGaussianNoise(double stddev);
    static Eigen::Matrix3d EulerAnglesToRotation(const Eigen::Vector3d &euler_angles);
    static Eigen::Vector3d EulerAngleRatesToBodyAngleRates(const Eigen::Vector3d &euler_angles, const Eigen::Vector3d &euler_angle_rates);
    
    // node handler:
    ros::NodeHandle private_nh_;

    ros::Publisher pub_imu_;
    // TODO: separate odometry estimation from IMU device
    ros::Publisher pub_odom_;

    // config:
    IMUConfig imu_config_;
    OdomConfig odom_config_;

    // noise generator:
    std::default_random_engine normal_generator_;
    std::normal_distribution<double> normal_distribution_;

    // measurements:
    ros::Time timestamp_;
    // a. gravity constant:
    Eigen::Vector3d G_;
    // b. pose:
    Eigen::Matrix3d R_gt_;
    Eigen::Vector3d t_gt_;
    Eigen::Vector3d v_gt_;
    // c. angular velocity:
    Eigen::Vector3d angular_vel_;
    Eigen::Vector3d angular_vel_bias_;
    // d. linear acceleration:
    Eigen::Vector3d linear_acc_;
    Eigen::Vector3d linear_acc_bias_;
    // ROS IMU message:
    sensor_msgs::Imu message_imu_;
    nav_msgs::Odometry message_odom_;
};

}  // namespace generator

}  // namespace imu_integration

#endif  // IMU_INTEGRATION_GENERATOR_ACTIVITY_HPP_