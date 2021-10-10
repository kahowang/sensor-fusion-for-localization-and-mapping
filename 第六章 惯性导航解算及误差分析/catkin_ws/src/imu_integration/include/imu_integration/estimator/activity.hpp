/*
 * @Description: IMU integration activity
 * @Author: Ge Yao
 * @Date: 2020-11-10 14:25:03
 */
#ifndef IMU_INTEGRATION_ACTIVITY_HPP_
#define IMU_INTEGRATION_ACTIVITY_HPP_

// common:
#include <ros/ros.h>

#include <Eigen/Dense>
#include <Eigen/Core>

// config:
#include "imu_integration/config/config.hpp"

// subscribers:
#include "imu_integration/subscriber/imu_subscriber.hpp"
#include "imu_integration/subscriber/odom_subscriber.hpp"

#include <nav_msgs/Odometry.h>

namespace imu_integration {

namespace estimator {

class Activity {
  public:
    Activity(void);
    void Init(void);
    bool Run(void);
  private:
    // workflow:
    bool ReadData(void);
    bool HasData(void);
    bool UpdatePose(void);
    bool PublishPose(void);

    // utils:
    /**
     * @brief  get unbiased angular velocity in body frame
     * @param  angular_vel, angular velocity measurement
     * @return unbiased angular velocity in body frame
     */
    Eigen::Vector3d GetUnbiasedAngularVel(const Eigen::Vector3d &angular_vel);
    /**
     * @brief  get unbiased linear acceleration in navigation frame
     * @param  linear_acc, linear acceleration measurement
     * @param  R, corresponding orientation of measurement
     * @return unbiased linear acceleration in navigation frame
     */
    Eigen::Vector3d GetUnbiasedLinearAcc(
        const Eigen::Vector3d &linear_acc,
        const Eigen::Matrix3d &R
    );
    /**
     * @brief  get angular delta
     * @param  index_curr, current imu measurement buffer index
     * @param  index_prev, previous imu measurement buffer index
     * @param  angular_delta, angular delta output
     * @return true if success false otherwise
     */
    bool GetAngularDelta(
        const size_t index_curr, const size_t index_prev,
        Eigen::Vector3d &angular_delta
    );
    /**
     * @brief  get velocity delta
     * @param  index_curr, current imu measurement buffer index
     * @param  index_prev, previous imu measurement buffer index
     * @param  R_curr, corresponding orientation of current imu measurement
     * @param  R_prev, corresponding orientation of previous imu measurement
     * @param  velocity_delta, velocity delta output
     * @return true if success false otherwise
     */
    bool GetVelocityDelta(
        const size_t index_curr, const size_t index_prev,
        const Eigen::Matrix3d &R_curr, const Eigen::Matrix3d &R_prev, 
        double &delta_t, Eigen::Vector3d &velocity_delta
    );
    /**
     * @brief  update orientation with effective rotation angular_delta
     * @param  angular_delta, effective rotation
     * @param  R_curr, current orientation
     * @param  R_prev, previous orientation
     * @return void
     */
    void UpdateOrientation(
        const Eigen::Vector3d &angular_delta,
        Eigen::Matrix3d &R_curr, Eigen::Matrix3d &R_prev
    );
    /**
     * @brief  update orientation with effective velocity change velocity_delta
     * @param  velocity_delta, effective velocity change
     * @return void
     */
    void UpdatePosition(const double &delta_t, const Eigen::Vector3d &velocity_delta);

  private:
    // node handler:
    ros::NodeHandle private_nh_;

    // subscriber:
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<OdomSubscriber> odom_ground_truth_sub_ptr;
    ros::Publisher odom_estimation_pub_;

    // data buffer:
    std::deque<IMUData> imu_data_buff_;
    std::deque<OdomData> odom_data_buff_;

    // config:
    bool initialized_ = false;

    IMUConfig imu_config_;
    OdomConfig odom_config_;

    // a. gravity constant:
    Eigen::Vector3d G_;
    // b. angular velocity:
    Eigen::Vector3d angular_vel_bias_;
    // c. linear acceleration:
    Eigen::Vector3d linear_acc_bias_;

    // IMU pose estimation:
    Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();
    Eigen::Vector3d vel_ = Eigen::Vector3d::Zero();

    nav_msgs::Odometry message_odom_;
};

} // namespace estimator

} // namespace imu_integration

#endif 