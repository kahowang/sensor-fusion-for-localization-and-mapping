/*
 * @Description: odometry data
 * @Author: Ge Yao
 * @Date: 2020-11-10 14:25:03
 */
#ifndef IMU_INTEGRATION_ODOM_DATA_HPP_
#define IMU_INTEGRATION_ODOM_DATA_HPP_

#include <Eigen/Dense>
#include <Eigen/Core>

namespace imu_integration {

struct OdomData {
    double time = 0.0;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
};

} // namespace imu_integration

#endif
