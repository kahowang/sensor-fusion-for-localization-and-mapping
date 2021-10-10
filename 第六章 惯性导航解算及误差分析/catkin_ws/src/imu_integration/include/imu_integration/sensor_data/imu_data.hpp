/*
 * @Description: IMU data
 * @Author: Ge Yao
 * @Date: 2020-11-10 14:25:03
 */
#ifndef IMU_INTEGRATION_IMU_DATA_HPP_
#define IMU_INTEGRATION_IMU_DATA_HPP_

#include <Eigen/Dense>
#include <Eigen/Core>

namespace imu_integration {

struct IMUData {
    double time = 0.0;
    Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

} // namespace imu_integration

#endif
