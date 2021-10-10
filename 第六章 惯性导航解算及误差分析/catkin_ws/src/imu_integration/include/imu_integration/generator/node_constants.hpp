#ifndef IMU_INTEGRATION_NODE_CONSTANTS_HPP_
#define IMU_INTEGRATION_NODE_CONSTANTS_HPP_

#include <cmath>
#include <ros/ros.h>

namespace imu_integration {

namespace generator {

constexpr double kRhoX = 3.0;
constexpr double kRhoY = 4.0;
constexpr double kRhoZ = 1.0;

constexpr double kOmegaXY = M_PI / 10.0;
constexpr double kOmegaZ = 10.0 * kOmegaXY;

constexpr double kYaw = M_PI / 10.0;
constexpr double kPitch = 0.20;
constexpr double kRoll = 0.10; 

}  // namespace generator

}  // namespace imu_integration

#endif  // IMU_INTEGRATION_NODE_CONSTANTS_HPP_