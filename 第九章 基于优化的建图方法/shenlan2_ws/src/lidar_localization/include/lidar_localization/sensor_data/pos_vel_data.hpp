/*
 * @Description: synced GNSS-odo measurements as PosVelData
 * @Author: Ge Yao
 * @Date: 2020-11-21 15:39:24
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_POS_VEL_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_POS_VEL_DATA_HPP_

#include <string>

#include <Eigen/Dense>

namespace lidar_localization {

class PosVelData {
  public:
    double time = 0.0;

    Eigen::Vector3f pos = Eigen::Vector3f::Zero();
    Eigen::Vector3f vel = Eigen::Vector3f::Zero();
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_SENSOR_DATA_POS_VEL_DATA_HPP_