/*
 * @Description: synced GNSS-Odo-Mag measurements as PosVelMagData
 * @Author: Ge Yao
 * @Date: 2020-11-21 15:39:24
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_POS_VEL_MAG_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_POS_VEL_MAG_DATA_HPP_

#include <string>

#include <Eigen/Dense>

namespace lidar_localization {

class PosVelMagData {
  public:
    double time = 0.0;

    Eigen::Vector3f pos = Eigen::Vector3f::Zero();
    Eigen::Vector3f vel = Eigen::Vector3f::Zero();
    Eigen::Vector3f mag = Eigen::Vector3f::Zero();
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_SENSOR_DATA_POS_VEL_MAG_DATA_HPP_