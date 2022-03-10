/*
 * @Description:
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:17:49
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_LIDAR_MEASUREMENT_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_LIDAR_MEASUREMENT_DATA_HPP_

#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/sensor_data/imu_data.hpp"
#include "lidar_localization/sensor_data/pose_data.hpp"

namespace lidar_localization {

class LidarMeasurementData {
  public:
    double time = 0.0;

    CloudData point_cloud;
    IMUData imu;
    PoseData gnss_odometry;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_SENSOR_DATA_LIDAR_MEASUREMENT_DATA_HPP_