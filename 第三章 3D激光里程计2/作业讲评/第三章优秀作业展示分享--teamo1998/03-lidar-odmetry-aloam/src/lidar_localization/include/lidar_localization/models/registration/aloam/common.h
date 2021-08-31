#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_COMMON_H_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_COMMON_H_
#include <cmath>

#include <pcl/point_types.h>

namespace lidar_localization {

inline double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}
}
#endif