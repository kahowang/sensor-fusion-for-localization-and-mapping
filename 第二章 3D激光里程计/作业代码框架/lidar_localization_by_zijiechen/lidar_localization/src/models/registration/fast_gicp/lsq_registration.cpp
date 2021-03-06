// #include <fast_gicp/gicp/lsq_registration.hpp>
// #include <fast_gicp/gicp/impl/lsq_registration_impl.hpp>
#include "lidar_localization/models/registration/fast_gicp/lsq_registration.hpp"
#include "lidar_localization/models/registration/fast_gicp/impl/lsq_registration_impl.hpp"

template class fast_gicp::LsqRegistration<pcl::PointXYZ, pcl::PointXYZ>;
template class fast_gicp::LsqRegistration<pcl::PointXYZI, pcl::PointXYZI>;
