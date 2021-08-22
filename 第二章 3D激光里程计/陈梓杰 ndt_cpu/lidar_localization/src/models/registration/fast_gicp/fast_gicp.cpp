// #include <fast_gicp/gicp/fast_gicp.hpp>
// #include <fast_gicp/gicp/impl/fast_gicp_impl.hpp>
#include "lidar_localization/models/registration/fast_gicp/fast_gicp.hpp"
#include "lidar_localization/models/registration/fast_gicp/impl/fast_gicp_impl.hpp"

template class fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>;
template class fast_gicp::FastGICP<pcl::PointXYZI, pcl::PointXYZI>;
