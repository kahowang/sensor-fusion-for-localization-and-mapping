// #include <fast_gicp/gicp/fast_vgicp.hpp>
// #include <fast_gicp/gicp/impl/fast_vgicp_impl.hpp>
#include "lidar_localization/models/registration/fast_gicp/fast_vgicp.hpp"
#include "lidar_localization/models/registration/fast_gicp/impl/fast_vgicp_impl.hpp"

template class fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ>;
template class fast_gicp::FastVGICP<pcl::PointXYZI, pcl::PointXYZI>;
