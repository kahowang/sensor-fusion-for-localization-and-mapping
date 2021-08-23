#pragma once

#include "lidar_localization/models/registration/registration_interface.hpp"

#include "lidar_localization/models/registration/fast_gicp/fast_gicp.hpp"

#include "lidar_localization/models/registration/fast_gicp/fast_vgicp.hpp"

namespace lidar_localization{

class FASTVGICPRegistration : public RegistrationInterface{
public:
    FASTVGICPRegistration(const YAML::Node& node);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
private:
    int num_threads_;
    double max_correspondence_dis_;
    fast_gicp::FastVGICP<CloudData::POINT, CloudData::POINT> fast_vgicp_;
};

} // namespace lidar_localization

