# pragma once

#include "lidar_localization/models/registration/omp_ndt/ndt_omp.h"
#include "lidar_localization/models/registration/registration_interface.hpp"


namespace lidar_localization {

class OMPNDTRegistration : public RegistrationInterface {
public:
    OMPNDTRegistration(const YAML::Node& node);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(
      const CloudData::CLOUD_PTR& input_source, 
      const Eigen::Matrix4f& predict_pose, 
      CloudData::CLOUD_PTR& result_cloud_ptr,
      Eigen::Matrix4f& result_pose
    ) override;

private:
    double resolution_;
    double step_size_;
    double epsilon_;
    int num_threads_;
    int search_method_;
    int max_iter_;
    

    pclomp::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr omp_ndt_ptr_;
};
}