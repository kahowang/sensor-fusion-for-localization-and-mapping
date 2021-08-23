#pragma once

#include "lidar_localization/models/registration/registration_interface.hpp"

#include "lidar_localization/models/registration/ndt_cpu/NormalDistributionsTransform.h"

namespace lidar_localization {
class NDTCPURegistration : public RegistrationInterface {
public:
    NDTCPURegistration(const YAML::Node& node);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;

    bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                const Eigen::Matrix4f& predict_pose, 
                CloudData::CLOUD_PTR& result_cloud_ptr,
                Eigen::Matrix4f& result_pose) override;

private:
    cpu::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT> ndt_cpu_;

    // 配准参数
    double trans_eps_, step_size_, ndt_resolution_;
    int max_iter_;
};

}