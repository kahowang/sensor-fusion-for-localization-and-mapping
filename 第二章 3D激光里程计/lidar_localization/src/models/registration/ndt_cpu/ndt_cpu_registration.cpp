/*
 * @Description: NDT CPU lidar odometry
 * @Author: KaHo
 * @Date: 2021-8-22 
 */

#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include "glog/logging.h"

#include "lidar_localization/models/registration/ndt_cpu/ndt_cpu_registration.hpp"

namespace  lidar_localization{

NDTCPURegistration::NDTCPURegistration(const YAML::Node& node){
    float res = node["res"].as<float>();
    float step_size = node["step_size"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

bool NDTCPURegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter) {
    ndt_cpu_.setResolution(res);
    ndt_cpu_.setStepSize(step_size);
    ndt_cpu_.setTransformationEpsilon(trans_eps);
    ndt_cpu_.setMaximumIterations(max_iter);

    LOG(INFO) << "NDT params:" << std::endl
              << "res: " << res << ", "
              << "step_size: " << step_size << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter 
              << std::endl << std::endl;

    return true;
}

bool NDTCPURegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    ndt_cpu_.setInputTarget(input_target);

    return true;
}

bool NDTCPURegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    ndt_cpu_.setInputSource(input_source);
    ndt_cpu_.align(*result_cloud_ptr, predict_pose);
    result_pose = ndt_cpu_.getFinalTransformation();    // 匹配后的点云

    return true;
}

}