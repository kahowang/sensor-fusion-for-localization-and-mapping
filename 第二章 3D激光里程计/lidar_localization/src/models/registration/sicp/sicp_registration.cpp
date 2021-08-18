/*
 * @Description: ICP SVD lidar odometry
 * @Author: Ge Yao
 * @Date: 2020-10-24 21:46:45
 */

#include <pcl/common/transforms.h>

#include <Eigen/Dense>

#include "glog/logging.h"

#include "lidar_localization/models/registration/sicp/ICP.h"

#include "lidar_localization/models/registration/sicp/scip_registration.hpp"

namespace lidar_localization {

SICPRegistration::SICPRegistration(
    const YAML::Node& node
) {
    // parse params:
    /*
    params_.p = node['p'].as<float>();
    params_.mu = node['mu'].as<float>();
    params_.alpha = node['alpha'].as<float>();
    params_.max_mu = node['max_mu'].as<float>();
    params_.max_icp = node['max_icp'].as<int>();
    params_.max_outer = node['max_outer'].as<int>();
    params_.max_inner = node['max_inner'].as<int>();
    params_.stop = node['stop'].as<float>();
    */
}

bool SICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    input_target_ = input_target;

    return true;
}

bool SICPRegistration::ScanMatch(
    const CloudData::CLOUD_PTR& input_source, 
    const Eigen::Matrix4f& predict_pose, 
    CloudData::CLOUD_PTR& result_cloud_ptr,
    Eigen::Matrix4f& result_pose
) {
    input_source_ = input_source;

    // pre-process input source:
    CloudData::CLOUD_PTR transformed_input_source(new CloudData::CLOUD());
    pcl::transformPointCloud(*input_source_, *transformed_input_source, predict_pose);

    //
    // TODO: second option -- adapt existing implementation
    //
    // TODO: format inputs for SICP:
    
    // TODO: SICP registration:

    // set output:
    result_pose = transformation_ * predict_pose;
    pcl::transformPointCloud(*input_source_, *result_cloud_ptr, result_pose);
    
    return true;
}

} // namespace lidar_localization