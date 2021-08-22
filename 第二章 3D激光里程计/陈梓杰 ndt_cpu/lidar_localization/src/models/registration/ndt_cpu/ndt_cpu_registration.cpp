#include "lidar_localization/models/registration/ndt_cpu/ndt_cpu_registration.hpp"

#include <ros/ros.h>

namespace lidar_localization {

NDTCPURegistration::NDTCPURegistration(const YAML::Node& node){
    trans_eps_ = node["trans_eps"].as<double>();
    step_size_ = node["step_size"].as<double>();
    ndt_resolution_ = node["ndt_resolution"].as<double>();
    max_iter_ = node["max_iter"].as<int>();

    ROS_INFO("trans_eps: %f", trans_eps_);
    ROS_INFO("step_size: %f", step_size_);
    ROS_INFO("ndt_res: %f", ndt_resolution_);
    ROS_INFO("max_iter: %d", max_iter_);

    ndt_cpu_.setTransformationEpsilon(trans_eps_);
    ndt_cpu_.setResolution(ndt_resolution_);
    ndt_cpu_.setStepSize(step_size_);
    ndt_cpu_.setMaximumIterations(max_iter_);

    
}

bool NDTCPURegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target){
    ndt_cpu_.setInputTarget(input_target);
    return true;
}

bool NDTCPURegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                    const Eigen::Matrix4f& predict_pose, 
                                    CloudData::CLOUD_PTR& result_cloud_ptr,
                                    Eigen::Matrix4f& result_pose){
        
    ndt_cpu_.setInputSource(input_source);
    // 配准后的点云, initial guess
    ndt_cpu_.align(*result_cloud_ptr, predict_pose);
    ROS_INFO("iter: %d", ndt_cpu_.getFinalNumIteration());
    result_pose = ndt_cpu_.getFinalTransformation();

    return true;
}



}
