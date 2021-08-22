#include"lidar_localization/models/registration/fast_gicp/fast_vgicp_registration.hpp"

namespace lidar_localization{

FASTVGICPRegistration::FASTVGICPRegistration(const YAML::Node& node){ 
    
    num_threads_ = node["num_threads"].as<int>();
    max_correspondence_dis_ = node["max_correspondence_dis"].as<double>();

    fast_vgicp_.setNumThreads(num_threads_);
    fast_vgicp_.setMaxCorrespondenceDistance(max_correspondence_dis_);
}

bool FASTVGICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target){
    fast_vgicp_.setInputTarget(input_target);
    return true;
}

bool FASTVGICPRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose){
    
    fast_vgicp_.setInputSource(input_source);
    fast_vgicp_.align(*result_cloud_ptr, predict_pose);
    result_pose = fast_vgicp_.getFinalTransformation();
}



}