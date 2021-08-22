#include"lidar_localization/models/registration/fast_gicp/fast_gicp_registration.hpp"

namespace lidar_localization{

FASTGICPRegistration::FASTGICPRegistration(const YAML::Node& node){ 
    
    num_threads_ = node["num_threads"].as<int>();
    max_correspondence_dis_ = node["max_correspondence_dis"].as<double>();

    fast_gicp_.setNumThreads(num_threads_);
    fast_gicp_.setMaxCorrespondenceDistance(max_correspondence_dis_);
}

bool FASTGICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target){
    fast_gicp_.setInputTarget(input_target);
    return true;
}

bool FASTGICPRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose){
    
    fast_gicp_.setInputSource(input_source);
    fast_gicp_.align(*result_cloud_ptr, predict_pose);
    result_pose = fast_gicp_.getFinalTransformation();
}



}