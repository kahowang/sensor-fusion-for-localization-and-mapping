#include "lidar_localization/models/registration/omp_ndt/omp_ndt_registration.hpp"

#include <ros/ros.h>

namespace lidar_localization{
    
OMPNDTRegistration::OMPNDTRegistration(const YAML::Node& node) : 
    omp_ndt_ptr_(new pclomp::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()){

    resolution_ = node["resolution"].as<double>();
    num_threads_ = node["num_threads"].as<int>();
    search_method_ = node["search_method"].as<int>();
    step_size_ = node["step_size"].as<double>();
    epsilon_ = node["epsilon"].as<double>();
    max_iter_ = node["max_iter"].as<int>();
    
    omp_ndt_ptr_->setResolution(resolution_);
    if(num_threads_ == 0)
        num_threads_ = omp_get_max_threads();
    omp_ndt_ptr_->setNumThreads(num_threads_);
    omp_ndt_ptr_->setStepSize(step_size_);
    omp_ndt_ptr_->setTransformationEpsilon(epsilon_);
    omp_ndt_ptr_->setMaximumIterations(max_iter_);

    ROS_INFO("resolution: %f", resolution_);
    ROS_INFO("num_thread: %d", num_threads_);
    ROS_INFO("max_iter: %d", max_iter_);

    if(search_method_ == 1){
        omp_ndt_ptr_->setNeighborhoodSearchMethod(pclomp::KDTREE);
        ROS_INFO("search_method: KDTREE");
    } else if(search_method_ == 2){
        omp_ndt_ptr_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        ROS_INFO("search_method: DIRECT7");
    } else if(search_method_ == 3){
        omp_ndt_ptr_->setNeighborhoodSearchMethod(pclomp::DIRECT1);
        ROS_INFO("search_method: DIRECT1");
    } else{
        ROS_WARN(" search method is warn ...");
        ros::shutdown();
    }

}

bool OMPNDTRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target){
    omp_ndt_ptr_->setInputTarget(input_target);
    return true;
}

bool OMPNDTRegistration::ScanMatch(
      const CloudData::CLOUD_PTR& input_source, 
      const Eigen::Matrix4f& predict_pose, 
      CloudData::CLOUD_PTR& result_cloud_ptr,
      Eigen::Matrix4f& result_pose
    ){
        omp_ndt_ptr_->setInputSource(input_source);
        omp_ndt_ptr_->align(*result_cloud_ptr, predict_pose);
        result_pose = omp_ndt_ptr_->getFinalTransformation();

        ROS_INFO("iter: %d", omp_ndt_ptr_->getFinalNumIteration());
        return true;
    }


} // namespace lidar_localization
