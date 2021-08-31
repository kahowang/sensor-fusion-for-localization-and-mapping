/*
 * @Description: ICP SVD lidar odometry
 * @Author: Ge Yao
 * @Date: 2020-10-24 21:46:57
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_SVD_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_SVD_REGISTRATION_HPP_

#include "lidar_localization/models/registration/registration_interface.hpp"

#include <pcl/kdtree/kdtree_flann.h>

namespace lidar_localization {

class ICPSVDRegistration: public RegistrationInterface {
  public:
    ICPSVDRegistration(const YAML::Node& node);
    ICPSVDRegistration(
      float max_corr_dist, 
      float trans_eps, 
      float euc_fitness_eps, 
      int max_iter
    );

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(
      const CloudData::CLOUD_PTR& input_source, 
      const Eigen::Matrix4f& predict_pose, 
      CloudData::CLOUD_PTR& result_cloud_ptr,
      Eigen::Matrix4f& result_pose
    ) override;
  
  private:
    bool SetRegistrationParam(
      float max_corr_dist, 
      float trans_eps, 
      float euc_fitness_eps, 
      int max_iter
    );

  private:
    size_t GetCorrespondence(
      const CloudData::CLOUD_PTR &input_source, 
      std::vector<Eigen::Vector3f> &xs,
      std::vector<Eigen::Vector3f> &ys
    );

    void GetTransform(
      const std::vector<Eigen::Vector3f> &xs,
      const std::vector<Eigen::Vector3f> &ys,
      Eigen::Matrix4f &transformation_
    );

    bool IsSignificant(
      const Eigen::Matrix4f &transformation,
      const float trans_eps
    );

    float max_corr_dist_;
    float trans_eps_; 
    float euc_fitness_eps_; 
    int max_iter_;

    CloudData::CLOUD_PTR input_target_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr input_target_kdtree_;
    CloudData::CLOUD_PTR input_source_;

    Eigen::Matrix4f transformation_;
};

}

#endif