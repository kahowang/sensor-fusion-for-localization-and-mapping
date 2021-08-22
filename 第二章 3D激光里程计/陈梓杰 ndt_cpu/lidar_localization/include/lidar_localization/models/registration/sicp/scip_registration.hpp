/*
 * @Description: SICP registration 
 * @Author: Ge Yao
 * @Date: 2021-04-24 08:46:57
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_SICP_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_SICP_REGISTRATION_HPP_

#include "lidar_localization/models/registration/registration_interface.hpp"
// #include "lidar_localization/models/registration/sicp/ICP.h"

namespace lidar_localization {

struct SICPParams{
  double p;
  double mu;
  double alpha;
  double max_mu;
  int max_icp;
  int max_outer;
  int max_inner;
  double stop;
};

class SICPRegistration: public RegistrationInterface {
  public:
    SICPRegistration(const YAML::Node& node);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(
      const CloudData::CLOUD_PTR& input_source, 
      const Eigen::Matrix4f& predict_pose, 
      CloudData::CLOUD_PTR& result_cloud_ptr,
      Eigen::Matrix4f& result_pose
    ) override;

  private:
    CloudData::CLOUD_PTR input_target_;
    CloudData::CLOUD_PTR input_source_;

    Eigen::Matrix4f transformation_;

    // SICP::Parameters params_;

    SICPParams params_;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MODELS_REGISTRATION_SICP_REGISTRATION_HPP_