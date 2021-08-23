#ifndef  LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_CPU_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_CPU_REGISTRATION_HPP_

#include "lidar_localization/models/registration/registration_interface.hpp"
#include "lidar_localization/models/registration/ndt_cpu/NormalDistributionsTransform.h"

namespace lidar_localization {

class NDTCPURegistration: public RegistrationInterface {     // 继承点云配准的基类
  public:
    NDTCPURegistration(const YAML::Node&  node);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(
      const CloudData::CLOUD_PTR& input_source, 
      const Eigen::Matrix4f& predict_pose, 
      CloudData::CLOUD_PTR& result_cloud_ptr,
      Eigen::Matrix4f& result_pose
    ) override;

   private:
    bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

  private:
    cpu::NormalDistributionsTransform<CloudData::POINT,   CloudData::POINT>  ndt_cpu_;      // 实例化cpu_ndt 对象

};

} // namespace lidar_localization

#endif