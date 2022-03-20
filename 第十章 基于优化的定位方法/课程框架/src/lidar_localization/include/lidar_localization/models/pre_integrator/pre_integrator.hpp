/*
 * @Description: pre-integrator interface
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */

#ifndef LIDAR_LOCALIZATION_MODELS_PRE_INTEGRATOR_PRE_INTEGRATOR_HPP_
#define LIDAR_LOCALIZATION_MODELS_PRE_INTEGRATOR_PRE_INTEGRATOR_HPP_

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace lidar_localization {

class PreIntegrator {
public:
    /**
     * @brief  whether the pre-integrator is inited:
     * @return true if inited false otherwise   
     */
    double IsInited(void) const { return is_inited_; }

    /**
     * @brief  get pre-integrator time
     * @return pre-integrator time as double    
     */
    double GetTime(void) const { return time_; }
    
protected:
    PreIntegrator() {}

    // init:
    bool is_inited_ = false;

    // time:
    double time_;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MODELS_PRE_INTEGRATOR_PRE_INTEGRATOR_HPP_