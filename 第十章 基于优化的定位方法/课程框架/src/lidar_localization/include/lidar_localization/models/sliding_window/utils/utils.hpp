/*
 * @Description: utils for ceres residual block analytic Jacobians
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
#ifndef LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_UTILS_HPP_
#define LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_UTILS_HPP_

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

namespace sliding_window {

Eigen::Matrix3d JacobianRInv(const Eigen::Vector3d &w) {
    Eigen::Matrix3d J_r_inv = Eigen::Matrix3d::Identity();

    double theta = w.norm();

    if ( theta > 1e-5 ) {
        Eigen::Vector3d k = w.normalized();
        Eigen::Matrix3d K = Sophus::SO3d::hat(k);
        
        J_r_inv = J_r_inv 
                  + 0.5 * K
                  + (1.0 - (1.0 + std::cos(theta)) * theta / (2.0 * std::sin(theta))) * K * K;
    }

    return J_r_inv;
}

} // namespace sliding_window

#endif // LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_UTILS_HPP_