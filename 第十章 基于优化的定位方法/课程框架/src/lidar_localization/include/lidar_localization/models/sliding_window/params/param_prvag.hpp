/*
 * @Description: ceres parameter block for LIO extended pose
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */

#ifndef LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_PARAM_PRVAG_HPP_
#define LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_PARAM_PRVAG_HPP_

#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

namespace sliding_window {

class ParamPRVAG : public ceres::LocalParameterization {
public:
    static const int INDEX_P = 0;
	static const int INDEX_R = 3;
	static const int INDEX_V = 6;
	static const int INDEX_A = 9;
	static const int INDEX_G = 12;
    
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const {
        Eigen::Map<const Eigen::Vector3d> pos(x + INDEX_P);
        Eigen::Map<const Eigen::Vector3d> ori(x + INDEX_R);
        Eigen::Map<const Eigen::Vector3d> vel(x + INDEX_V);
        Eigen::Map<const Eigen::Vector3d> b_a(x + INDEX_A);
        Eigen::Map<const Eigen::Vector3d> b_g(x + INDEX_G);

        Eigen::Map<const Eigen::Vector3d> d_pos(delta + INDEX_P);
        Eigen::Map<const Eigen::Vector3d> d_ori(delta + INDEX_R);
        Eigen::Map<const Eigen::Vector3d> d_vel(delta + INDEX_V);
        Eigen::Map<const Eigen::Vector3d> d_b_a(delta + INDEX_A);
        Eigen::Map<const Eigen::Vector3d> d_b_g(delta + INDEX_G);

        Eigen::Map<Eigen::Vector3d> pos_plus(x_plus_delta + INDEX_P);
        Eigen::Map<Eigen::Vector3d> ori_plus(x_plus_delta + INDEX_R);
        Eigen::Map<Eigen::Vector3d> vel_plus(x_plus_delta + INDEX_V);
        Eigen::Map<Eigen::Vector3d> b_a_plus(x_plus_delta + INDEX_A);
        Eigen::Map<Eigen::Vector3d> b_g_plus(x_plus_delta + INDEX_G);

        pos_plus = pos + d_pos;
        //
        // TODO: evaluate performance penalty of applying exp-exp-log transform for each update
        //
        ori_plus = (Sophus::SO3d::exp(ori) * Sophus::SO3d::exp(d_ori)).log();
        vel_plus = vel + d_vel;
        b_a_plus = b_a + d_b_a;
        b_g_plus = b_g + b_g_plus;

        return true;
    }

    virtual bool ComputeJacobian(const double *x, double *jacobian) const {
        Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> J(jacobian);
        J.setIdentity();

        return true;
    }

    virtual int GlobalSize() const { return 15; };

    virtual int LocalSize() const { return 15; };
};

} // namespace sliding_window

#endif //LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_PARAM_PRVAG_HPP_