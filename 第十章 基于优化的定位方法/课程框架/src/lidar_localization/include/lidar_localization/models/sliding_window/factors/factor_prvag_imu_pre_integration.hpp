/*
 * @Description: ceres residual block for LIO IMU pre-integration measurement
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
#ifndef LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_IMU_PRE_INTEGRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_IMU_PRE_INTEGRATION_HPP_

#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

#include "glog/logging.h"

namespace sliding_window {

class FactorPRVAGIMUPreIntegration : public ceres::SizedCostFunction<15, 15, 15> {
public:
	static const int INDEX_P = 0;
	static const int INDEX_R = 3;
	static const int INDEX_V = 6;
	static const int INDEX_A = 9;
	static const int INDEX_G = 12;

  FactorPRVAGIMUPreIntegration(void) {};

	void SetT(const double &T) {
		T_ = T;
	}

	void SetGravitiy(const Eigen::Vector3d &g) {
		g_ = g;
	}

  void SetMeasurement(const Eigen::VectorXd &m) {
		m_ = m;
	}

  void SetInformation(const Eigen::MatrixXd &I) {
    I_ = I;
  }

	void SetJacobian(const Eigen::MatrixXd &J) {
		J_ = J;
	}

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    //
    // parse parameters:
    //
    // a. pose i
    Eigen::Map<const Eigen::Vector3d>     pos_i(&parameters[0][INDEX_P]);
    Eigen::Map<const Eigen::Vector3d> log_ori_i(&parameters[0][INDEX_R]);
    const Sophus::SO3d                    ori_i = Sophus::SO3d::exp(log_ori_i);
		Eigen::Map<const Eigen::Vector3d>     vel_i(&parameters[0][INDEX_V]);
		Eigen::Map<const Eigen::Vector3d>     b_a_i(&parameters[0][INDEX_A]);
		Eigen::Map<const Eigen::Vector3d>     b_g_i(&parameters[0][INDEX_G]);

    // b. pose j
    Eigen::Map<const Eigen::Vector3d>     pos_j(&parameters[1][INDEX_P]);
    Eigen::Map<const Eigen::Vector3d> log_ori_j(&parameters[1][INDEX_R]);
    const Sophus::SO3d                    ori_j = Sophus::SO3d::exp(log_ori_j);
		Eigen::Map<const Eigen::Vector3d>     vel_j(&parameters[1][INDEX_V]);
		Eigen::Map<const Eigen::Vector3d>     b_a_j(&parameters[1][INDEX_A]);
		Eigen::Map<const Eigen::Vector3d>     b_g_j(&parameters[1][INDEX_G]);

    //
    // parse measurement:
    // 
		const Eigen::Vector3d &alpha_ij = m_.block<3, 1>(INDEX_P, 0);
		const Eigen::Vector3d &theta_ij = m_.block<3, 1>(INDEX_R, 0);
		const Eigen::Vector3d  &beta_ij = m_.block<3, 1>(INDEX_V, 0);

    //
    // TODO: get square root of information matrix:
    //

    //
    // TODO: compute residual:
    //

    //
    // TODO: compute jacobians:
    //
    if ( jacobians ) {
      // compute shared intermediate results:

      if ( jacobians[0] ) {
        // a. residual, position:

        // b. residual, orientation:

        // c. residual, velocity:

        // d. residual, bias accel:

        // d. residual, bias accel:
      }

      if ( jacobians[1] ) {
        // a. residual, position:

        // b. residual, orientation:

        // c. residual, velocity:

        // d. residual, bias accel:

        // d. residual, bias accel:
      }
    }

    //
    // TODO: correct residual by square root of information matrix:
    //
		
    return true;
  }

private:
  static Eigen::Matrix3d JacobianRInv(const Eigen::Vector3d &w) {
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
  
	double T_ = 0.0;

	Eigen::Vector3d g_ = Eigen::Vector3d::Zero();

  Eigen::VectorXd m_;
  Eigen::MatrixXd I_;

	Eigen::MatrixXd J_;
};

} // namespace sliding_window

#endif // LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_IMU_PRE_INTEGRATION_HPP_
