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
    // Cholesky 分解 : http://eigen.tuxfamily.org/dox/classEigen_1_1LLT.html
    Eigen::LLT<Eigen::Matrix<double,15,15>> LowerI(I_);
    // sqrt_info 为上三角阵
    Eigen::Matrix<double,15,15> sqrt_info = LowerI.matrixL().transpose();

    //
    // TODO: compute residual:
    //
     Eigen::Map<Eigen::Matrix<double, 15, 1>>  residual(residuals);
     
     residual.block<3, 1>(INDEX_P, 0) = ori_i.inverse().matrix() * (pos_j - pos_i - (vel_i - 0.5 * g_ * T_) * T_) - alpha_ij ;
     residual.block<3, 1>(INDEX_R,0) = (Sophus::SO3d::exp(theta_ij).inverse()*ori_i.inverse()*ori_j).log(); 
     residual.block<3, 1>(INDEX_V,0)  = ori_i.inverse()* (vel_j - vel_i + g_ * T_) - beta_ij ;
     residual.block<3, 1>(INDEX_A,0)  = b_a_j - b_a_i ; 
     residual.block<3, 1>(INDEX_G,0)  = b_g_j - b_g_i;
    //
    // TODO: compute jacobians:   imu预积分的残差 对状态量的雅克比，第九章已推导
    //
    if ( jacobians ) {
      // compute shared intermediate results:
      const Eigen::Matrix3d R_i_inv = ori_i.inverse().matrix();
      const Eigen::Matrix3d J_r_inv = JacobianRInv(residual.block(INDEX_R, 0 ,3 , 1));    //  右雅克比
     
      if ( jacobians[0] ) {
        Eigen::Map<Eigen::Matrix<double, 15 , 15,  Eigen::RowMajor>>  jacobian_i (jacobians[0] );  
        jacobian_i.setZero();
 
         // a. residual, position:
        jacobian_i.block<3, 3>(INDEX_P, INDEX_P) = -R_i_inv;
        jacobian_i.block<3, 3>(INDEX_P, INDEX_R) = Sophus::SO3d::hat(
          ori_i.inverse() * (pos_j - pos_i - (vel_i - 0.50 * g_ * T_) * T_)
        );
        jacobian_i.block<3, 3>(INDEX_P, INDEX_V) = -T_*R_i_inv;
        jacobian_i.block<3, 3>(INDEX_P, INDEX_A) = -J_.block<3,3>(INDEX_P, INDEX_A);
        jacobian_i.block<3, 3>(INDEX_P, INDEX_G) = -J_.block<3,3>(INDEX_P, INDEX_G);

        // b. residual, orientation:
        jacobian_i.block<3, 3>(INDEX_R, INDEX_R) = -J_r_inv*(ori_j.inverse()*ori_i).matrix();
        jacobian_i.block<3, 3>(INDEX_R, INDEX_G) = -J_r_inv*(
          Sophus::SO3d::exp(residual.block<3, 1>(INDEX_R, 0))
        ).matrix().inverse()*J_.block<3,3>(INDEX_R, INDEX_G);

        // c. residual, velocity:
        jacobian_i.block<3, 3>(INDEX_V, INDEX_R) = Sophus::SO3d::hat(
          ori_i.inverse() * (vel_j - vel_i + g_ * T_)
        );
        jacobian_i.block<3, 3>(INDEX_V, INDEX_V) = -R_i_inv;
        jacobian_i.block<3, 3>(INDEX_V, INDEX_A) = -J_.block<3,3>(INDEX_V, INDEX_A);
        jacobian_i.block<3, 3>(INDEX_V, INDEX_G) = -J_.block<3,3>(INDEX_V, INDEX_G);

        // d. residual, bias accel:
        jacobian_i.block<3, 3>(INDEX_A, INDEX_A) = -Eigen::Matrix3d::Identity();

        // d. residual, bias accel:
        jacobian_i.block<3, 3>(INDEX_G, INDEX_G) = -Eigen::Matrix3d::Identity();

        jacobian_i = sqrt_info * jacobian_i;
      }

      if ( jacobians[1] ) {
        Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> jacobian_j(jacobians[1]);
        jacobian_j.setZero();

        // a. residual, position:
        jacobian_j.block<3, 3>(INDEX_P, INDEX_P) = R_i_inv;

        // b. residual, orientation:
        jacobian_j.block<3, 3>(INDEX_R, INDEX_R) = J_r_inv;

        // c. residual, velocity:
        jacobian_j.block<3, 3>(INDEX_V, INDEX_V) = R_i_inv;

        // d. residual, bias accel:
        jacobian_j.block<3, 3>(INDEX_A, INDEX_A) = Eigen::Matrix3d::Identity();

        // d. residual, bias accel:
        jacobian_j.block<3, 3>(INDEX_G, INDEX_G) = Eigen::Matrix3d::Identity();

        jacobian_j = sqrt_info * jacobian_j;
      }
    }

    //
    // TODO: correct residual by square root of information matrix:
    //
    residual = sqrt_info * residual;
		
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
