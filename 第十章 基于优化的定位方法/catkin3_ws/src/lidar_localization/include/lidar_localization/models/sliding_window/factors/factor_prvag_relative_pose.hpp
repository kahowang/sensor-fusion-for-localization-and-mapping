/*
 * @Description: ceres residual block for lidar frontend relative pose measurement
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
#ifndef LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_RELATIVE_POSE_HPP_
#define LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_RELATIVE_POSE_HPP_

#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

#include "glog/logging.h"

namespace sliding_window {

class FactorPRVAGRelativePose : public ceres::SizedCostFunction<6, 15, 15> {
public:
	static const int INDEX_P = 0;
	static const int INDEX_R = 3;

  FactorPRVAGRelativePose(void) {};

  void SetMeasurement(const Eigen::VectorXd &m) {
		m_ = m;
	}

  void SetInformation(const Eigen::MatrixXd &I) {
    I_ = I;
  }

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    //
    // parse parameters:
    //
    // a. pose i
    Eigen::Map<const Eigen::Vector3d>     pos_i(&parameters[0][INDEX_P]);
    Eigen::Map<const Eigen::Vector3d> log_ori_i(&parameters[0][INDEX_R]);
    const Sophus::SO3d                    ori_i = Sophus::SO3d::exp(log_ori_i);

    // b. pose j
    Eigen::Map<const Eigen::Vector3d>     pos_j(&parameters[1][INDEX_P]);
    Eigen::Map<const Eigen::Vector3d> log_ori_j(&parameters[1][INDEX_R]);
    const Sophus::SO3d                    ori_j = Sophus::SO3d::exp(log_ori_j);

    //
    // parse measurement:
    // 
		const Eigen::Vector3d     &pos_ij = m_.block<3, 1>(INDEX_P, 0);
		const Eigen::Vector3d &log_ori_ij = m_.block<3, 1>(INDEX_R, 0);
    const Sophus::SO3d         ori_ij = Sophus::SO3d::exp(log_ori_ij);

    //
    // TODO: get square root of information matrix:
    //
    Eigen::Matrix<double, 6, 6>  sqrt_info =  Eigen::LLT<Eigen::Matrix<double, 6 ,6>>(
      I_
    ).matrixL().transpose() ;

    //
    // TODO: compute residual:
    //
    Eigen::Map<Eigen::Matrix<double, 6 ,1>>  residual(residuals);     //  残差 r_P  r_R

    residual.block(INDEX_P,  0 , 3 , 1)   =  ori_i.inverse() * (pos_j - pos_i) - pos_ij ;
    residual.block(INDEX_R, 0  , 3 , 1)   =  (ori_i.inverse()*ori_j*ori_ij.inverse()).log( );  


    //
    // TODO: compute jacobians:     二元边
    //
    if ( jacobians ) {
      // compute shared intermediate results:
      const Eigen::Matrix3d R_i_inv = ori_i.inverse().matrix();
      const Eigen::Matrix3d J_r_inv = JacobianRInv(residual.block(INDEX_R, 0 ,3 , 1));    //  右雅克比
      const Eigen::Vector3d pos_ij = ori_i.inverse() * (pos_j - pos_i) ;

      if ( jacobians[0] ) {      //   残差rL0(rp  rq )  对 T0(p q)  M0(v ba bg) 的雅克比 
        // implement computing:
        Eigen::Map<Eigen::Matrix<double, 6 , 15,  Eigen::RowMajor>>  jacobian_i (jacobians[0] );  //  col : rp_i[3] rq_i[3]  row : p[3] q[3] v[3] ba[3] bg[3]  
        jacobian_i.setZero();

        jacobian_i.block<3, 3>(INDEX_P, INDEX_P) =  -R_i_inv;
        jacobian_i.block<3, 3>(INDEX_R,INDEX_R)  =  -J_r_inv*(ori_ij*ori_j.inverse()*ori_i).matrix();
        jacobian_i.block<3, 3>(INDEX_P,INDEX_R) =  Sophus::SO3d::hat(pos_ij).matrix();

        jacobian_i = sqrt_info * jacobian_i ;        //   注意 sqrt_i 为对角的协方差矩阵对角线为观测的方差，可理解为传感器的测量误差，用于调整权重用
      }

      if ( jacobians[1] ) {      //  残差rL0(rp  rq )  对 T0(p q)  M0(v ba bg) 的雅克比 
        // implement computing:
        Eigen::Map<Eigen::Matrix<double, 6 ,15, Eigen::RowMajor>> jacobian_j (jacobians[1]);
        jacobian_j.setZero();

        jacobian_j.block<3, 3>(INDEX_P, INDEX_P) = R_i_inv;
        jacobian_j.block<3, 3>(INDEX_R,INDEX_R)  = J_r_inv*ori_ij.matrix();

        jacobian_j = sqrt_info * jacobian_j ;
      }
    }

    //
    // TODO: correct residual by square root of information matrix:
    //
    residual = sqrt_info * residual;
    
    return true;
  }

private:
//  右乘BCH 有问题  fix bug
  // static Eigen::Matrix3d JacobianRInv(const Eigen::Vector3d &w) {
  //     Eigen::Matrix3d J_r_inv = Eigen::Matrix3d::Identity();

  //     double theta = w.norm();

  //     if ( theta > 1e-5 ) {
  //         Eigen::Vector3d k = w.normalized();
  //         Eigen::Matrix3d K = Sophus::SO3d::hat(k);
          
  //         J_r_inv = J_r_inv 
  //                   + 0.5 * K
  //                   + (1.0 - (1.0 + std::cos(theta)) * theta / (2.0 * std::sin(theta))) * K * K;
  //     }

  //     return J_r_inv;
  // }

    static Eigen::Matrix3d JacobianRInv(const Eigen::Vector3d &w) {
      Eigen::Matrix3d J_r_inv = Eigen::Matrix3d::Identity();

      double theta = w.norm();

      if ( theta > 1e-5 ) 
      {
          Eigen::Vector3d a = w.normalized();
          Eigen::Matrix3d a_hat = Sophus::SO3d::hat(a);
          double theta_half = 0.5 * theta ;
          double cot_theta = 1.0 / tan(theta_half);

          J_r_inv = theta_half * cot_theta * J_r_inv  +  (1.0 - 
          theta_half * cot_theta) * a * a.transpose() + 
          theta_half * a_hat ;
      }

      return J_r_inv;
  }

  Eigen::VectorXd m_;
  Eigen::MatrixXd I_;
};

} // namespace sliding_window

#endif // LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_RELATIVE_POSE_HPP_
