/*
 * @Description: ceres residual block for sliding window marginalization
 * @Author: Ge Yao
 * @Date: 2021-01-05 21:57:49
 */
#ifndef LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_MARGINALIZATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_MARGINALIZATION_HPP_

#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

#include "glog/logging.h"

namespace sliding_window {

class FactorPRVAGMarginalization : public ceres::SizedCostFunction<15, 15> {
public:
	static const int INDEX_M =  0;
  static const int INDEX_R = 15;

  FactorPRVAGMarginalization(void) {
    H_ = Eigen::MatrixXd::Zero(30, 30);
    b_ = Eigen::VectorXd::Zero(30);

    J_ = Eigen::MatrixXd::Zero(15, 15);
    e_ = Eigen::VectorXd::Zero(15);
  }

  void SetResMapMatchingPose(
    const ceres::CostFunction *residual,
    const std::vector<double *> &parameter_blocks
  ) {
    // init:
    ResidualBlockInfo res_map_matching_pose(residual, parameter_blocks);
    Eigen::VectorXd residuals;
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;

    // compute:
    Evaluate(res_map_matching_pose, residuals, jacobians);
    const Eigen::MatrixXd &J_m = jacobians.at(0);

    //
    // TODO: Update H:
    //
    // a. H_mm:
    H_.block<15,  15>(INDEX_M, INDEX_M) += J_m.transpose() * J_m ;
    //
    // TODO: Update b:
    //
    // a. b_m:
    b_.block<15, 1>(INDEX_M , 0) +=  J_m.transpose() * residuals ;  //  因子图叠加
  }

  void SetResRelativePose(
    const ceres::CostFunction *residual,
    const std::vector<double *> &parameter_blocks
  ) {
    // init:
    ResidualBlockInfo res_relative_pose(residual, parameter_blocks);
    Eigen::VectorXd residuals;
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;

    // compute:
    Evaluate(res_relative_pose, residuals, jacobians);
    const Eigen::MatrixXd &J_m = jacobians.at(0);
    const Eigen::MatrixXd &J_r = jacobians.at(1);

    //
    // TODO: Update H:
    //
    // a. H_mm:
    H_.block<15, 15>(INDEX_M, INDEX_M)  += J_m.transpose() * J_m;
    // b. H_mr:
    H_.block<15, 15>(INDEX_M, INDEX_R)   += J_m.transpose()* J_r;
    // c. H_rm:
    H_.block<15,  15>(INDEX_R, INDEX_M)  += J_r.transpose() * J_m; 
    // d. H_rr:
    H_.block<15,  15>(INDEX_R, INDEX_R)   += J_r.transpose() * J_r;


    //
    // TODO: Update b:
    //
    // a. b_m:
    b_.block<15,  1>(INDEX_M, 0) += J_m.transpose() * residuals ;
    // a. b_r:
    b_.block<15,   1>(INDEX_R,  0) += J_r.transpose() * residuals ;
  }

  void SetResIMUPreIntegration(
    const ceres::CostFunction *residual,
    const std::vector<double *> &parameter_blocks
  ) {
    // init:
    ResidualBlockInfo res_imu_pre_integration(residual, parameter_blocks);
    Eigen::VectorXd residuals;
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;

    // compute:
    Evaluate(res_imu_pre_integration, residuals, jacobians);
    const Eigen::MatrixXd &J_m = jacobians.at(0);
    const Eigen::MatrixXd &J_r = jacobians.at(1);

    //
    // TODO: Update H:
    //
    // a. H_mm:
    H_.block<15, 15>(INDEX_M, INDEX_M)  += J_m.transpose() * J_m;
    // b. H_mr:
    H_.block<15, 15>(INDEX_M, INDEX_R)   += J_m.transpose()* J_r;
    // c. H_rm:
    H_.block<15,  15>(INDEX_R, INDEX_M)  += J_r.transpose() * J_m; 
    // d. H_rr:
    H_.block<15,  15>(INDEX_R, INDEX_R)   += J_r.transpose() * J_r;


    //
    // TODO: Update b:
    //
    // a. b_m:
    b_.block<15,  1>(INDEX_M, 0) += J_m.transpose() * residuals ;
    // a. b_r:
    b_.block<15,   1>(INDEX_R,  0) += J_r.transpose() * residuals ;
  }

  void Marginalize(
    const double *raw_param_r_0
  ) {
    // TODO: implement marginalization logic
    //  save x_m_0 
    Eigen::Map<const  Eigen::Matrix<double, 15 , 1>>  x_0(raw_param_r_0);
    x_0_ = x_0 ;
    //   marginalize
    const Eigen::MatrixXd  &H_mm = H_.block<15,  15>(INDEX_M, INDEX_M);
    const Eigen::MatrixXd  &H_mr   = H_.block<15,   15>(INDEX_M,INDEX_R);
    const Eigen::MatrixXd  &H_rm   = H_.block<15,   15>(INDEX_R,INDEX_M);
    const Eigen::MatrixXd  &H_rr     = H_.block<15,    15>(INDEX_R,INDEX_R);

    const Eigen::VectorXd  &b_m = b_.block<15,  1>(INDEX_M, 0);
    const Eigen::VectorXd  &b_r   = b_.block<15,   1>(INDEX_R, 0);

    Eigen::MatrixXd H_mm_inv = H_mm.inverse();
    Eigen::MatrixXd H_marginalized = H_rr - H_rm * H_mm_inv * H_mr ;
    Eigen::MatrixXd b_marginalized = b_r - H_rm * H_mm_inv * b_m ;
    // 线性化残差 和 雅克比
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(H_marginalized);
    Eigen::VectorXd S = Eigen::VectorXd(
      (saes.eigenvalues().array() > 1.0e-5).select(saes.eigenvalues().array(), 0)
    );
    Eigen::VectorXd S_inv = Eigen::VectorXd(
      (saes.eigenvalues().array() > 1.0e-5).select(saes.eigenvalues().array().inverse(), 0)
    );

    Eigen::VectorXd S_sqrt = S.cwiseSqrt();
    Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

    // finally:
    J_ = S_sqrt.asDiagonal() * saes.eigenvectors().transpose();         //  b0
    e_ = S_inv_sqrt.asDiagonal() * saes.eigenvectors().transpose() * b_marginalized;      //   eo
  }

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {	
    //
    // parse parameters:
    //
    Eigen::Map<const Eigen::Matrix<double, 15, 1>> x(parameters[0]);
    Eigen::VectorXd dx = x - x_0_;

    //
    // TODO: compute residual:
    //
    Eigen::Map<Eigen::Matrix<double, 15, 1>>  residual(residuals);
    residual = e_ + J_ * dx ;       //  e_prior

    //
    // TODO: compute jacobian:
    //
    if ( jacobians ) {
      if ( jacobians[0] ) {
        // implement computing:
        Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>>  jacobian_marginalization(jacobians[0]);
        jacobian_marginalization.setZero();

        jacobian_marginalization = J_ ;        
      }
    }

    return true;
  }

private:
  Eigen::MatrixXd H_;
  Eigen::VectorXd b_;

  Eigen::MatrixXd J_;
  Eigen::VectorXd e_;

  Eigen::VectorXd x_0_;

  struct ResidualBlockInfo {
    const ceres::CostFunction *residual = nullptr;
    std::vector<double *> parameter_blocks;

    ResidualBlockInfo(void) {}

    ResidualBlockInfo(
      const ceres::CostFunction *_residual,
      const std::vector<double *> &_parameter_blocks
    ) : residual(_residual), parameter_blocks(_parameter_blocks) {}
  };

  static void Evaluate(
    ResidualBlockInfo &residual_info,
    Eigen::VectorXd &residuals,
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> &jacobians
  ) {
    // init residual output:
    const int D = static_cast<int>(residual_info.residual->num_residuals());
    residuals.resize(D);

    // init jacobians output:
    std::vector<int> block_sizes = residual_info.residual->parameter_block_sizes();
    const int N = static_cast<int>(block_sizes.size());

    double **raw_jacobians = new double *[N];
    jacobians.resize(N);

    // create raw pointer adaptor:
    for (int i = 0; i < N; i++) {
      jacobians[i].resize(D, block_sizes[i]);
      raw_jacobians[i] = jacobians[i].data();
    }

    residual_info.residual->Evaluate(
      residual_info.parameter_blocks.data(), 
      residuals.data(), raw_jacobians
    );
  }
};

} // namespace sliding_window

#endif // LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_MARGINALIZATION_HPP_
