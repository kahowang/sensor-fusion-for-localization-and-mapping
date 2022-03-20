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

    //
    // TODO: Update b:
    //
    // a. b_m:
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
    // b. H_mr:
    // c. H_rm:
    // d. H_rr:


    //
    // TODO: Update b:
    //
    // a. b_m:
    // a. b_r:
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
    // b. H_mr:
    // c. H_rm:
    // d. H_rr:


    //
    // Update b:
    //
    // a. b_m:
    // a. b_r:
  }

  void Marginalize(
    const double *raw_param_r_0
  ) {
    // TODO: implement marginalization logic
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

    //
    // TODO: compute jacobian:
    //
    if ( jacobians ) {
      if ( jacobians[0] ) {
        // implement computing:
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
