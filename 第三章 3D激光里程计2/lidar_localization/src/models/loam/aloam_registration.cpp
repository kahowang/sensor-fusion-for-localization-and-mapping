/*
 * @Description: LOAM scan registration, implementation
 * @Author: Ge Yao
 * @Date: 2021-05-04 14:53:21
 */

#include <chrono>

#include "glog/logging.h"

#include "lidar_localization/models/loam/aloam_factor.hpp"

#include "lidar_localization/models/loam/aloam_analytic_factor.hpp"

#include "lidar_localization/models/loam/aloam_registration.hpp"


//#define  autograde                  //  使用自动求导  or  解析求导
//#define  maunual_block_loder                    //  使用ceres 自带EigenQuaternionParameterization  参数块   or  自定义 PoseSO3Parameterization 旋转参数块

namespace lidar_localization {

CeresALOAMRegistration::CeresALOAMRegistration(const Eigen::Quaternionf &dq, const Eigen::Vector3f &dt) {   
    //
    // config optimizer:
    // 
    // 1. parameterization:
    #ifdef  maunual_block_loder
        config_.q_parameterization_ptr =  new  PoseSO3Parameterization() ;                    //  自定义旋转参数块
    #else 
        config_.q_parameterization_ptr = new ceres::EigenQuaternionParameterization();          //   SE3 转换矩阵/位姿参数化，基类ceres派生 ， 构造顺序( x y z  w)
    #endif
    // 2. loss function:
    // TODO: move param to config
    config_.loss_function_ptr =  new ceres::HuberLoss(0.10);

    // 3. solver:
    config_.options.linear_solver_type = ceres::DENSE_QR;
    // config_.options.use_explicit_schur_complement = true;
    // config_.options.trust_region_strategy_type = ceres::DOGLEG;
    // config_.options.use_nonmonotonic_steps = true;
    config_.options.num_threads = 2;
    config_.options.max_num_iterations = 50;
    config_.options.minimizer_progress_to_stdout = false;
    config_.options.max_solver_time_in_seconds = 0.10;

    //
    // config target variables:
    //
    param_.q[0] = dq.x(); param_.q[1] = dq.y(); param_.q[2] = dq.z(); param_.q[3] = dq.w();
    param_.t[0] = dt.x(); param_.t[1] = dt.y(); param_.t[2] = dt.z();
    problem_.AddParameterBlock(param_.q, 4, config_.q_parameterization_ptr);            //  加载自定义旋转参数块
    problem_.AddParameterBlock(param_.t, 3);
}

CeresALOAMRegistration::~CeresALOAMRegistration() {
}

/**
  * @brief  add residual block for edge constraint from lidar frontend   
  * @param  source, source point  
  * @param  target_x, target point x
  * @param  target_y, target point y
  * @param  ratio, interpolation ratio 
  * @return void
  */
bool CeresALOAMRegistration::AddEdgeFactor(
    const Eigen::Vector3d &source,
    const Eigen::Vector3d &target_x, const Eigen::Vector3d &target_y,
    const double &ratio
) {

    /*自动求导*/
  #ifdef  autograde
    ceres::CostFunction *factor_edge = LidarEdgeFactor::Create(                  //   创建误差项
        source, 
        target_x, target_y, 
        ratio
    );

    problem_.AddResidualBlock(
        factor_edge,                                    //   约束边   cost_function
        config_.loss_function_ptr,        //   鲁棒核函数  lost_function
        param_.q, param_.t                      //  关联参数
    );

    #else
   /*解析求导*/
   ceres::CostFunction *factor_analytic_edge =   new EdgeAnalyticCostFunction(
        source, 
        target_x, target_y, 
        ratio
   );

    problem_.AddResidualBlock(
        factor_analytic_edge,                                    //   约束边   cost_function
        config_.loss_function_ptr,        //   鲁棒核函数  lost_function
        param_.q, param_.t                      //  关联参数
    );

#endif
    return true;
}

/**
  * @brief  add residual block for plane constraint from lidar frontend
  * @param  source, source point
  * @param  target_x, target point x
  * @param  target_y, target point y
  * @param  target_z, target point z
  * @param  ratio, interpolation ratio
  * @return void
  */
bool CeresALOAMRegistration::AddPlaneFactor(
    const Eigen::Vector3d &source,
    const Eigen::Vector3d &target_x, const Eigen::Vector3d &target_y, const Eigen::Vector3d &target_z,
    const double &ratio
) {

    /*自动求导*/
    #ifdef  autograde
    ceres::CostFunction *factor_plane = LidarPlaneFactor::Create(
        source, 
        target_x, target_y, target_z, 
        ratio
    );

    problem_.AddResidualBlock(
        factor_plane,
        config_.loss_function_ptr, 
        param_.q, param_.t
    );

#else
   /*解析求导*/
    ceres::CostFunction *factor_analytic_plane =new PlaneAnalyticCostFunction(
        source, 
        target_x, target_y, target_z, 
        ratio
    );

    problem_.AddResidualBlock(
        factor_analytic_plane,
        config_.loss_function_ptr, 
        param_.q, param_.t
    );

#endif

    return true;
}

bool CeresALOAMRegistration::Optimize() {
    // solve:
    ceres::Solver::Summary summary;
    
    // time it:
    auto start = std::chrono::steady_clock::now();

    ceres::Solve(config_.options, &problem_, &summary);              //  优化
    
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = end - start;

    // prompt:
    LOG(INFO) << "Time Used: " << time_used.count() << " seconds." << std::endl
                << "Cost Reduced: " << summary.initial_cost - summary.final_cost << std::endl
                << summary.BriefReport() << std::endl
                << std::endl;
    
    return true;
}

/**
  * @brief  get optimized relative pose
  * @return true if success false otherwise
  */
bool CeresALOAMRegistration::GetOptimizedRelativePose(Eigen::Quaternionf &dq, Eigen::Vector3f &dt) {
    Eigen::Quaternionf q(param_.q[0], param_.q[1], param_.q[2], param_.q[3]);
    Eigen::Vector3f t(param_.t[0], param_.t[1], param_.t[2]);

    dq = q;
    dt = t;

    return true;
}

} // namespace graph_ptr_optimization
