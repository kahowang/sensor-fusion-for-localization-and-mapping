/*
 * @Description: Odometer pre-integrator for LIO mapping, implementation
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */

#include "lidar_localization/models/pre_integrator/odo_pre_integrator.hpp"

#include "lidar_localization/global_defination/global_defination.h"

#include "glog/logging.h"

namespace lidar_localization {

OdoPreIntegrator::OdoPreIntegrator(const YAML::Node& node) {
    //
    // parse config:
    // 
    // a. process noise:
    COV.MEASUREMENT.V = node["covariance"]["measurement"]["v"].as<double>();
    COV.MEASUREMENT.W = node["covariance"]["measurement"]["w"].as<double>();

    // prompt:
    LOG(INFO) << std::endl 
              << "Odo Pre-Integration params:" << std::endl
              << "\tprocess noise:" << std::endl
              << "\t\tmeasurement:" << std::endl
              << "\t\t\tv.: " << COV.MEASUREMENT.V << std::endl
              << "\t\t\tw.: " << COV.MEASUREMENT.W << std::endl
              << std::endl;

    // a. process noise:
    Q_.block<3, 3>(INDEX_M_V_PREV, INDEX_M_V_PREV) = Q_.block<3, 3>(INDEX_M_V_CURR, INDEX_M_V_CURR) = COV.MEASUREMENT.V * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(INDEX_M_W_PREV, INDEX_M_W_PREV) = Q_.block<3, 3>(INDEX_M_W_CURR, INDEX_M_W_CURR) = COV.MEASUREMENT.W * Eigen::Matrix3d::Identity();

    // b. process equation, noise input:
    B_.block<3, 3>(INDEX_THETA, INDEX_M_W_PREV) = B_.block<3, 3>(INDEX_THETA, INDEX_M_W_CURR) = 0.50 * Eigen::Matrix3d::Identity();
}

/**
 * @brief  init odo pre-integrator
 * @param  init_velocity_data, init odometer measurement
 * @return true if success false otherwise
 */
bool OdoPreIntegrator::Init(const VelocityData &init_velocity_data) {
    // reset pre-integrator state:
    ResetState(init_velocity_data);
    
    // mark as inited:
    is_inited_ = true;

    return true;
}

/**
 * @brief  update odo pre-integrator
 * @param  velocity_data, current odometer measurement
 * @return true if success false otherwise
 */
bool OdoPreIntegrator::Update(const VelocityData &velocity_data) {
    if ( odo_data_buff_.front().time < velocity_data.time ) {
        // set buffer:
        odo_data_buff_.push_back(velocity_data);

        // update state mean, covariance and Jacobian:
        UpdateState();

        // move forward:
        odo_data_buff_.pop_front();
    }

    return true;
}

/**
 * @brief  reset odo pre-integrator using new init odo measurement
 * @param  init_velocity_data, new init odo measurements
 * @param  output pre-integration result for constraint building as OdoPreIntegration
 * @return true if success false otherwise
 */
bool OdoPreIntegrator::Reset(
    const VelocityData &init_velocity_data, 
    OdoPreIntegration &odo_pre_integration
) {
    // one last update:
    Update(init_velocity_data);

    // set measurement:
    odo_pre_integration.alpha_ij_ = state.alpha_ij_;
    odo_pre_integration.theta_ij_ = state.theta_ij_;
    // set information:
    odo_pre_integration.P_ = P_;

    // reset:
    ResetState(init_velocity_data);

    return true;
}

/**
 * @brief  reset pre-integrator state using odo measurement
 * @param  void
 * @return void
 */
void OdoPreIntegrator::ResetState(const VelocityData &init_velocity_data) {
    // reset time:
    time_ = init_velocity_data.time;

    // a. reset relative translation:
    state.alpha_ij_ = Eigen::Vector3d::Zero();
    // b. reset relative orientation:
    state.theta_ij_ = Sophus::SO3d();

    // reset state covariance:
    P_ = MatrixP::Zero();

    // reset buffer:
    odo_data_buff_.clear();
    odo_data_buff_.push_back(init_velocity_data);
}

/**
 * @brief  update pre-integrator state: mean, covariance
 * @param  void
 * @return void
 */
void OdoPreIntegrator::UpdateState(void) {
    static double T = 0.0;

    static Eigen::Vector3d w_mid = Eigen::Vector3d::Zero();
    static Eigen::Vector3d v_mid = Eigen::Vector3d::Zero();

    static Sophus::SO3d prev_theta_ij = Sophus::SO3d();
    static Sophus::SO3d curr_theta_ij = Sophus::SO3d();
    static Sophus::SO3d d_theta_ij = Sophus::SO3d();

    static Eigen::Matrix3d dR_inv = Eigen::Matrix3d::Zero();
    static Eigen::Matrix3d prev_R = Eigen::Matrix3d::Zero();
    static Eigen::Matrix3d curr_R = Eigen::Matrix3d::Zero();
    static Eigen::Matrix3d prev_R_v_hat = Eigen::Matrix3d::Zero();
    static Eigen::Matrix3d curr_R_v_hat = Eigen::Matrix3d::Zero();

    //
    // parse measurements:
    //
    // get measurement handlers:
    const VelocityData &prev_odo_data = odo_data_buff_.at(0);
    const VelocityData &curr_odo_data = odo_data_buff_.at(1);

    // get time delta:
    T = curr_odo_data.time - prev_odo_data.time;

    // get measurements:
    const Eigen::Vector3d prev_w(
        prev_odo_data.angular_velocity.x,
        prev_odo_data.angular_velocity.y,
        prev_odo_data.angular_velocity.z
    );
    const Eigen::Vector3d curr_w(
        curr_odo_data.angular_velocity.x,
        curr_odo_data.angular_velocity.y,
        curr_odo_data.angular_velocity.z
    );

    const Eigen::Vector3d prev_v(
        prev_odo_data.linear_velocity.x,
        prev_odo_data.linear_velocity.y,
        prev_odo_data.linear_velocity.z
    );
    const Eigen::Vector3d curr_v(
        curr_odo_data.linear_velocity.x,
        curr_odo_data.linear_velocity.y,
        curr_odo_data.linear_velocity.z
    );

    //
    // a. update mean:
    //
    // 1. get w_mid:
    w_mid = 0.5 * ( prev_w + curr_w );
    // 2. update relative orientation, so3:
    prev_theta_ij = state.theta_ij_;
    d_theta_ij = Sophus::SO3d::exp(w_mid * T);
    state.theta_ij_ = state.theta_ij_ * d_theta_ij;
    curr_theta_ij = state.theta_ij_;
    // 3. get v_mid:
    v_mid = 0.5 * ( prev_theta_ij * prev_v + curr_theta_ij * curr_v );
    // 4. update relative velocity:
    state.alpha_ij_ += v_mid * T;

    //
    // b. update covariance:
    //
    // 1. intermediate results:
    dR_inv = d_theta_ij.inverse().matrix();
    prev_R = prev_theta_ij.matrix();
    curr_R = curr_theta_ij.matrix();
    prev_R_v_hat = prev_R * Sophus::SO3d::hat(prev_v);
    curr_R_v_hat = curr_R * Sophus::SO3d::hat(curr_v);

    //
    // 2. set up F:
    //
    // F12:
    F_.block<3, 3>(INDEX_ALPHA, INDEX_THETA) = -0.50 * (prev_R_v_hat + curr_R_v_hat * dR_inv);
    // F22:
    F_.block<3, 3>(INDEX_THETA, INDEX_THETA) = -Sophus::SO3d::hat(w_mid);

    //
    // 3. set up G:
    //
    // G11:
    B_.block<3, 3>(INDEX_ALPHA, INDEX_M_V_PREV) = +0.50 * prev_R;
    // G13:
    B_.block<3, 3>(INDEX_ALPHA, INDEX_M_V_CURR) = +0.50 * curr_R;
    // G12 & G14:
    B_.block<3, 3>(INDEX_ALPHA, INDEX_M_W_PREV) = B_.block<3, 3>(INDEX_ALPHA, INDEX_M_W_CURR) = -0.25 * T * curr_R_v_hat;

    // 4. update P_:
    MatrixF F = MatrixF::Identity() + T * F_;
    MatrixB B = T * B_;

    P_ = F*P_*F.transpose() + B*Q_*B.transpose();
}

} // namespace lidar_localization