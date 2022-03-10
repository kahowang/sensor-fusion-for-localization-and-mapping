/*
 * @Description: IMU pre-integrator for LIO mapping, interface
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */

#ifndef LIDAR_LOCALIZATION_MODELS_PRE_INTEGRATOR_IMU_PRE_INTEGRATOR_HPP_
#define LIDAR_LOCALIZATION_MODELS_PRE_INTEGRATOR_IMU_PRE_INTEGRATOR_HPP_

#include "lidar_localization/models/pre_integrator/pre_integrator.hpp"

#include "lidar_localization/sensor_data/imu_data.hpp"

#include "lidar_localization/models/graph_optimizer/g2o/edge/edge_prvag_imu_pre_integration.hpp"

#include <sophus/so3.hpp>

namespace lidar_localization {

class IMUPreIntegrator : public PreIntegrator {
public:
    static const int DIM_STATE = 15;

    typedef Eigen::Matrix<double, DIM_STATE, DIM_STATE> MatrixP;
    typedef Eigen::Matrix<double, DIM_STATE, DIM_STATE> MatrixJ;

    struct IMUPreIntegration {
        // time delta:
        double T_;
        
        // gravity constant:
        Eigen::Vector3d g_;

        // a. measurement:
        // a.1. relative translation:
        Eigen::Vector3d alpha_ij_;
        // a.2. relative orientation:
        Sophus::SO3d theta_ij_;
        // a.3. relative velocity:
        Eigen::Vector3d beta_ij_;
        // a.4. accel bias:
        Eigen::Vector3d b_a_i_;
        // a.5. gyro bias:
        Eigen::Vector3d b_g_i_;

        // b. information:
        MatrixP P_;
        // c. Jacobian for update caused by bias:
        MatrixJ J_;

        double GetT(void) const { return T_; }
        
        Eigen::Vector3d GetGravity(void) const { return g_; }

        Vector15d GetMeasurement(void) const {
            Vector15d measurement = Vector15d::Zero();

            measurement.block<3, 1>(g2o::EdgePRVAGIMUPreIntegration::INDEX_P, 0) = alpha_ij_;
            measurement.block<3, 1>(g2o::EdgePRVAGIMUPreIntegration::INDEX_R, 0) = theta_ij_.log();
            measurement.block<3, 1>(g2o::EdgePRVAGIMUPreIntegration::INDEX_V, 0) = beta_ij_;

            return measurement;
        }

        Eigen::MatrixXd GetInformation(void) const {
            return P_.inverse();
        }

        Eigen::MatrixXd GetJacobian(void) const {
            return J_;
        }
    };

    IMUPreIntegrator(const YAML::Node& node);

    /**
     * @brief  init IMU pre-integrator
     * @param  init_imu_data, init IMU measurements
     * @return true if success false otherwise
     */
    bool Init(const IMUData &init_imu_data);

    /**
     * @brief  update IMU pre-integrator
     * @param  imu_data, current IMU measurements
     * @return true if success false otherwise
     */
    bool Update(const IMUData &imu_data);

    /**
     * @brief  reset IMU pre-integrator using new init IMU measurement
     * @param  init_imu_data, new init IMU measurements
     * @param  output pre-integration result for constraint building as IMUPreIntegration
     * @return true if success false otherwise
     */
    bool Reset(const IMUData &init_imu_data, IMUPreIntegration &imu_pre_integration);

private:
    static const int DIM_NOISE = 18;

    static const int INDEX_ALPHA = 0;
    static const int INDEX_THETA = 3;
    static const int INDEX_BETA = 6;
    static const int INDEX_B_A = 9;
    static const int INDEX_B_G = 12;

    static const int INDEX_M_ACC_PREV = 0;
    static const int INDEX_M_GYR_PREV = 3;
    static const int INDEX_M_ACC_CURR = 6;
    static const int INDEX_M_GYR_CURR = 9;
    static const int INDEX_R_ACC_PREV = 12;
    static const int INDEX_R_GYR_PREV = 15;

    typedef Eigen::Matrix<double, DIM_STATE, DIM_STATE> MatrixF;
    typedef Eigen::Matrix<double, DIM_STATE, DIM_NOISE> MatrixB;
    typedef Eigen::Matrix<double, DIM_NOISE, DIM_NOISE> MatrixQ;

    // data buff:
    std::deque<IMUData> imu_data_buff_;

    // hyper-params:
    // a. earth constants:
    struct {
        double GRAVITY_MAGNITUDE;
    } EARTH;
    // b. prior state covariance, process & measurement noise:
    struct {
        struct {
            double ACCEL;
            double GYRO;
        } RANDOM_WALK;
        struct {
            double ACCEL;
            double GYRO;
        } MEASUREMENT;
    } COV;

    // pre-integration state:
    struct {
        // gravity constant:
        Eigen::Vector3d g_;

        // a. relative translation:
        Eigen::Vector3d alpha_ij_;
        // b. relative orientation:
        Sophus::SO3d theta_ij_;
        // c. relative velocity:
        Eigen::Vector3d beta_ij_;
        // d. accel bias:
        Eigen::Vector3d b_a_i_;
        // e. gyro bias:
        Eigen::Vector3d b_g_i_;
    } state;

    // state covariance:
    MatrixP P_ = MatrixP::Zero();

    // Jacobian:
    MatrixJ J_ = MatrixJ::Identity();

    // process noise:
    MatrixQ Q_ = MatrixQ::Zero();

    // process equation:
    MatrixF F_ = MatrixF::Zero();
    MatrixB B_ = MatrixB::Zero();

    /**
     * @brief  reset pre-integrator state using IMU measurements
     * @param  void
     * @return void
     */
    void ResetState(const IMUData &init_imu_data);

    /**
     * @brief  update pre-integrator state: mean, covariance and Jacobian
     * @param  void
     * @return void
     */
    void UpdateState(void);
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MODELS_PRE_INTEGRATOR_IMU_PRE_INTEGRATOR_HPP_