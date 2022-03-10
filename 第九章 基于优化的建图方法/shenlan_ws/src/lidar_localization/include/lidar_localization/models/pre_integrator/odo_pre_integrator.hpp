/*
 * @Description: Odometer pre-integrator for LIO mapping, interface
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */

#ifndef LIDAR_LOCALIZATION_MODELS_PRE_INTEGRATOR_ODO_PRE_INTEGRATOR_HPP_
#define LIDAR_LOCALIZATION_MODELS_PRE_INTEGRATOR_ODO_PRE_INTEGRATOR_HPP_

#include "lidar_localization/models/pre_integrator/pre_integrator.hpp"

#include "lidar_localization/sensor_data/velocity_data.hpp"

#include "lidar_localization/models/graph_optimizer/g2o/edge/edge_prvag_odo_pre_integration.hpp"

#include <sophus/so3.hpp>

namespace lidar_localization {

class OdoPreIntegrator : public PreIntegrator {
public:
    static const int DIM_STATE = 6;

    typedef Eigen::Matrix<double, DIM_STATE, DIM_STATE> MatrixP;

    struct OdoPreIntegration {
        // a. measurement:
        // a.1. relative translation:
        Eigen::Vector3d alpha_ij_;
        // a.2. relative orientation:
        Sophus::SO3d theta_ij_;

        // b. information:
        MatrixP P_;

        g2o::EdgePRVAGOdoPreIntegration::Measurement GetMeasurement(void) const {
            g2o::EdgePRVAGOdoPreIntegration::Measurement measurement = g2o::EdgePRVAGOdoPreIntegration::Measurement::Zero();

            measurement.block<3, 1>(g2o::EdgePRVAGOdoPreIntegration::EdgePRVAGOdoPreIntegration::INDEX_P, 0) = alpha_ij_;
            measurement.block<3, 1>(g2o::EdgePRVAGOdoPreIntegration::EdgePRVAGOdoPreIntegration::INDEX_R, 0) = theta_ij_.log();

            return measurement;
        }

        Eigen::MatrixXd GetInformation(void) const {
            return P_.inverse();
        }
    };

    OdoPreIntegrator(const YAML::Node& node);

    /**
     * @brief  init odo pre-integrator
     * @param  init_velocity_data, init odometer measurement
     * @return true if success false otherwise
     */
    bool Init(const VelocityData &init_velocity_data);

    /**
     * @brief  update odo pre-integrator
     * @param  velocity_data, current odometer measurement
     * @return true if success false otherwise
     */
    bool Update(const VelocityData &velocity_data);

    /**
     * @brief  reset odo pre-integrator using new init odo measurement
     * @param  init_velocity_data, new init odo measurements
     * @param  output pre-integration result for constraint building as OdoPreIntegration
     * @return true if success false otherwise
     */
    bool Reset(const VelocityData &init_velocity_data, OdoPreIntegration &odo_pre_integration);

private:
    static const int DIM_NOISE = 12;

    static const int INDEX_ALPHA = 0;
    static const int INDEX_THETA = 3;

    static const int INDEX_M_V_PREV = 0;
    static const int INDEX_M_W_PREV = 3;
    static const int INDEX_M_V_CURR = 6;
    static const int INDEX_M_W_CURR = 9;

    typedef Eigen::Matrix<double, DIM_STATE, DIM_STATE> MatrixF;
    typedef Eigen::Matrix<double, DIM_STATE, DIM_NOISE> MatrixB;
    typedef Eigen::Matrix<double, DIM_NOISE, DIM_NOISE> MatrixQ;

    // data buff:
    std::deque<VelocityData> odo_data_buff_;

    // hyper-params:
    // a. prior state covariance, process & measurement noise:
    struct {
        struct {
            double V;
            double W;
        } MEASUREMENT;
    } COV;

    // pre-integration state:
    struct {
        // a. relative translation:
        Eigen::Vector3d alpha_ij_;
        // b. relative orientation:
        Sophus::SO3d theta_ij_;
    } state;

    // state covariance:
    MatrixP P_ = MatrixP::Zero();

    // process noise:
    MatrixQ Q_ = MatrixQ::Zero();

    // process equation:
    MatrixF F_ = MatrixF::Zero();
    MatrixB B_ = MatrixB::Zero();

    /**
     * @brief  reset pre-integrator state using odo measurement
     * @param  void
     * @return void
     */
    void ResetState(const VelocityData &init_velocity_data);

    /**
     * @brief  update pre-integrator state: mean, covariance
     * @param  void
     * @return void
     */
    void UpdateState(void);
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MODELS_PRE_INTEGRATOR_ODO_PRE_INTEGRATOR_HPP_