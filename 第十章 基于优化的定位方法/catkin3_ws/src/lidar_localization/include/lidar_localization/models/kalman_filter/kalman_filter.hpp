/*
 * @Description: Kalman Filter interface.
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */

#ifndef LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_KALMAN_FILTER_HPP_
#define LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_KALMAN_FILTER_HPP_

#include <yaml-cpp/yaml.h>

#include <deque>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "lidar_localization/sensor_data/imu_data.hpp"

namespace lidar_localization {

class KalmanFilter {
public:
    /**
     * @class MeasurementType
     * @brief enum for observation type
     */
    enum MeasurementType {
        POSE = 0,
        POSE_VEL,
        POSI,
        POSI_VEL,
        POSI_MAG,
        POSI_VEL_MAG,
        NUM_TYPES
    };

    /**
     * @class Measurement
     * @brief Kalman filter measurement data
     */
    struct Measurement {
        // timestamp:
        double time;
        // a. pose observation, lidar/visual frontend:
        Eigen::Matrix4d T_nb;
        // b. body frame velocity observation, odometer:
        Eigen::Vector3d v_b;
        // c. body frame angular velocity, needed by motion constraint:
        Eigen::Vector3d w_b;
        // d. magnetometer:
        Eigen::Vector3d B_b;
    };

    /**
     * @class Cov
     * @brief Kalman filter process covariance data
     */
    struct Cov {
        struct {
            double x;
            double y;
            double z;
        } pos;
        struct {
            double x;
            double y;
            double z;
        } vel;
        // here quaternion is used for orientation representation:
        struct {
            double w;
            double x;
            double y;
            double z;
        } ori;
        struct {
            double x;
            double y;
            double z;
        } gyro_bias;
        struct {
            double x;
            double y;
            double z;
        } accel_bias;
    };

    /**
     * @brief  init filter
     * @param  imu_data, input IMU measurements
     * @return true if success false otherwise
     */
    virtual void Init(
        const Eigen::Vector3d &vel, 
        const IMUData &imu_data
    ) = 0;

    /**
     * @brief  update state & covariance estimation, Kalman prediction
     * @param  imu_data, input IMU measurements
     * @return true if success false otherwise
     */
    virtual bool Update(
        const IMUData &imu_data
    ) = 0;

    /**
     * @brief  correct state & covariance estimation, Kalman correction
     * @param  measurement_type, input measurement type
     * @param  measurement, input measurement
     * @return void                                   
     */
    virtual bool Correct(
        const IMUData &imu_data, 
        const MeasurementType &measurement_type, const Measurement &measurement
    ) = 0;

    /**
     * @brief  get filter time
     * @return filter time as double    
     */
    double GetTime(void) const { return time_; }
    
    /**
     * @brief  get odometry estimation
     * @param  pose, output pose
     * @param  vel, output vel
     * @return void
     */
    virtual void GetOdometry(Eigen::Matrix4f &pose, Eigen::Vector3f &vel) = 0;

    /**
     * @brief  get covariance estimation
     * @param  cov, output covariance 
     * @return void
     */
    virtual void GetCovariance(Cov &cov) = 0;
    
    /**
     * @brief  update observability analysis
     * @param  time, measurement time
     * @param  measurement_type, measurement type
     * @return void
     */
    virtual void UpdateObservabilityAnalysis(
        const double &time,
        const MeasurementType &measurement_type
    ) = 0;

    /**
     * @brief  save observability analysis to persistent storage
     * @param  measurement_type, measurement type
     * @return void
     */
    virtual bool SaveObservabilityAnalysis(const MeasurementType &measurement_type) = 0;

protected:
    KalmanFilter(){}

    static void AnalyzeQ(
        const int DIM_STATE,
        const double &time, const Eigen::MatrixXd &Q,
        std::vector<std::vector<double>> &data
    );

    static void WriteAsCSV(
        const int DIM_STATE,
        const std::vector<std::vector<double>> &data,
        const std::string filename
    );

    // time:
    double time_;

    // data buff:
    std::deque<IMUData> imu_data_buff_;

    // earth constants:
    Eigen::Vector3d g_;
    Eigen::Vector3d w_;
    Eigen::Vector3d b_;

    // observability analysis:
    struct {
        std::vector<double> time_;
        std::vector<Eigen::MatrixXd> Q_;
        
        std::vector<std::vector<double>> pose_;
        std::vector<std::vector<double>> pose_vel_;
        std::vector<std::vector<double>> posi_;
        std::vector<std::vector<double>> posi_vel_;
        std::vector<std::vector<double>> posi_mag_;
        std::vector<std::vector<double>> posi_vel_mag_;
    } observability;

    // hyper-params:
    // a. earth constants:
    struct {
        double GRAVITY_MAGNITUDE;
        double ROTATION_SPEED;
        double LATITUDE;
        double LONGITUDE;
        struct {
            double B_E;
            double B_N;
            double B_U;
        } MAG;
    } EARTH;
    // b. prior state covariance, process & measurement noise:
    struct {
        struct {
            double POSI;
            double VEL;
            double ORI;
            double EPSILON;
            double DELTA;
        } PRIOR;
        struct {
            double GYRO;
            double ACCEL;
        } PROCESS;
        struct {
            struct {
                double POSI;
                double ORI;
            } POSE;
            double POSI;
            double VEL;
            double ORI;
            double MAG;
        } MEASUREMENT;
    } COV;
    // c. motion constraint:
    struct {
        bool ACTIVATED;
        double W_B_THRESH;
    } MOTION_CONSTRAINT;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_KALMAN_FILTER_HPP_