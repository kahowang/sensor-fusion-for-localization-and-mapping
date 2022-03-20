/*
 * @Description: Error-State Kalman Filter for IMU-Lidar-GNSS-Odo fusion
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */

#ifndef LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_ERROR_STATE_KALMAN_FILTER_HPP_
#define LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_ERROR_STATE_KALMAN_FILTER_HPP_

#include "lidar_localization/models/kalman_filter/kalman_filter.hpp"

namespace lidar_localization {

class ErrorStateKalmanFilter : public KalmanFilter {
public:
    ErrorStateKalmanFilter(const YAML::Node& node);

    /**
     * @brief  init filter
     * @param  imu_data, input IMU measurements
     * @return true if success false otherwise
     */
    void Init(
        const Eigen::Vector3d &vel,
        const IMUData &imu_data
    );

    /**
     * @brief  Kalman update
     * @param  imu_data, input IMU measurements
     * @return true if success false otherwise
     */
    bool Update(
        const IMUData &imu_data
    );

    /**
     * @brief  Kalman correction, pose measurement
     * @param  measurement_type, input measurement type
     * @param  measurement, input measurement
     * @return void                                   
     */
    bool Correct(
        const IMUData &imu_data, 
        const MeasurementType &measurement_type, const Measurement &measurement
    );

    /**
     * @brief  Kalman correction, pose measurement and other measurement in body frame
     * @param  T_nb, pose measurement
     * @param  v_b, velocity or magnetometer measurement
     * @return void                                   
     */
    bool Correct(
        const IMUData &imu_data, 
        const double &time, const MeasurementType &measurement_type, 
        const Eigen::Matrix4f &T_nb, const Eigen::Vector3f &v_b
    );
    
    /**
     * @brief  get odometry estimation
     * @param  pose, init pose
     * @param  vel, init vel
     * @return void
     */
    void GetOdometry(Eigen::Matrix4f &pose, Eigen::Vector3f &vel);

    /**
     * @brief  get covariance estimation
     * @param  cov, covariance output
     * @return void
     */
    void GetCovariance(Cov &cov);
    
    /**
     * @brief  update observability analysis
     * @param  time, measurement time
     * @param  measurement_type, measurement type
     * @return void
     */
    void UpdateObservabilityAnalysis(
        const double &time,
        const MeasurementType &measurement_type
    );

    /**
     * @brief  save observability analysis to persistent storage
     * @param  measurement_type, measurement type
     * @return void
     */
    bool SaveObservabilityAnalysis(
        const MeasurementType &measurement_type
    );

private:
    // dimensions:
    static const int DIM_STATE = 15;
    static const int DIM_PROCESS_NOISE = 6;

    static const int DIM_MEASUREMENT_POSE = 6;
    static const int DIM_MEASUREMENT_POSE_NOISE = 6;
    static const int DIM_MEASUREMENT_POSE_VEL = 9;
    static const int DIM_MEASUREMENT_POSE_VEL_NOISE = 9;
    static const int DIM_MEASUREMENT_POSI = 3;
    static const int DIM_MEASUREMENT_POSI_NOISE = 3;
    static const int DIM_MEASUREMENT_POSI_VEL = 6;
    static const int DIM_MEASUREMENT_POSI_VEL_NOISE = 6;

    // indices:
    static const int INDEX_ERROR_POS = 0;
    static const int INDEX_ERROR_VEL = 3;
    static const int INDEX_ERROR_ORI = 6;
    static const int INDEX_ERROR_GYRO = 9;
    static const int INDEX_ERROR_ACCEL = 12;
    
    // state:
    typedef Eigen::Matrix<double,                      DIM_STATE,                              1> VectorX;
    typedef Eigen::Matrix<double,                      DIM_STATE,                      DIM_STATE> MatrixP;
    // process equation:
    typedef Eigen::Matrix<double,                      DIM_STATE,                      DIM_STATE> MatrixF;
    typedef Eigen::Matrix<double,                      DIM_STATE,              DIM_PROCESS_NOISE> MatrixB;
    typedef Eigen::Matrix<double,              DIM_PROCESS_NOISE,              DIM_PROCESS_NOISE> MatrixQ;
    // measurement equation:
    typedef Eigen::Matrix<double,           DIM_MEASUREMENT_POSE,                      DIM_STATE> MatrixGPose;
    typedef Eigen::Matrix<double,       DIM_MEASUREMENT_POSE_VEL,                      DIM_STATE> MatrixGPoseVel;
    typedef Eigen::Matrix<double,   DIM_MEASUREMENT_POSE_VEL - 1,                      DIM_STATE> MatrixGPoseVelCons;
    typedef Eigen::Matrix<double,           DIM_MEASUREMENT_POSI,                      DIM_STATE> MatrixGPosi;
    typedef Eigen::Matrix<double,       DIM_MEASUREMENT_POSI_VEL,                      DIM_STATE> MatrixGPosiVel;
    typedef Eigen::Matrix<double,   DIM_MEASUREMENT_POSI_VEL - 1,                      DIM_STATE> MatrixGPosiVelCons;

    typedef Eigen::Matrix<double,           DIM_MEASUREMENT_POSE,     DIM_MEASUREMENT_POSE_NOISE> MatrixCPose;
    typedef Eigen::Matrix<double,       DIM_MEASUREMENT_POSE_VEL, DIM_MEASUREMENT_POSE_VEL_NOISE> MatrixCPoseVel;
    typedef Eigen::Matrix<double,   DIM_MEASUREMENT_POSE_VEL - 1, DIM_MEASUREMENT_POSE_VEL_NOISE> MatrixCPoseVelCons;
    typedef Eigen::Matrix<double,           DIM_MEASUREMENT_POSI,     DIM_MEASUREMENT_POSI_NOISE> MatrixCPosi;
    typedef Eigen::Matrix<double,       DIM_MEASUREMENT_POSI_VEL, DIM_MEASUREMENT_POSI_VEL_NOISE> MatrixCPosiVel;
    typedef Eigen::Matrix<double,   DIM_MEASUREMENT_POSI_VEL - 1, DIM_MEASUREMENT_POSI_VEL_NOISE> MatrixCPosiVelCons;

    typedef Eigen::Matrix<double,         DIM_MEASUREMENT_POSE_NOISE,         DIM_MEASUREMENT_POSE_NOISE> MatrixRPose;
    typedef Eigen::Matrix<double,     DIM_MEASUREMENT_POSE_VEL_NOISE,     DIM_MEASUREMENT_POSE_VEL_NOISE> MatrixRPoseVel;
    typedef Eigen::Matrix<double, DIM_MEASUREMENT_POSE_VEL_NOISE - 1, DIM_MEASUREMENT_POSE_VEL_NOISE - 1> MatrixRPoseVelCons;
    typedef Eigen::Matrix<double,         DIM_MEASUREMENT_POSI_NOISE,         DIM_MEASUREMENT_POSI_NOISE> MatrixRPosi;
    typedef Eigen::Matrix<double,     DIM_MEASUREMENT_POSI_VEL_NOISE,     DIM_MEASUREMENT_POSI_VEL_NOISE> MatrixRPosiVel;
    typedef Eigen::Matrix<double, DIM_MEASUREMENT_POSI_VEL_NOISE - 1, DIM_MEASUREMENT_POSI_VEL_NOISE - 1> MatrixRPosiVelCons;

    // measurement:
    typedef Eigen::Matrix<double,           DIM_MEASUREMENT_POSE,                              1> VectorYPose;
    typedef Eigen::Matrix<double,       DIM_MEASUREMENT_POSE_VEL,                              1> VectorYPoseVel;
    typedef Eigen::Matrix<double,   DIM_MEASUREMENT_POSE_VEL - 1,                              1> VectorYPoseVelCons;
    typedef Eigen::Matrix<double,           DIM_MEASUREMENT_POSI,                              1> VectorYPosi;
    typedef Eigen::Matrix<double,       DIM_MEASUREMENT_POSI_VEL,                              1> VectorYPosiVel;
    typedef Eigen::Matrix<double,   DIM_MEASUREMENT_POSI_VEL - 1,                              1> VectorYPosiVelCons;
    // Kalman gain:
    typedef Eigen::Matrix<double,                      DIM_STATE,           DIM_MEASUREMENT_POSE> MatrixKPose;
    typedef Eigen::Matrix<double,                      DIM_STATE,       DIM_MEASUREMENT_POSE_VEL> MatrixKPoseVel;
    typedef Eigen::Matrix<double,                      DIM_STATE,   DIM_MEASUREMENT_POSE_VEL - 1> MatrixKPoseVelCons;
    typedef Eigen::Matrix<double,                      DIM_STATE,           DIM_MEASUREMENT_POSI> MatrixKPosi;
    typedef Eigen::Matrix<double,                      DIM_STATE,       DIM_MEASUREMENT_POSI_VEL> MatrixKPosiVel;
    typedef Eigen::Matrix<double,                      DIM_STATE,   DIM_MEASUREMENT_POSI_VEL - 1> MatrixKPosiVelCons;

    // state observality matrix:
    typedef Eigen::Matrix<double,     DIM_STATE*DIM_MEASUREMENT_POSE, DIM_STATE> MatrixQPose;
    typedef Eigen::Matrix<double, DIM_STATE*DIM_MEASUREMENT_POSE_VEL, DIM_STATE> MatrixQPoseVel;
    typedef Eigen::Matrix<double,     DIM_STATE*DIM_MEASUREMENT_POSI, DIM_STATE> MatrixQPosi;
    typedef Eigen::Matrix<double, DIM_STATE*DIM_MEASUREMENT_POSI_VEL, DIM_STATE> MatrixQPosiVel;

    /**
     * @brief  get unbiased angular velocity in body frame
     * @param  angular_vel, angular velocity measurement
     * @param  R, corresponding orientation of measurement
     * @return unbiased angular velocity in body frame
     */
    Eigen::Vector3d GetUnbiasedAngularVel(
        const Eigen::Vector3d &angular_vel,
        const Eigen::Matrix3d &R
    );
    /**
     * @brief  get unbiased linear acceleration in navigation frame
     * @param  linear_acc, linear acceleration measurement
     * @param  R, corresponding orientation of measurement
     * @return unbiased linear acceleration in navigation frame
     */
    Eigen::Vector3d GetUnbiasedLinearAcc(
        const Eigen::Vector3d &linear_acc,
        const Eigen::Matrix3d &R
    );
    bool IsTurning(const Eigen::Vector3d &w_b);
    /**
     * @brief  apply motion constraint on velocity estimation
     * @param  void
     * @return void
     */
    void ApplyMotionConstraint(void);
    /**
     * @brief  get angular delta
     * @param  index_curr, current imu measurement buffer index
     * @param  index_prev, previous imu measurement buffer index
     * @param  angular_delta, angular delta output
     * @return true if success false otherwise
     */
    bool GetAngularDelta(
        const size_t index_curr, const size_t index_prev,
        Eigen::Vector3d &angular_delta
    );
    /**
     * @brief  get velocity delta
     * @param  index_curr, current imu measurement buffer index
     * @param  index_prev, previous imu measurement buffer index
     * @param  R_curr, corresponding orientation of current imu measurement
     * @param  R_prev, corresponding orientation of previous imu measurement
     * @param  velocity_delta, velocity delta output
     * @param  linear_acc_mid, mid-value unbiased linear acc
     * @return true if success false otherwise
     */
    bool GetVelocityDelta(
        const size_t index_curr, const size_t index_prev,
        const Eigen::Matrix3d &R_curr, const Eigen::Matrix3d &R_prev, 
        double &T, 
        Eigen::Vector3d &velocity_delta,
        Eigen::Vector3d &linear_acc_mid
    );
    /**
     * @brief  update orientation with effective rotation angular_delta
     * @param  angular_delta, effective rotation
     * @param  R_curr, current orientation
     * @param  R_prev, previous orientation
     * @return void
     */
    void UpdateOrientation(
        const Eigen::Vector3d &angular_delta,
        Eigen::Matrix3d &R_curr, Eigen::Matrix3d &R_prev
    );
    /**
     * @brief  update orientation with effective velocity change velocity_delta
     * @param  velocity_delta, effective velocity change
     * @return void
     */
    void UpdatePosition(const double &T, const Eigen::Vector3d &velocity_delta);
    /**
     * @brief  update IMU odometry estimation
     * @param  linear_acc_mid, output mid-value unbiased linear acc
     * @return void
     */
    void UpdateOdomEstimation(Eigen::Vector3d &linear_acc_mid);

    /**
     * @brief  set process equation
     * @param  C_nb, rotation matrix, body frame -> navigation frame
     * @param  f_n, accel measurement in navigation frame
     * @return void
     */
    void SetProcessEquation(
        const Eigen::Matrix3d &C_nb, const Eigen::Vector3d &f_n
    );
    /**
     * @brief  update process equation
     * @param  linear_acc_mid, input mid-value unbiased linear acc
     * @return void
     */
    void UpdateProcessEquation(const Eigen::Vector3d &linear_acc_mid);

    /**
     * @brief  update error estimation
     * @param  linear_acc_mid, input mid-value unbiased linear acc
     * @return void
     */
    void UpdateErrorEstimation(
        const double &T,
        const Eigen::Vector3d &linear_acc_mid
    );

    /**
     * @brief  correct error estimation using pose measurement
     * @param  T_nb, input pose measurement
     * @return void
     */
    void CorrectErrorEstimationPose(
        const Eigen::Matrix4d &T_nb,
        Eigen::VectorXd &Y, Eigen::MatrixXd &G, Eigen::MatrixXd &K
    );

    /**
     * @brief  correct error estimation using pose and body velocity measurement
     * @param  T_nb, input pose measurement
     * @param  v_b, input velocity measurement
     * @return void
     */
    void CorrectErrorEstimationPoseVel(
        const Eigen::Matrix4d &T_nb, const Eigen::Vector3d &v_b, const Eigen::Vector3d &w_b,
        Eigen::VectorXd &Y, Eigen::MatrixXd &G, Eigen::MatrixXd &K
    );

    /**
     * @brief  correct error estimation using position measurement
     * @param  T_nb, input position measurement
     * @return void
     */
    void CorrectErrorEstimationPosi(
        const Eigen::Matrix4d &T_nb,
        Eigen::VectorXd &Y, Eigen::MatrixXd &G, Eigen::MatrixXd &K
    );

    /**
     * @brief  correct error estimation using navigation position and body velocity measurement
     * @param  T_nb, input position measurement
     * @param  v_b, input velocity measurement
     * @return void
     */
    void CorrectErrorEstimationPosiVel(
        const Eigen::Matrix4d &T_nb, const Eigen::Vector3d &v_b, const Eigen::Vector3d &w_b,
        Eigen::VectorXd &Y, Eigen::MatrixXd &G, Eigen::MatrixXd &K
    );

    /**
     * @brief  correct error estimation
     * @param  measurement_type, measurement type
     * @param  measurement, input measurement
     * @return void
     */
    void CorrectErrorEstimation(
        const MeasurementType &measurement_type, 
        const Measurement &measurement
    );

    /**
     * @brief  eliminate error
     * @param  void
     * @return void
     */
    void EliminateError(void);

    /**
     * @brief  is covariance stable
     * @param  INDEX_OFSET, state index offset
     * @param  THRESH, covariance threshold, defaults to 1.0e-5
     * @return void
     */
    bool IsCovStable(const int INDEX_OFSET, const double THRESH = 1.0e-5);

    /**
     * @brief  reset filter state
     * @param  void
     * @return void
     */
    void ResetState(void);
    /**
     * @brief  reset filter covariance
     * @param  void
     * @return void
     */
    void ResetCovariance(void);

    /**
     * @brief  get Q analysis for pose measurement
     * @param  void
     * @return void
     */
    Eigen::MatrixXd GetQPose(void);

    /**
     * @brief  get Q for pose & body velocity measurement
     * @param  void
     * @return QPoseVel
     */
    Eigen::MatrixXd GetQPoseVel(void);

    /**
     * @brief  get Q for position measurement
     * @param  void
     * @return QPos
     */
    Eigen::MatrixXd GetQPosi(void);

    /**
     * @brief  get Q for navigation position & body velocity measurement
     * @param  void
     * @return void
     */
    Eigen::MatrixXd GetQPosiVel(void);

    // odometry estimation from IMU integration:
    Eigen::Matrix4d init_pose_ = Eigen::Matrix4d::Identity();
    
    Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();
    Eigen::Vector3d vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d accl_bias_ = Eigen::Vector3d::Zero();

    // state:
    VectorX X_ = VectorX::Zero();
    MatrixP P_ = MatrixP::Zero();
    // process & measurement equations:
    MatrixF F_ = MatrixF::Zero();
    MatrixB B_ = MatrixB::Zero();
    MatrixQ Q_ = MatrixQ::Zero();

    MatrixGPose GPose_ = MatrixGPose::Zero();
    MatrixGPoseVel GPoseVel_ = MatrixGPoseVel::Zero();
    MatrixGPoseVelCons GPoseVelCons_ = MatrixGPoseVelCons::Zero();
    MatrixGPosi GPosi_ = MatrixGPosi::Zero();
    MatrixGPosiVel GPosiVel_ = MatrixGPosiVel::Zero();
    MatrixGPosiVelCons GPosiVelCons_ = MatrixGPosiVelCons::Zero();
    
    MatrixCPose CPose_ = MatrixCPose::Zero();
    MatrixCPoseVel CPoseVel_ = MatrixCPoseVel::Zero();
    MatrixCPoseVelCons CPoseVelCons_ = MatrixCPoseVelCons::Zero();
    MatrixCPosi CPosi_ = MatrixCPosi::Zero();
    MatrixCPosiVel CPosiVel_ = MatrixCPosiVel::Zero();
    MatrixCPosiVelCons CPosiVelCons_ = MatrixCPosiVelCons::Zero();

    MatrixRPose RPose_ = MatrixRPose::Zero();
    MatrixRPoseVel RPoseVel_ = MatrixRPoseVel::Zero();
    MatrixRPosi RPosi_ = MatrixRPosi::Zero();
    MatrixRPosiVel RPosiVel_ = MatrixRPosiVel::Zero();

    MatrixQPose QPose_ = MatrixQPose::Zero();
    MatrixQPoseVel QPoseVel_ = MatrixQPoseVel::Zero();
    MatrixQPosi QPosi_ = MatrixQPosi::Zero();
    MatrixQPosiVel QPosiVel_ = MatrixQPosiVel::Zero();

    // measurement:
    VectorYPose YPose_;
    VectorYPoseVel YPoseVel_;
    VectorYPosi YPosi_;
    VectorYPosiVel YPosiVel_;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_ERROR_STATE_KALMAN_FILTER_HPP_