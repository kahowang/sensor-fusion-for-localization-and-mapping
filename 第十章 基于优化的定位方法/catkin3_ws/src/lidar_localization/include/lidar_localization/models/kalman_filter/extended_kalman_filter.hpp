/*
 * @Description: Extended Kalman Filter for IMU-Lidar-GNSS-Odo-Mag fusion
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */

#ifndef LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_EXTENDED_KALMAN_FILTER_HPP_
#define LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_EXTENDED_KALMAN_FILTER_HPP_

#include "lidar_localization/models/kalman_filter/kalman_filter.hpp"

namespace lidar_localization {

class ExtendedKalmanFilter : public KalmanFilter {
public:
    ExtendedKalmanFilter(const YAML::Node& node);
    
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
     * @brief  update state & covariance estimation, Kalman prediction
     * @param  imu_data, input IMU measurements
     * @return true if success false otherwise
     */
    bool Update(
        const IMUData &imu_data
    );

    /**
     * @brief  correct state & covariance estimation, Kalman correction
     * @param  measurement_type, input measurement type
     * @param  measurement, input measurement
     * @return void                                   
     */
    bool Correct(
        const IMUData &imu_data, 
        const MeasurementType &measurement_type, const Measurement &measurement
    );

    /**
     * @brief  get odometry estimation
     * @param  pose, output pose
     * @param  vel, output vel
     * @return void
     */
    void GetOdometry(Eigen::Matrix4f &pose, Eigen::Vector3f &vel);

    /**
     * @brief  get covariance estimation
     * @param  cov, output covariance 
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
    static const int DIM_STATE = 16;
    static const int DIM_PROCESS_NOISE = 6;

    static const int DIM_MEASUREMENT_POSE = 7;
    static const int DIM_MEASUREMENT_POSE_NOISE = 7;
    static const int DIM_MEASUREMENT_POSE_VEL = 10;
    static const int DIM_MEASUREMENT_POSE_VEL_NOISE = 10;
    static const int DIM_MEASUREMENT_POSI = 3;
    static const int DIM_MEASUREMENT_POSI_NOISE = 3;
    static const int DIM_MEASUREMENT_POSI_VEL = 6;
    static const int DIM_MEASUREMENT_POSI_VEL_NOISE = 6;
    static const int DIM_MEASUREMENT_POSI_MAG = 6;
    static const int DIM_MEASUREMENT_POSI_MAG_NOISE = 6;
    static const int DIM_MEASUREMENT_POSI_VEL_MAG = 9;
    static const int DIM_MEASUREMENT_POSI_VEL_MAG_NOISE = 9;

    // indices:
    static const int INDEX_POS = 0;
    static const int INDEX_VEL = 3;
    static const int INDEX_ORI = 6;
    static const int INDEX_GYRO_BIAS = 10;
    static const int INDEX_ACCEL_BIAS = 13;
    
    // state:
    typedef Eigen::Matrix<double,                          DIM_STATE,                                  1> VectorX;
    typedef Eigen::Matrix<double,                          DIM_STATE,                          DIM_STATE> MatrixP;
    // process equation:
    typedef Eigen::Matrix<double,                          DIM_STATE,                          DIM_STATE> MatrixF;
    typedef Eigen::Matrix<double,                          DIM_STATE,                  DIM_PROCESS_NOISE> MatrixB;
    typedef Eigen::Matrix<double,                  DIM_PROCESS_NOISE,                  DIM_PROCESS_NOISE> MatrixQ;
    // measurement equation:
    typedef Eigen::Matrix<double,               DIM_MEASUREMENT_POSE,                          DIM_STATE> MatrixGPose;
    typedef Eigen::Matrix<double,           DIM_MEASUREMENT_POSE_VEL,                          DIM_STATE> MatrixGPoseVel;
    typedef Eigen::Matrix<double,               DIM_MEASUREMENT_POSI,                          DIM_STATE> MatrixGPosi;
    typedef Eigen::Matrix<double,           DIM_MEASUREMENT_POSI_VEL,                          DIM_STATE> MatrixGPosiVel;
    typedef Eigen::Matrix<double,           DIM_MEASUREMENT_POSI_MAG,                          DIM_STATE> MatrixGPosiMag;
    typedef Eigen::Matrix<double,       DIM_MEASUREMENT_POSI_VEL_MAG,                          DIM_STATE> MatrixGPosiVelMag;

    typedef Eigen::Matrix<double,               DIM_MEASUREMENT_POSE,         DIM_MEASUREMENT_POSI_NOISE> MatrixCPose;
    typedef Eigen::Matrix<double,           DIM_MEASUREMENT_POSE_VEL,     DIM_MEASUREMENT_POSI_VEL_NOISE> MatrixCPoseVel;
    typedef Eigen::Matrix<double,               DIM_MEASUREMENT_POSI,         DIM_MEASUREMENT_POSI_NOISE> MatrixCPosi;
    typedef Eigen::Matrix<double,           DIM_MEASUREMENT_POSI_VEL,     DIM_MEASUREMENT_POSI_VEL_NOISE> MatrixCPosiVel;
    typedef Eigen::Matrix<double,           DIM_MEASUREMENT_POSI_MAG,     DIM_MEASUREMENT_POSI_MAG_NOISE> MatrixCPosiMag;
    typedef Eigen::Matrix<double,       DIM_MEASUREMENT_POSI_VEL_MAG, DIM_MEASUREMENT_POSI_VEL_MAG_NOISE> MatrixCPosiVelMag;

    typedef Eigen::Matrix<double,         DIM_MEASUREMENT_POSE_NOISE,         DIM_MEASUREMENT_POSE_NOISE> MatrixRPose;
    typedef Eigen::Matrix<double,     DIM_MEASUREMENT_POSE_VEL_NOISE,     DIM_MEASUREMENT_POSE_VEL_NOISE> MatrixRPoseVel;
    typedef Eigen::Matrix<double,         DIM_MEASUREMENT_POSI_NOISE,         DIM_MEASUREMENT_POSI_NOISE> MatrixRPosi;
    typedef Eigen::Matrix<double,     DIM_MEASUREMENT_POSI_VEL_NOISE,     DIM_MEASUREMENT_POSI_VEL_NOISE> MatrixRPosiVel;
    typedef Eigen::Matrix<double,     DIM_MEASUREMENT_POSI_MAG_NOISE,     DIM_MEASUREMENT_POSI_MAG_NOISE> MatrixRPosiMag;
    typedef Eigen::Matrix<double, DIM_MEASUREMENT_POSI_VEL_MAG_NOISE, DIM_MEASUREMENT_POSI_VEL_MAG_NOISE> MatrixRPosiVelMag;

    // measurement:
    typedef Eigen::Matrix<double,           DIM_MEASUREMENT_POSE,                              1> VectorYPose;
    typedef Eigen::Matrix<double,       DIM_MEASUREMENT_POSE_VEL,                              1> VectorYPoseVel;
    typedef Eigen::Matrix<double,           DIM_MEASUREMENT_POSI,                              1> VectorYPosi;
    typedef Eigen::Matrix<double,       DIM_MEASUREMENT_POSI_VEL,                              1> VectorYPosiVel;
    typedef Eigen::Matrix<double,       DIM_MEASUREMENT_POSI_MAG,                              1> VectorYPosiMag;
    typedef Eigen::Matrix<double,   DIM_MEASUREMENT_POSI_VEL_MAG,                              1> VectorYPosiVelMag;

    // Kalman gain:
    typedef Eigen::Matrix<double,                      DIM_STATE,           DIM_MEASUREMENT_POSE> MatrixKPose;
    typedef Eigen::Matrix<double,                      DIM_STATE,       DIM_MEASUREMENT_POSE_VEL> MatrixKPoseVel;
    typedef Eigen::Matrix<double,                      DIM_STATE,           DIM_MEASUREMENT_POSI> MatrixKPosi;
    typedef Eigen::Matrix<double,                      DIM_STATE,       DIM_MEASUREMENT_POSI_VEL> MatrixKPosiVel;
    typedef Eigen::Matrix<double,                      DIM_STATE,       DIM_MEASUREMENT_POSI_MAG> MatrixKPosiMag;
    typedef Eigen::Matrix<double,                      DIM_STATE,   DIM_MEASUREMENT_POSI_VEL_MAG> MatrixKPosiVelMag;

    // state observality matrix:
    typedef Eigen::Matrix<double, DIM_STATE*        DIM_MEASUREMENT_POSE, DIM_STATE> MatrixQPose;
    typedef Eigen::Matrix<double, DIM_STATE*    DIM_MEASUREMENT_POSE_VEL, DIM_STATE> MatrixQPoseVel;
    typedef Eigen::Matrix<double, DIM_STATE*        DIM_MEASUREMENT_POSI, DIM_STATE> MatrixQPosi;
    typedef Eigen::Matrix<double, DIM_STATE*    DIM_MEASUREMENT_POSI_VEL, DIM_STATE> MatrixQPosiVel;
    typedef Eigen::Matrix<double, DIM_STATE*    DIM_MEASUREMENT_POSI_MAG, DIM_STATE> MatrixQPosiMag;
    typedef Eigen::Matrix<double, DIM_STATE*DIM_MEASUREMENT_POSI_VEL_MAG, DIM_STATE> MatrixQPosVelMag;

    /**
     * @brief  get block matrix for velocity update by orientation quaternion
     * @param  f_b, accel measurement
     * @param  q_nb, orientation quaternion
     * @return block matrix Fvq
     */
    Eigen::Matrix<double, 3, 4> GetFVelOri(
        const Eigen::Vector3d &f_b,
        const Eigen::Quaterniond &q_nb
    );
    /**
     * @brief  get block matrix for orientation quaternion update by orientation quaternion
     * @param  w_b, gyro measurement
     * @return block matrix Fqq
     */
    Eigen::Matrix<double, 4, 4> GetFOriOri(
        const Eigen::Vector3d &w_b
    );
    /**
     * @brief  get block matrix for orientation quaternion update by epsilon, angular velocity bias
     * @param  q_nb, orientation quaternion
     * @return block matrix Fqe
     */
    Eigen::Matrix<double, 4, 3> GetFOriEps(
        const Eigen::Quaterniond &q_nb
    );
    /**
     * @brief  set process equation
     * @param  void
     * @return void
     */
    void SetProcessEquation(const IMUData &imu_data);
    /**
     * @brief  update covariance estimation
     * @param  void
     * @return void
     */
    void UpdateCovarianceEstimation(const IMUData &imu_data);

    /**
     * @brief  get unbiased angular velocity in body frame
     * @param  angular_vel, angular velocity measurement
     * @param  C_nb, corresponding orientation of measurement
     * @return unbiased angular velocity in body frame
     */
    inline Eigen::Vector3d GetUnbiasedAngularVel(
        const Eigen::Vector3d &angular_vel,
        const Eigen::Matrix3d &C_nb
    );
    /**
     * @brief  get unbiased linear acceleration in navigation frame
     * @param  linear_acc, linear acceleration measurement
     * @param  C_nb, corresponding orientation of measurement
     * @return unbiased linear acceleration in navigation frame
     */
    inline Eigen::Vector3d GetUnbiasedLinearAcc(
        const Eigen::Vector3d &linear_acc,
        const Eigen::Matrix3d &C_nb
    );
    /**
     * @brief  remove gravity component from accel measurement
     * @param  f_b, accel measurement measurement
     * @param  C_nb, orientation matrix
     * @return f_b
     */
    inline Eigen::Vector3d RemoveGravity(
        const Eigen::Vector3d &f_b,
        const Eigen::Matrix3d &C_nb
    );

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
        Eigen::Vector3d &velocity_delta
    );
    /**
     * @brief  update orientation with effective velocity change velocity_delta
     * @param  T, timestamp delta 
     * @param  velocity_delta, effective velocity change
     * @return void
     */
    void UpdatePosition(const double &T, const Eigen::Vector3d &velocity_delta);
    
    /**
     * @brief  update state estimation
     * @param  void
     * @return void
     */
    void UpdateStateEstimation(void);

    /**
     * @brief  correct state estimation using frontend pose
     * @param  T_nb, input GNSS position
     * @return void
     */
    void CorrectStateEstimationPose(
        const Eigen::Matrix4d &T_nb
    );

    /**
     * @brief  correct state estimation using frontend pose & body velocity measurement
     * @param  T_nb, input frontend pose estimation
     * @param  v_b, input odo
     * @return void
     */
    void CorrectStateEstimationPoseVel(
        const Eigen::Matrix4d &T_nb, 
        const Eigen::Vector3d &v_b
    );

    /**
     * @brief  correct state estimation using GNSS position
     * @param  T_nb, input GNSS position
     * @return void
     */
    void CorrectStateEstimationPosi(const Eigen::Matrix4d &T_nb);

    /**
     * @brief  get block matrix for observation by orientation quaternion
     * @param  m_n, measurement in navigation frame
     * @param  q_nb, orientation quaternion
     * @return block matrix Gq
     */
    Eigen::Matrix<double, 3, 4> GetGMOri(
        const Eigen::Vector3d &m_n,
        const Eigen::Quaterniond &q_nb
    );

    /**
     * @brief  correct state estimation using GNSS position and odometer measurement
     * @param  T_nb, input GNSS position 
     * @param  v_b, input odo
     * @return void
     */
    void CorrectStateEstimationPosiVel(
        const Eigen::Matrix4d &T_nb, const Eigen::Vector3d &v_b
    );

    /**
     * @brief  correct state estimation using GNSS position and magneto measurement
     * @param  T_nb, input GNSS position 
     * @param  B_b, input magneto
     * @return void
     */
    void CorrectStateEstimationPosiMag(
        const Eigen::Matrix4d &T_nb, const Eigen::Vector3d &B_b);

    /**
     * @brief  correct state estimation using GNSS position, odometer and magneto measurement
     * @param  T_nb, input GNSS position 
     * @param  v_b, input odo
     * @param  B_b, input magneto
     * @return void
     */
    void CorrectStateEstimationPosiVelMag(
        const Eigen::Matrix4d &T_nb, const Eigen::Vector3d &v_b, const Eigen::Vector3d &B_b
    );

    /**
     * @brief  correct state estimation
     * @param  measurement_type, measurement type
     * @param  measurement, input measurement
     * @return void
     */
    void CorrectStateEstimation(
        const MeasurementType &measurement_type, 
        const Measurement &measurement
    );

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
     * @brief  get Q for pose measurement
     * @param  void
     * @return void
     */
    void GetQPose(Eigen::MatrixXd &Q);

    /**
     * @brief  get Q for pose & body velocity measurement
     * @param  void
     * @return void
     */
    void GetQPoseVel(Eigen::MatrixXd &Q);

    /**
     * @brief  get Q for GNSS position measurement
     * @param  void
     * @return QPosi
     */
    void GetQPosi(Eigen::MatrixXd &Q);

    /**
     * @brief  get Q for GNSS position & body velocity measurement
     * @param  void
     * @return QPosiVel
     */
    void GetQPosiVel(Eigen::MatrixXd &Q);

    /**
     * @brief  get Q for GNSS position & magneto measurement
     * @param  void
     * @return QPosiMag
     */
    void GetQPosiMag(Eigen::MatrixXd &Q);

    /**
     * @brief  get Q for GNSS position, body velocity & magneto measurement
     * @param  void
     * @return QPosiVelMag
     */
    void GetQPosiVelMag(Eigen::MatrixXd &Q);
     
    // init pose, vel, gyro & accel bias:
    Eigen::Matrix4d init_pose_ = Eigen::Matrix4d::Identity();
    Eigen::Vector3d init_vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();

    // state:
    VectorX X_ = VectorX::Zero();
    MatrixP P_ = MatrixP::Zero();
    // process & measurement equations:
    MatrixF F_ = MatrixF::Zero();
    MatrixB B_ = MatrixB::Zero();
    MatrixQ Q_ = MatrixQ::Zero();

    MatrixGPose GPose_ = MatrixGPose::Zero();
    MatrixGPoseVel GPoseVel_ = MatrixGPoseVel::Zero();
    MatrixGPosi GPosi_ = MatrixGPosi::Zero();
    MatrixGPosiVel GPosiVel_ = MatrixGPosiVel::Zero();
    MatrixGPosiMag GPosiMag_ = MatrixGPosiMag::Zero();
    MatrixGPosiVelMag GPosiVelMag_ = MatrixGPosiVelMag::Zero();

    MatrixCPose CPose_ = MatrixCPose::Zero();
    MatrixCPoseVel CPoseVel_ = MatrixCPoseVel::Zero();
    MatrixCPosi CPosi_ = MatrixCPosi::Zero();
    MatrixCPosiVel CPosiVel_ = MatrixCPosiVel::Zero();
    MatrixCPosiMag CPosiMag_ = MatrixCPosiMag::Zero();
    MatrixCPosiVelMag CPosiVelMag_ = MatrixCPosiVelMag::Zero();

    MatrixRPose RPose_ = MatrixRPose::Zero();
    MatrixRPoseVel RPoseVel_ = MatrixRPoseVel::Zero();
    MatrixRPosi RPosi_ = MatrixRPosi::Zero();
    MatrixRPosiVel RPosiVel_ = MatrixRPosiVel::Zero();
    MatrixRPosiMag RPosiMag_ = MatrixRPosiMag::Zero();
    MatrixRPosiVelMag RPosiVelMag_ = MatrixRPosiVelMag::Zero();

    MatrixQPose QPose_ = MatrixQPose::Zero();
    MatrixQPoseVel QPoseVel_ = MatrixQPoseVel::Zero();
    MatrixQPosi QPosi_ = MatrixQPosi::Zero();
    MatrixQPosiVel QPosiVel_ = MatrixQPosiVel::Zero();
    MatrixQPosiMag QPosiMag_ = MatrixQPosiMag::Zero();
    MatrixQPosVelMag QPosVelMag_ = MatrixQPosVelMag::Zero();

    // measurement:
    VectorYPose YPose_;
    VectorYPoseVel YPoseVel_;
    VectorYPosi YPosi_;
    VectorYPosiVel YPosiVel_;
    VectorYPosiMag YPosiMag_;
    VectorYPosiVelMag YPosiVelMag_;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MODELS_KALMAN_FILTER_EXTENDED_KALMAN_FILTER_HPP_