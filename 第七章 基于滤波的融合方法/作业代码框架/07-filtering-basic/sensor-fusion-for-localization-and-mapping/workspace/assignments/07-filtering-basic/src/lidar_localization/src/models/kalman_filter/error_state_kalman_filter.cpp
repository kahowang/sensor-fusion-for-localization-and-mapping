/*
 * @Description: Error-State Kalman Filter for IMU-Lidar-GNSS-Odo fusion
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */
#include <cstdlib>
#include <limits>

#include <cmath>
#include <fstream>
#include <iostream>
#include <ostream>

// use sophus to handle so3 hat & SO3 log operations:
#include <sophus/so3.hpp>

#include "lidar_localization/models/kalman_filter/error_state_kalman_filter.hpp"

#include "lidar_localization/global_defination/global_defination.h"

#include "glog/logging.h"

namespace lidar_localization {

ErrorStateKalmanFilter::ErrorStateKalmanFilter(const YAML::Node &node) {
  //
  // parse config:
  //
  // a. earth constants:
  EARTH.GRAVITY_MAGNITUDE = node["earth"]["gravity_magnitude"].as<double>();
  EARTH.LATITUDE = node["earth"]["latitude"].as<double>();
  EARTH.LATITUDE *= M_PI / 180.0;

  // b. prior state covariance:
  COV.PRIOR.POSI = node["covariance"]["prior"]["pos"].as<double>();
  COV.PRIOR.VEL = node["covariance"]["prior"]["vel"].as<double>();
  COV.PRIOR.ORI = node["covariance"]["prior"]["ori"].as<double>();
  COV.PRIOR.EPSILON = node["covariance"]["prior"]["epsilon"].as<double>();
  COV.PRIOR.DELTA = node["covariance"]["prior"]["delta"].as<double>();

  // c. process noise:
  COV.PROCESS.ACCEL = node["covariance"]["process"]["accel"].as<double>();
  COV.PROCESS.GYRO = node["covariance"]["process"]["gyro"].as<double>();
  COV.PROCESS.BIAS_ACCEL =
      node["covariance"]["process"]["bias_accel"].as<double>();
  COV.PROCESS.BIAS_GYRO =
      node["covariance"]["process"]["bias_gyro"].as<double>();

  // d. measurement noise:
  COV.MEASUREMENT.POSE.POSI =
      node["covariance"]["measurement"]["pose"]["pos"].as<double>();
  COV.MEASUREMENT.POSE.ORI =
      node["covariance"]["measurement"]["pose"]["ori"].as<double>();

  // prompt:
  LOG(INFO) << std::endl
            << "Error-State Kalman Filter params:" << std::endl
            << "\tgravity magnitude: " << EARTH.GRAVITY_MAGNITUDE << std::endl
            << "\tlatitude: " << EARTH.LATITUDE << std::endl
            << std::endl
            << "\tprior cov. pos.: " << COV.PRIOR.POSI << std::endl
            << "\tprior cov. vel.: " << COV.PRIOR.VEL << std::endl
            << "\tprior cov. ori: " << COV.PRIOR.ORI << std::endl
            << "\tprior cov. epsilon.: " << COV.PRIOR.EPSILON << std::endl
            << "\tprior cov. delta.: " << COV.PRIOR.DELTA << std::endl
            << std::endl
            << "\tprocess noise gyro.: " << COV.PROCESS.GYRO << std::endl
            << "\tprocess noise accel.: " << COV.PROCESS.ACCEL << std::endl
            << std::endl
            << "\tmeasurement noise pose.: " << std::endl
            << "\t\tpos: " << COV.MEASUREMENT.POSE.POSI
            << ", ori.: " << COV.MEASUREMENT.POSE.ORI << std::endl
            << std::endl
            << std::endl;

  //
  // init filter:
  //
  // a. earth constants:
  g_ = Eigen::Vector3d(0.0, 0.0, EARTH.GRAVITY_MAGNITUDE);
  // b. prior state & covariance:
  ResetState();
  ResetCovariance();

  // c. process noise:
  Q_.block<3, 3>(kIndexNoiseAccel, kIndexNoiseAccel) = COV.PROCESS.ACCEL * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(kIndexNoiseGyro, kIndexNoiseGyro) = COV.PROCESS.GYRO * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(kIndexNoiseBiasAccel, kIndexNoiseBiasAccel) = COV.PROCESS.BIAS_ACCEL * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(kIndexNoiseBiasGyro, kIndexNoiseBiasGyro) = COV.PROCESS.BIAS_GYRO * Eigen::Matrix3d::Identity();

  // d. measurement noise:
  RPose_.block<3, 3>(0, 0) = COV.MEASUREMENT.POSE.POSI * Eigen::Matrix3d::Identity();
  RPose_.block<3, 3>(3, 3) = COV.MEASUREMENT.POSE.ORI * Eigen::Matrix3d::Identity();

  // e. process equation:
  F_.block<3, 3>(kIndexErrorPos, kIndexErrorVel) = Eigen::Matrix3d::Identity();
  F_.block<3, 3>(kIndexErrorOri, kIndexErrorGyro) = -Eigen::Matrix3d::Identity();

  B_.block<3, 3>(kIndexErrorOri, kIndexNoiseGyro) = Eigen::Matrix3d::Identity();
  B_.block<3, 3>(kIndexErrorAccel, kIndexNoiseBiasAccel) = Eigen::Matrix3d::Identity();
  B_.block<3, 3>(kIndexErrorGyro, kIndexNoiseBiasGyro) = Eigen::Matrix3d::Identity();

  // f. measurement equation:
  GPose_.block<3, 3>(0, kIndexErrorPos) = Eigen::Matrix3d::Identity();
  GPose_.block<3, 3>(3, kIndexErrorOri) = Eigen::Matrix3d::Identity();
  CPose_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  CPose_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

  // init soms:
  QPose_.block<kDimMeasurementPose, kDimState>(0, 0) = GPose_;
}

/**
 * @brief  init filter
 * @param  pose, init pose
 * @param  vel, init vel
 * @param  imu_data, init IMU measurements
 * @return true if success false otherwise
 */
void ErrorStateKalmanFilter::Init(const Eigen::Vector3d &vel,
                                  const IMUData &imu_data) {
  // init odometry:
  Eigen::Matrix3d C_nb = imu_data.GetOrientationMatrix().cast<double>();
  // a. init C_nb using IMU estimation:
  pose_.block<3, 3>(0, 0) = C_nb;
  // b. convert flu velocity into navigation frame:
  vel_ = C_nb * vel;

  // save init pose:
  init_pose_ = pose_;

  // init IMU data buffer:
  imu_data_buff_.clear();
  imu_data_buff_.push_back(imu_data);

  // init filter time:
  time_ = imu_data.time;

  // set process equation in case of one step prediction & correction:
  Eigen::Vector3d linear_acc_init(imu_data.linear_acceleration.x,
                                  imu_data.linear_acceleration.y,
                                  imu_data.linear_acceleration.z);
  Eigen::Vector3d angular_vel_init(imu_data.angular_velocity.x,
                                   imu_data.angular_velocity.y,
                                   imu_data.angular_velocity.z);
  // covert to navigation frame:
  linear_acc_init = GetUnbiasedLinearAcc(linear_acc_init, C_nb);
  angular_vel_init = GetUnbiasedAngularVel(angular_vel_init, C_nb);

  // init process equation, in case of direct correct step:
  UpdateProcessEquation(linear_acc_init, angular_vel_init);

  LOG(INFO) << std::endl
            << "Kalman Filter Inited at " << static_cast<int>(time_)
            << std::endl
            << "Init Position: " << pose_(0, 3) << ", " << pose_(1, 3) << ", "
            << pose_(2, 3) << std::endl
            << "Init Velocity: " << vel_.x() << ", " << vel_.y() << ", "
            << vel_.z() << std::endl;
}

/**
 * @brief  Kalman update
 * @param  imu_data, input IMU measurements
 * @return true if success false otherwise
 */
bool ErrorStateKalmanFilter::Update(const IMUData &imu_data) {
  //
  // TODO: understand ESKF update workflow
  //
  // update IMU buff:
  if (time_ < imu_data.time) {
    // update IMU odometry:
    Eigen::Vector3d linear_acc_mid;
    Eigen::Vector3d angular_vel_mid;
    imu_data_buff_.push_back(imu_data);
    UpdateOdomEstimation(linear_acc_mid, angular_vel_mid);
    imu_data_buff_.pop_front();

    // update error estimation:
    double T = imu_data.time - time_;
    UpdateErrorEstimation(T, linear_acc_mid, angular_vel_mid);

    // move forward:
    time_ = imu_data.time;

    return true;
  }

  return false;
}

/**
 * @brief  Kalman correction, pose measurement and other measurement in body
 * frame
 * @param  measurement_type, input measurement type
 * @param  measurement, input measurement
 * @return void
 */
bool ErrorStateKalmanFilter::Correct(const IMUData &imu_data,
                                     const MeasurementType &measurement_type,
                                     const Measurement &measurement) {
  static Measurement measurement_;

  // get time delta:
  double time_delta = measurement.time - time_;

  if (time_delta > -0.05) {
    // perform Kalman prediction:
    if (time_ < measurement.time) {
      Update(imu_data);
    }

    // get observation in navigation frame:
    measurement_ = measurement;
    measurement_.T_nb = init_pose_ * measurement_.T_nb;

    // correct error estimation:
    CorrectErrorEstimation(measurement_type, measurement_);

    // eliminate error:
    EliminateError();

    // reset error state:
    ResetState();

    return true;
  }

  LOG(INFO) << "ESKF Correct: Observation is not synced with filter. Skip, "
            << (int)measurement.time << " <-- " << (int)time_ << " @ "
            << time_delta << std::endl;

  return false;
}

/**
 * @brief  get odometry estimation
 * @param  pose, init pose
 * @param  vel, init vel
 * @return void
 */
void ErrorStateKalmanFilter::GetOdometry(Eigen::Matrix4f &pose,
                                         Eigen::Vector3f &vel) {
  // init:
  Eigen::Matrix4d pose_double = pose_;
  Eigen::Vector3d vel_double = vel_;

  // eliminate error:
  // a. position:
  pose_double.block<3, 1>(0, 3) =
      pose_double.block<3, 1>(0, 3) - X_.block<3, 1>(kIndexErrorPos, 0);
  // b. velocity:
  vel_double = vel_double - X_.block<3, 1>(kIndexErrorVel, 0);
  // c. orientation:
  Eigen::Matrix3d C_nn =
      Sophus::SO3d::exp(X_.block<3, 1>(kIndexErrorOri, 0)).matrix();
  pose_double.block<3, 3>(0, 0) = C_nn * pose_double.block<3, 3>(0, 0);

  // finally:
  pose_double = init_pose_.inverse() * pose_double;
  vel_double = init_pose_.block<3, 3>(0, 0).transpose() * vel_double;

  pose = pose_double.cast<float>();
  vel = vel_double.cast<float>();
}

/**
 * @brief  get covariance estimation
 * @param  cov, covariance output
 * @return void
 */
void ErrorStateKalmanFilter::GetCovariance(Cov &cov) {
  static int OFFSET_X = 0;
  static int OFFSET_Y = 1;
  static int OFFSET_Z = 2;

  // a. delta position:
  cov.pos.x = P_(kIndexErrorPos + OFFSET_X, kIndexErrorPos + OFFSET_X);
  cov.pos.y = P_(kIndexErrorPos + OFFSET_Y, kIndexErrorPos + OFFSET_Y);
  cov.pos.z = P_(kIndexErrorPos + OFFSET_Z, kIndexErrorPos + OFFSET_Z);

  // b. delta velocity:
  cov.vel.x = P_(kIndexErrorVel + OFFSET_X, kIndexErrorVel + OFFSET_X);
  cov.vel.y = P_(kIndexErrorVel + OFFSET_Y, kIndexErrorVel + OFFSET_Y);
  cov.vel.z = P_(kIndexErrorVel + OFFSET_Z, kIndexErrorVel + OFFSET_Z);

  // c. delta orientation:
  cov.ori.x = P_(kIndexErrorOri + OFFSET_X, kIndexErrorOri + OFFSET_X);
  cov.ori.y = P_(kIndexErrorOri + OFFSET_Y, kIndexErrorOri + OFFSET_Y);
  cov.ori.z = P_(kIndexErrorOri + OFFSET_Z, kIndexErrorOri + OFFSET_Z);

  // d. gyro. bias:
  cov.gyro_bias.x =
      P_(kIndexErrorGyro + OFFSET_X, kIndexErrorGyro + OFFSET_X);
  cov.gyro_bias.y =
      P_(kIndexErrorGyro + OFFSET_Y, kIndexErrorGyro + OFFSET_Y);
  cov.gyro_bias.z =
      P_(kIndexErrorGyro + OFFSET_Z, kIndexErrorGyro + OFFSET_Z);

  // e. accel bias:
  cov.accel_bias.x =
      P_(kIndexErrorAccel + OFFSET_X, kIndexErrorAccel + OFFSET_X);
  cov.accel_bias.y =
      P_(kIndexErrorAccel + OFFSET_Y, kIndexErrorAccel + OFFSET_Y);
  cov.accel_bias.z =
      P_(kIndexErrorAccel + OFFSET_Z, kIndexErrorAccel + OFFSET_Z);
}

/**
 * @brief  get unbiased angular velocity in body frame
 * @param  angular_vel, angular velocity measurement
 * @param  R, corresponding orientation of measurement
 * @return unbiased angular velocity in body frame
 */
inline Eigen::Vector3d ErrorStateKalmanFilter::GetUnbiasedAngularVel(
    const Eigen::Vector3d &angular_vel, const Eigen::Matrix3d &R) {
  return angular_vel - gyro_bias_;
}

/**
 * @brief  get unbiased linear acceleration in navigation frame
 * @param  linear_acc, linear acceleration measurement
 * @param  R, corresponding orientation of measurement
 * @return unbiased linear acceleration in navigation frame
 */
inline Eigen::Vector3d
ErrorStateKalmanFilter::GetUnbiasedLinearAcc(const Eigen::Vector3d &linear_acc,
                                             const Eigen::Matrix3d &R) {
  return R * (linear_acc - accl_bias_) - g_;
}

/**
 * @brief  get angular delta
 * @param  index_curr, current imu measurement buffer index
 * @param  index_prev, previous imu measurement buffer index
 * @param  angular_delta, angular delta output
 * @return true if success false otherwise
 */
bool ErrorStateKalmanFilter::GetAngularDelta(const size_t index_curr,
                                             const size_t index_prev,
                                             Eigen::Vector3d &angular_delta,
                                             Eigen::Vector3d &angular_vel_mid) {
  if (index_curr <= index_prev || imu_data_buff_.size() <= index_curr) {
    return false;
  }

  const IMUData &imu_data_curr = imu_data_buff_.at(index_curr);
  const IMUData &imu_data_prev = imu_data_buff_.at(index_prev);

  double delta_t = imu_data_curr.time - imu_data_prev.time;

  Eigen::Vector3d angular_vel_curr = Eigen::Vector3d(
      imu_data_curr.angular_velocity.x, imu_data_curr.angular_velocity.y,
      imu_data_curr.angular_velocity.z);
  Eigen::Matrix3d R_curr = imu_data_curr.GetOrientationMatrix().cast<double>();
  angular_vel_curr = GetUnbiasedAngularVel(angular_vel_curr, R_curr);

  Eigen::Vector3d angular_vel_prev = Eigen::Vector3d(
      imu_data_prev.angular_velocity.x, imu_data_prev.angular_velocity.y,
      imu_data_prev.angular_velocity.z);
  Eigen::Matrix3d R_prev = imu_data_prev.GetOrientationMatrix().cast<double>();
  angular_vel_prev = GetUnbiasedAngularVel(angular_vel_prev, R_prev);

  angular_delta = 0.5 * delta_t * (angular_vel_curr + angular_vel_prev);

  angular_vel_mid = 0.5 * (angular_vel_curr + angular_vel_prev);
  return true;
}

/**
 * @brief  update orientation with effective rotation angular_delta
 * @param  angular_delta, effective rotation
 * @param  R_curr, current orientation
 * @param  R_prev, previous orientation
 * @return void
 */
void ErrorStateKalmanFilter::UpdateOrientation(
    const Eigen::Vector3d &angular_delta, Eigen::Matrix3d &R_curr,
    Eigen::Matrix3d &R_prev) {
  // magnitude:
  double angular_delta_mag = angular_delta.norm();
  // direction:
  Eigen::Vector3d angular_delta_dir = angular_delta.normalized();

  // build delta q:
  double angular_delta_cos = cos(angular_delta_mag / 2.0);
  double angular_delta_sin = sin(angular_delta_mag / 2.0);
  Eigen::Quaterniond dq(angular_delta_cos,
                        angular_delta_sin * angular_delta_dir.x(),
                        angular_delta_sin * angular_delta_dir.y(),
                        angular_delta_sin * angular_delta_dir.z());
  Eigen::Quaterniond q(pose_.block<3, 3>(0, 0));

  // update:
  q = q * dq;

  // write back:
  R_prev = pose_.block<3, 3>(0, 0);
  pose_.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  R_curr = pose_.block<3, 3>(0, 0);
}

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
bool ErrorStateKalmanFilter::GetVelocityDelta(
    const size_t index_curr, const size_t index_prev,
    const Eigen::Matrix3d &R_curr, const Eigen::Matrix3d &R_prev, double &T,
    Eigen::Vector3d &velocity_delta, Eigen::Vector3d &linear_acc_mid) {
  if (index_curr <= index_prev || imu_data_buff_.size() <= index_curr) {
    return false;
  }

  const IMUData &imu_data_curr = imu_data_buff_.at(index_curr);
  const IMUData &imu_data_prev = imu_data_buff_.at(index_prev);

  T = imu_data_curr.time - imu_data_prev.time;

  Eigen::Vector3d linear_acc_curr = Eigen::Vector3d(
      imu_data_curr.linear_acceleration.x, imu_data_curr.linear_acceleration.y,
      imu_data_curr.linear_acceleration.z);
  linear_acc_curr = GetUnbiasedLinearAcc(linear_acc_curr, R_curr);
  Eigen::Vector3d linear_acc_prev = Eigen::Vector3d(
      imu_data_prev.linear_acceleration.x, imu_data_prev.linear_acceleration.y,
      imu_data_prev.linear_acceleration.z);
  linear_acc_prev = GetUnbiasedLinearAcc(linear_acc_prev, R_prev);

  // mid-value acc can improve error state prediction accuracy:
  linear_acc_mid = 0.5 * (linear_acc_curr + linear_acc_prev);
  velocity_delta = T * linear_acc_mid;

  return true;
}

/**
 * @brief  update orientation with effective velocity change velocity_delta
 * @param  T, timestamp delta
 * @param  velocity_delta, effective velocity change
 * @return void
 */
void ErrorStateKalmanFilter::UpdatePosition(
    const double &T, const Eigen::Vector3d &velocity_delta) {
  pose_.block<3, 1>(0, 3) += T * vel_ + 0.5 * T * velocity_delta;
  vel_ += velocity_delta;
}

/**
 * @brief  update IMU odometry estimation
 * @param  linear_acc_mid, output mid-value unbiased linear acc
 * @return void
 */
void ErrorStateKalmanFilter::UpdateOdomEstimation(
    Eigen::Vector3d &linear_acc_mid, Eigen::Vector3d &angular_vel_mid) {
  //
  // TODO: this is one possible solution to previous chapter, IMU Navigation,
  // assignment
  //
  // get deltas:

  // update orientation:

  // get velocity delta:

  // save mid-value unbiased linear acc for error-state update:

  // update position:
}

/**
 * @brief  set process equation
 * @param  C_nb, rotation matrix, body frame -> navigation frame
 * @param  f_n, accel measurement in navigation frame
 * @return void
 */
void ErrorStateKalmanFilter::SetProcessEquation(const Eigen::Matrix3d &C_nb,
                                                const Eigen::Vector3d &f_n,
                                                const Eigen::Vector3d &w_b) {
  // TODO: set process / system equation:
  // a. set process equation for delta vel:

  // b. set process equation for delta ori:
}

/**
 * @brief  update process equation
 * @param  imu_data, input IMU measurement
 * @param  T, output time delta
 * @return void
 */
void ErrorStateKalmanFilter::UpdateProcessEquation(
    const Eigen::Vector3d &linear_acc_mid,
    const Eigen::Vector3d &angular_vel_mid) {
  // set linearization point:
  Eigen::Matrix3d C_nb = pose_.block<3, 3>(0, 0);
  Eigen::Vector3d f_n = linear_acc_mid + g_;
  Eigen::Vector3d w_b = angular_vel_mid;

  // set process equation:
  SetProcessEquation(C_nb, f_n, w_b);
}

/**
 * @brief  update error estimation
 * @param  linear_acc_mid, input mid-value unbiased linear acc
 * @return void
 */
void ErrorStateKalmanFilter::UpdateErrorEstimation(
    const double &T, const Eigen::Vector3d &linear_acc_mid,
    const Eigen::Vector3d &angular_vel_mid) {
  static MatrixF F_1st;
  static MatrixF F_2nd;
  // TODO: update process equation:

  // TODO: get discretized process equations:

  // TODO: perform Kalman prediction
}

/**
 * @brief  correct error estimation using pose measurement
 * @param  T_nb, input pose measurement
 * @return void
 */
void ErrorStateKalmanFilter::CorrectErrorEstimationPose(
    const Eigen::Matrix4d &T_nb, Eigen::VectorXd &Y, Eigen::MatrixXd &G,
    Eigen::MatrixXd &K) {
  //
  // TODO: set measurement:
  //

  // TODO: set measurement equation:

  // TODO: set Kalman gain:              
}

/**
 * @brief  correct error estimation
 * @param  measurement_type, measurement type
 * @param  measurement, input measurement
 * @return void
 */
void ErrorStateKalmanFilter::CorrectErrorEstimation(
    const MeasurementType &measurement_type, const Measurement &measurement) {
  //
  // TODO: understand ESKF correct workflow
  //
  Eigen::VectorXd Y;
  Eigen::MatrixXd G, K;
  switch (measurement_type) {
  case MeasurementType::POSE:
    CorrectErrorEstimationPose(measurement.T_nb, Y, G, K);
    break;
  default:
    break;
  }

  // TODO: perform Kalman correct:
}

/**
 * @brief  eliminate error
 * @param  void
 * @return void
 */
void ErrorStateKalmanFilter::EliminateError(void) {
  //
  // TODO: correct state estimation using the state of ESKF
  //
  // a. position:
  // do it!
  // b. velocity:
  // do it!
  // c. orientation:
  // do it!

  // d. gyro bias:
  if (IsCovStable(kIndexErrorGyro)) {
    gyro_bias_ += X_.block<3, 1>(kIndexErrorGyro, 0);
  }

  // e. accel bias:
  if (IsCovStable(kIndexErrorAccel)) {
    accl_bias_ += X_.block<3, 1>(kIndexErrorAccel, 0);
  }
}

/**
 * @brief  is covariance stable
 * @param  INDEX_OFSET, state index offset
 * @param  THRESH, covariance threshold, defaults to 1.0e-5
 * @return void
 */
bool ErrorStateKalmanFilter::IsCovStable(const int INDEX_OFSET,
                                         const double THRESH) {
  for (int i = 0; i < 3; ++i) {
    if (P_(INDEX_OFSET + i, INDEX_OFSET + i) > THRESH) {
      return false;
    }
  }

  return true;
}

/**
 * @brief  reset filter state
 * @param  void
 * @return void
 */
void ErrorStateKalmanFilter::ResetState(void) {
  // reset current state:
  X_ = VectorX::Zero();
}

/**
 * @brief  reset filter covariance
 * @param  void
 * @return void
 */
void ErrorStateKalmanFilter::ResetCovariance(void) {
  P_ = MatrixP::Zero();

  P_.block<3, 3>(kIndexErrorPos, kIndexErrorPos) =
      COV.PRIOR.POSI * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(kIndexErrorVel, kIndexErrorVel) =
      COV.PRIOR.VEL * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(kIndexErrorOri, kIndexErrorOri) =
      COV.PRIOR.ORI * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(kIndexErrorGyro, kIndexErrorGyro) =
      COV.PRIOR.EPSILON * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(kIndexErrorAccel, kIndexErrorAccel) =
      COV.PRIOR.DELTA * Eigen::Matrix3d::Identity();
}

/**
 * @brief  get Q analysis for pose measurement
 * @param  void
 * @return void
 */
void ErrorStateKalmanFilter::GetQPose(Eigen::MatrixXd &Q, Eigen::VectorXd &Y) {
  // build observability matrix for position measurement:
  Y = Eigen::VectorXd::Zero(kDimState * kDimMeasurementPose);
  Y.block<kDimMeasurementPose, 1>(0, 0) = YPose_;
  for (int i = 1; i < kDimState; ++i) {
    QPose_.block<kDimMeasurementPose, kDimState>(i * kDimMeasurementPose, 0) =
        (QPose_.block<kDimMeasurementPose, kDimState>(
             (i - 1) * kDimMeasurementPose, 0) *
         F_);

    Y.block<kDimMeasurementPose, 1>(i * kDimMeasurementPose, 0) = YPose_;
  }

  Q = QPose_;
}

/**
 * @brief  update observability analysis
 * @param  measurement_type, measurement type
 * @return void
 */
void ErrorStateKalmanFilter::UpdateObservabilityAnalysis(
    const double &time, const MeasurementType &measurement_type) {
  // get Q:
  Eigen::MatrixXd Q;
  Eigen::VectorXd Y;
  switch (measurement_type) {
  case MeasurementType::POSE:
    GetQPose(Q, Y);
    break;
  default:
    break;
  }

  observability.time_.push_back(time);
  observability.Q_.push_back(Q);
  observability.Y_.push_back(Y);
}

/**
 * @brief  save observability analysis to persistent storage
 * @param  measurement_type, measurement type
 * @return void
 */
bool ErrorStateKalmanFilter::SaveObservabilityAnalysis(
    const MeasurementType &measurement_type) {
  // get fusion strategy:
  std::string type;
  switch (measurement_type) {
  case MeasurementType::POSE:
    type = std::string("pose");
    break;
  case MeasurementType::POSE_VEL:
    type = std::string("pose_velocity");
    break;
  case MeasurementType::POSI:
    type = std::string("position");
    break;
  case MeasurementType::POSI_VEL:
    type = std::string("position_velocity");
    break;
  default:
    return false;
    break;
  }

  // build Q_so:
  const int N = observability.Q_.at(0).rows();

  std::vector<std::vector<double>> q_data, q_so_data;

  Eigen::MatrixXd Qso(observability.Q_.size() * N, kDimState);
  Eigen::VectorXd Yso(observability.Y_.size() * N);

  for (size_t i = 0; i < observability.Q_.size(); ++i) {
    const double &time = observability.time_.at(i);

    const Eigen::MatrixXd &Q = observability.Q_.at(i);
    const Eigen::VectorXd &Y = observability.Y_.at(i);

    Qso.block(i * N, 0, N, kDimState) = Q;
    Yso.block(i * N, 0, N, 1) = Y;

    KalmanFilter::AnalyzeQ(kDimState, time, Q, Y, q_data);

    if (0 < i && (0 == i % 10)) {
      KalmanFilter::AnalyzeQ(kDimState, observability.time_.at(i - 5),
                             Qso.block((i - 10), 0, 10 * N, kDimState),
                             Yso.block((i - 10), 0, 10 * N, 1), q_so_data);
    }
  }

  std::string q_data_csv =
      WORK_SPACE_PATH + "/slam_data/observability/" + type + ".csv";
  std::string q_so_data_csv =
      WORK_SPACE_PATH + "/slam_data/observability/" + type + "_som.csv";

  KalmanFilter::WriteAsCSV(kDimState, q_data, q_data_csv);
  KalmanFilter::WriteAsCSV(kDimState, q_so_data, q_so_data_csv);

  return true;
}

} // namespace lidar_localization