/*
 * @Description: Extended Kalman Filter for IMU-Lidar-GNSS-Odo-Mag fusion
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */
#include <limits>

#include <cmath>
#include <iostream>
#include <fstream>
#include <ostream>

// use sophus to handle so3 hat & SO3 log operations:
#include <sophus/so3.hpp>

// SVD for observability analysis:
#include <Eigen/SVD>

// mag table lookup:
#include "lidar_localization/models/mag_table/geo_mag_declination.hpp"

#include "lidar_localization/models/kalman_filter/extended_kalman_filter.hpp"

#include "lidar_localization/global_defination/global_defination.h"

#include "lidar_localization/tools/CSVWriter.hpp"

#include "glog/logging.h"

namespace lidar_localization {

ExtendedKalmanFilter::ExtendedKalmanFilter(const YAML::Node& node) {
    //
    // parse config:
    // 
    // a. earth constants:
    EARTH.GRAVITY_MAGNITUDE = node["earth"]["gravity_magnitude"].as<double>();
    EARTH.ROTATION_SPEED = node["earth"]["rotation_speed"].as<double>();
   
    EARTH.LATITUDE = node["earth"]["latitude"].as<double>();
    EARTH.LONGITUDE = node["earth"]["longitude"].as<double>();
    
    /*
    // TODO: fix missing dependency mathlib
    double dec = get_mag_declination(
        static_cast<float>(EARTH.LATITUDE), static_cast<float>(EARTH.LONGITUDE)
    );
    double inc = get_mag_inclination(
        static_cast<float>(EARTH.LATITUDE), static_cast<float>(EARTH.LONGITUDE)
    );
    double mag = get_mag_strength(
        static_cast<float>(EARTH.LATITUDE), static_cast<float>(EARTH.LONGITUDE)
    );

    EARTH.MAG.B_U = -mag*sin(inc/180.0*M_PI);
    EARTH.MAG.B_N = +mag*cos(inc/180.0*M_PI)*cos(dec/180.0*M_PI);
    EARTH.MAG.B_E = +mag*cos(inc/180.0*M_PI)*sin(dec/180.0*M_PI);
    */

    EARTH.MAG.B_E = node["earth"]["magneto"]["B_E"].as<double>();
    EARTH.MAG.B_N = node["earth"]["magneto"]["B_N"].as<double>();
    EARTH.MAG.B_U = node["earth"]["magneto"]["B_U"].as<double>();

    EARTH.LATITUDE *= M_PI / 180.0;

    // b. prior state covariance:
    COV.PRIOR.POSI = node["covariance"]["prior"]["pos"].as<double>();
    COV.PRIOR.VEL = node["covariance"]["prior"]["vel"].as<double>();
    // TODO: find a better way for quaternion orientation prior covariance assignment
    COV.PRIOR.ORI = node["covariance"]["prior"]["ori"].as<double>();
    COV.PRIOR.EPSILON = node["covariance"]["prior"]["epsilon"].as<double>();
    COV.PRIOR.DELTA = node["covariance"]["prior"]["delta"].as<double>();
    // c. process noise:
    COV.PROCESS.GYRO = node["covariance"]["process"]["gyro"].as<double>();
    COV.PROCESS.ACCEL = node["covariance"]["process"]["accel"].as<double>();
    // d. measurement noise:
    COV.MEASUREMENT.POSE.POSI = node["covariance"]["measurement"]["pose"]["pos"].as<double>();
    COV.MEASUREMENT.POSE.ORI = node["covariance"]["measurement"]["pose"]["ori"].as<double>();
    COV.MEASUREMENT.POSI = node["covariance"]["measurement"]["pos"].as<double>();
    COV.MEASUREMENT.VEL = node["covariance"]["measurement"]["vel"].as<double>();
    COV.MEASUREMENT.MAG = node["covariance"]["measurement"]["mag"].as<double>();
    // e. motion constraint:
    MOTION_CONSTRAINT.ACTIVATED = node["motion_constraint"]["activated"].as<bool>();
    MOTION_CONSTRAINT.W_B_THRESH = node["motion_constraint"]["w_b_thresh"].as<double>();

    // prompt:
    LOG(INFO) << std::endl 
              << "Iterative Extended Kalman Filter params:" << std::endl
              << "\tgravity magnitude: " << EARTH.GRAVITY_MAGNITUDE << std::endl
              << "\tearth rotation speed: " << EARTH.ROTATION_SPEED << std::endl
              << "\tlatitude: " << EARTH.LATITUDE << std::endl
              << "\tlongitude: " << EARTH.LATITUDE << std::endl
              << "\tmagneto: " << std::endl
              << "\t\tB_E: " << EARTH.MAG.B_E << std::endl
              << "\t\tB_N: " << EARTH.MAG.B_N << std::endl
              << "\t\tB_U: " << EARTH.MAG.B_U << std::endl
              << std::endl
              << "\tprior cov. pos.: " << COV.PRIOR.POSI  << std::endl
              << "\tprior cov. vel.: " << COV.PRIOR.VEL << std::endl
              << "\tprior cov. ori: " << COV.PRIOR.ORI << std::endl
              << "\tprior cov. epsilon.: " << COV.PRIOR.EPSILON  << std::endl
              << "\tprior cov. delta.: " << COV.PRIOR.DELTA << std::endl
              << std::endl
              << "\tprocess noise gyro.: " << COV.PROCESS.GYRO << std::endl
              << "\tprocess noise accel.: " << COV.PROCESS.ACCEL << std::endl
              << std::endl
              << "\tmeasurement noise pose.: " << std::endl 
              << "\t\tpos: " << COV.MEASUREMENT.POSE.POSI << ", ori.: " << COV.MEASUREMENT.POSE.ORI << std::endl
              << "\tmeasurement noise pos.: " << COV.MEASUREMENT.POSI << std::endl
              << "\tmeasurement noise vel.: " << COV.MEASUREMENT.VEL << std::endl
              << "\tmeasurement noise mag.: " << COV.MEASUREMENT.MAG << std::endl
              << std::endl
              << "\tmotion constraint: " << std::endl 
              << "\t\tactivated: " << (MOTION_CONSTRAINT.ACTIVATED ? "true" : "false") << std::endl
              << "\t\tw_b threshold: " << MOTION_CONSTRAINT.W_B_THRESH << std::endl
              << std::endl;
    
    //
    // init filter:
    //
    // a. earth constants:
    g_ = Eigen::Vector3d(
        0.0, 
        0.0, 
        EARTH.GRAVITY_MAGNITUDE
    );
    w_ = Eigen::Vector3d(
        0.0,
        EARTH.ROTATION_SPEED*cos(EARTH.LATITUDE),
        EARTH.ROTATION_SPEED*sin(EARTH.LATITUDE)
    );
    b_ = Eigen::Vector3d(
        EARTH.MAG.B_E,
        EARTH.MAG.B_N,
        EARTH.MAG.B_U
    );
    
    // b. prior state & covariance:
    ResetState();
    ResetCovariance();

    // c. process noise:
    Q_.block<3, 3>(0, 0) = COV.PROCESS.GYRO*Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(3, 3) = COV.PROCESS.ACCEL*Eigen::Matrix3d::Identity();

    // d. measurement noise:
    RPose_.block<3, 3>(0, 0) = COV.MEASUREMENT.POSE.POSI*Eigen::Matrix3d::Identity();
    RPose_.block<4, 4>(3, 3) = COV.MEASUREMENT.POSE.ORI*Eigen::Matrix4d::Identity();

    RPoseVel_.block<3, 3>(0, 0) = COV.MEASUREMENT.POSE.POSI*Eigen::Matrix3d::Identity();
    RPoseVel_.block<4, 4>(3, 3) = COV.MEASUREMENT.POSE.ORI*Eigen::Matrix4d::Identity();
    RPoseVel_.block<3, 3>(7, 7) = COV.MEASUREMENT.VEL*Eigen::Matrix3d::Identity();

    RPosi_.block<3, 3>(0, 0) = COV.MEASUREMENT.POSI*Eigen::Matrix3d::Identity();

    RPosiVel_.block<3, 3>(0, 0) = COV.MEASUREMENT.POSI*Eigen::Matrix3d::Identity();
    RPosiVel_.block<3, 3>(3, 3) = COV.MEASUREMENT.VEL*Eigen::Matrix3d::Identity();

    RPosiMag_.block<3, 3>(0, 0) = COV.MEASUREMENT.POSI*Eigen::Matrix3d::Identity();
    RPosiMag_.block<3, 3>(3, 3) = COV.MEASUREMENT.MAG*Eigen::Matrix3d::Identity();

    RPosiVelMag_.block<3, 3>(0, 0) = COV.MEASUREMENT.POSI*Eigen::Matrix3d::Identity();
    RPosiVelMag_.block<3, 3>(3, 3) = COV.MEASUREMENT.VEL*Eigen::Matrix3d::Identity();
    RPosiVelMag_.block<3, 3>(6, 6) = COV.MEASUREMENT.MAG*Eigen::Matrix3d::Identity();

    // e. process equation:
    F_.block<3, 3>( INDEX_POS, INDEX_VEL ) = Eigen::Matrix3d::Identity();

    // f. measurement equation:
    GPose_.block<3, 3>( 0, INDEX_POS ) = Eigen::Matrix3d::Identity();
    GPose_.block<4, 4>( 3, INDEX_ORI ) = Eigen::Matrix4d::Identity();

    GPoseVel_.block<3, 3>( 0, INDEX_POS ) = Eigen::Matrix3d::Identity();
    GPoseVel_.block<4, 4>( 3, INDEX_ORI ) = Eigen::Matrix4d::Identity();

    GPosi_.block<3, 3>( 0, INDEX_POS ) = Eigen::Matrix3d::Identity();

    GPosiVel_.block<3, 3>( 0, INDEX_POS ) = Eigen::Matrix3d::Identity();

    GPosiMag_.block<3, 3>( 0, INDEX_POS ) = Eigen::Matrix3d::Identity();

    GPosiVelMag_.block<3, 3>( 0, INDEX_POS ) = Eigen::Matrix3d::Identity();

    // init soms:
    QPose_.block<DIM_MEASUREMENT_POSE, DIM_STATE>(0, 0) = GPose_;
    QPoseVel_.block<DIM_MEASUREMENT_POSE_VEL, DIM_STATE>(0, 0) = GPoseVel_;
    QPosi_.block<DIM_MEASUREMENT_POSI, DIM_STATE>(0, 0) = GPosi_;
    QPosiVel_.block<DIM_MEASUREMENT_POSI_VEL, DIM_STATE>(0, 0) = GPosiVel_;
    QPosiMag_.block<DIM_MEASUREMENT_POSI_MAG, DIM_STATE>(0, 0) = GPosiMag_;
    QPosVelMag_.block<DIM_MEASUREMENT_POSI_VEL_MAG, DIM_STATE>(0, 0) = GPosiVelMag_;
}

/**
 * @brief  init filter
 * @param  pose, init pose
 * @param  vel, init vel
 * @param  imu_data, init IMU measurements
 * @return true if success false otherwise
 */
void ExtendedKalmanFilter::Init(
    const Eigen::Vector3d &vel,
    const IMUData &imu_data
) {
    // get init C_nb from IMU estimation:
    Eigen::Matrix3d C_nb = imu_data.GetOrientationMatrix().cast<double>();
    Eigen::Vector4d q_nb(
        imu_data.orientation.w, 
        imu_data.orientation.x, 
        imu_data.orientation.y, 
        imu_data.orientation.z
    );
    // get init v_n from v_b:
    Eigen::Vector3d v_n = C_nb*vel;

    // set init pose:
    init_pose_.block<3, 3>(0, 0) = C_nb;

    // set init velocity:
    init_vel_ = v_n;

    // set init state:
    X_.block<3, 1>(INDEX_POS, 0) = init_pose_.block<3, 1>(0, 3);
    X_.block<3, 1>(INDEX_VEL, 0) = v_n;
    X_.block<4, 1>(INDEX_ORI, 0) = q_nb;
    X_.block<3, 1>(INDEX_GYRO_BIAS, 0) = gyro_bias_;
    X_.block<3, 1>(INDEX_ACCEL_BIAS, 0) = accel_bias_;

    // init IMU data buffer:
    imu_data_buff_.clear();
    imu_data_buff_.push_back(imu_data);

    // init filter time:
    time_ = imu_data.time;

    LOG(INFO) << std::endl 
              << "IEKF Inited at " << static_cast<int>(time_) << std::endl
              << "Init Position: " 
              << init_pose_(0, 3) << ", "
              << init_pose_(1, 3) << ", "
              << init_pose_(2, 3) << std::endl
              << "Init Velocity: "
              << init_vel_.x() << ", "
              << init_vel_.y() << ", "
              << init_vel_.z() << std::endl;
}

/**
 * @brief  Kalman update
 * @param  imu_data, input IMU measurements
 * @return true if success false otherwise
 */
bool ExtendedKalmanFilter::Update(const IMUData &imu_data) {
    // update IMU buff:
    if (time_ < imu_data.time) {
        // Kalman prediction for covariance:
        UpdateCovarianceEstimation(imu_data);

        // Kalman prediction for state:
        imu_data_buff_.push_back(imu_data);
        UpdateStateEstimation();
        imu_data_buff_.pop_front();
        
        // move forward:
        time_ = imu_data.time;

        return true;
    }

    return false;
}

/**
 * @brief  Kalman correction, pose measurement and other measurement in body frame
 * @param  measurement_type, input measurement type
 * @param  measurement, input measurement
 * @return void                                   
 */
bool ExtendedKalmanFilter::Correct(
    const IMUData &imu_data, 
    const MeasurementType &measurement_type, const Measurement &measurement
) { 
    static Measurement measurement_;

    // get time delta:
    double time_delta = measurement.time - time_;

    if ( time_delta > -0.05 ) {
        // perform Kalman prediction:
        if ( time_ < measurement.time ) {
            Update(imu_data);
        }

        // get observation in navigation frame:
        measurement_ = measurement;
        measurement_.T_nb = init_pose_ * measurement_.T_nb;

        // correct error estimation:
        CorrectStateEstimation(measurement_type, measurement_);

        return true;
    }

    LOG(INFO) << "IEKF Correct: Observation is not synced with filter. Skip, " 
              << (int)measurement.time << " <-- " << (int)time_ << " @ " << time_delta
              << std::endl; 
    
    return false;
}

/**
 * @brief  get odometry estimation
 * @param  pose, init pose
 * @param  vel, init vel
 * @return void
 */
void ExtendedKalmanFilter::GetOdometry(
    Eigen::Matrix4f &pose, Eigen::Vector3f &vel
) {
    // init:
    Eigen::Matrix4d pose_double = Eigen::Matrix4d::Identity();
    Eigen::Vector3d vel_double = Eigen::Vector3d::Zero();

    // eliminate error:
    // a. position:
    pose_double.block<3, 1>(0, 3) = X_.block<3, 1>(INDEX_POS, 0);
    // b. velocity:
    vel_double = X_.block<3, 1>(INDEX_VEL, 0);
    // c. orientation:
    Eigen::Quaterniond q_nb(
        X_(INDEX_ORI + 0, 0),
        X_(INDEX_ORI + 1, 0),
        X_(INDEX_ORI + 2, 0),
        X_(INDEX_ORI + 3, 0)
    );
    pose_double.block<3, 3>(0, 0) = q_nb.toRotationMatrix();

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
void ExtendedKalmanFilter::GetCovariance(Cov &cov) {
    static int OFFSET_X = 0;
    static int OFFSET_Y = 1;
    static int OFFSET_Z = 2;

    // a. delta position:
    cov.pos.x = P_(INDEX_POS + OFFSET_X, INDEX_POS + OFFSET_X);
    cov.pos.y = P_(INDEX_POS + OFFSET_Y, INDEX_POS + OFFSET_Y);
    cov.pos.z = P_(INDEX_POS + OFFSET_Z, INDEX_POS + OFFSET_Z);

    // b. delta velocity:
    cov.vel.x = P_(INDEX_VEL + OFFSET_X, INDEX_VEL + OFFSET_X);
    cov.vel.y = P_(INDEX_VEL + OFFSET_Y, INDEX_VEL + OFFSET_Y);
    cov.vel.z = P_(INDEX_VEL + OFFSET_Z, INDEX_VEL + OFFSET_Z);

    // c. delta orientation:
    cov.ori.w = P_(INDEX_ORI + 0, INDEX_ORI + 0);
    cov.ori.x = P_(INDEX_ORI + 1, INDEX_ORI + 1);
    cov.ori.y = P_(INDEX_ORI + 2, INDEX_ORI + 2);
    cov.ori.z = P_(INDEX_ORI + 3, INDEX_ORI + 3);

    // d. gyro. bias:
    cov.gyro_bias.x = P_(INDEX_GYRO_BIAS + OFFSET_X, INDEX_GYRO_BIAS + OFFSET_X);
    cov.gyro_bias.y = P_(INDEX_GYRO_BIAS + OFFSET_Y, INDEX_GYRO_BIAS + OFFSET_Y);
    cov.gyro_bias.z = P_(INDEX_GYRO_BIAS + OFFSET_Z, INDEX_GYRO_BIAS + OFFSET_Z);

    // e. accel bias:
    cov.accel_bias.x = P_(INDEX_ACCEL_BIAS + OFFSET_X, INDEX_ACCEL_BIAS + OFFSET_X);
    cov.accel_bias.y = P_(INDEX_ACCEL_BIAS + OFFSET_Y, INDEX_ACCEL_BIAS + OFFSET_Y);
    cov.accel_bias.z = P_(INDEX_ACCEL_BIAS + OFFSET_Z, INDEX_ACCEL_BIAS + OFFSET_Z);
}

/**
 * @brief  get unbiased angular velocity in body frame
 * @param  angular_vel, angular velocity measurement
 * @param  C_nb, corresponding orientation of measurement
 * @return unbiased angular velocity in body frame
 */
inline Eigen::Vector3d ExtendedKalmanFilter::GetUnbiasedAngularVel(
    const Eigen::Vector3d &angular_vel,
    const Eigen::Matrix3d &C_nb
) {
    return angular_vel - gyro_bias_;
}

/**
 * @brief  get unbiased linear acceleration in navigation frame
 * @param  linear_acc, linear acceleration measurement
 * @param  C_nb, corresponding orientation of measurement
 * @return unbiased linear acceleration in navigation frame
 */
inline Eigen::Vector3d ExtendedKalmanFilter::GetUnbiasedLinearAcc(
    const Eigen::Vector3d &linear_acc,
    const Eigen::Matrix3d &C_nb
) {
    return C_nb*(linear_acc - accel_bias_) - g_;
}

/**
 * @brief  remove gravity component from accel measurement
 * @param  f_b, accel measurement measurement
 * @param  C_nb, orientation matrix
 * @return f_b
 */
inline Eigen::Vector3d ExtendedKalmanFilter::RemoveGravity (
    const Eigen::Vector3d &f_b,
    const Eigen::Matrix3d &C_nb
) {
    return f_b - C_nb.transpose()*g_;
}

/**
 * @brief  apply motion constraint on velocity estimation
 * @param  void
 * @return void
 */
void ExtendedKalmanFilter::ApplyMotionConstraint(void) {
    const Eigen::Quaterniond q_nb(
        X_( INDEX_ORI + 0, 0),
        X_( INDEX_ORI + 1, 0),
        X_( INDEX_ORI + 2, 0),
        X_( INDEX_ORI + 3, 0)
    );
    const Eigen::Matrix3d C_nb = q_nb.toRotationMatrix();

    Eigen::Vector3d v_b = C_nb.transpose() * X_.block<3, 1>( INDEX_VEL, 0 );
    v_b.y() = 0.0;
    X_.block<3, 1>( INDEX_VEL, 0 ) = C_nb * v_b;
}

/**
 * @brief  get angular delta
 * @param  index_curr, current imu measurement buffer index
 * @param  index_prev, previous imu measurement buffer index
 * @param  angular_delta, angular delta output
 * @return true if success false otherwise
 */
bool ExtendedKalmanFilter::GetAngularDelta(
    const size_t index_curr, const size_t index_prev,
    Eigen::Vector3d &angular_delta
) {
    if (
        index_curr <= index_prev ||
        imu_data_buff_.size() <= index_curr
    ) {
        return false;
    }

    const IMUData &imu_data_curr = imu_data_buff_.at(index_curr);
    const IMUData &imu_data_prev = imu_data_buff_.at(index_prev);

    double delta_t = imu_data_curr.time - imu_data_prev.time;

    Eigen::Vector3d angular_vel_curr = Eigen::Vector3d(
        imu_data_curr.angular_velocity.x,
        imu_data_curr.angular_velocity.y,
        imu_data_curr.angular_velocity.z
    );
    Eigen::Matrix3d R_curr = imu_data_curr.GetOrientationMatrix().cast<double>();
    angular_vel_curr = GetUnbiasedAngularVel(angular_vel_curr, R_curr);

    Eigen::Vector3d angular_vel_prev = Eigen::Vector3d(
        imu_data_prev.angular_velocity.x,
        imu_data_prev.angular_velocity.y,
        imu_data_prev.angular_velocity.z
    );
    Eigen::Matrix3d R_prev = imu_data_prev.GetOrientationMatrix().cast<double>();
    angular_vel_prev = GetUnbiasedAngularVel(angular_vel_prev, R_prev);

    angular_delta = 0.5*delta_t*(angular_vel_curr + angular_vel_prev);

    return true;
}

/**
 * @brief  update orientation with effective rotation angular_delta
 * @param  angular_delta, effective rotation
 * @param  R_curr, current orientation
 * @param  R_prev, previous orientation
 * @return void
 */
void ExtendedKalmanFilter::UpdateOrientation(
    const Eigen::Vector3d &angular_delta,
    Eigen::Matrix3d &R_curr, Eigen::Matrix3d &R_prev
) {
    // magnitude:
    double angular_delta_mag = angular_delta.norm();
    // direction:
    Eigen::Vector3d angular_delta_dir = angular_delta.normalized();

    // build delta q:
    double angular_delta_cos = cos(angular_delta_mag/2.0);
    double angular_delta_sin = sin(angular_delta_mag/2.0);

    Eigen::Quaterniond q(
        X_(INDEX_ORI + 0, 0),
        X_(INDEX_ORI + 1, 0),
        X_(INDEX_ORI + 2, 0),
        X_(INDEX_ORI + 3, 0)
    );
    Eigen::Quaterniond dq(
        angular_delta_cos, 
        angular_delta_sin*angular_delta_dir.x(), 
        angular_delta_sin*angular_delta_dir.y(), 
        angular_delta_sin*angular_delta_dir.z()
    );

    // update:
    R_prev = q.toRotationMatrix();

    q = q*dq;
    q.normalize();

    R_curr = q.toRotationMatrix();

    // write back:
    X_(INDEX_ORI + 0, 0) = q.w();
    X_(INDEX_ORI + 1, 0) = q.x();
    X_(INDEX_ORI + 2, 0) = q.y();
    X_(INDEX_ORI + 3, 0) = q.z();
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
bool ExtendedKalmanFilter::GetVelocityDelta(
    const size_t index_curr, const size_t index_prev,
    const Eigen::Matrix3d &R_curr, const Eigen::Matrix3d &R_prev, 
    double &T, 
    Eigen::Vector3d &velocity_delta
) {
    if (
        index_curr <= index_prev ||
        imu_data_buff_.size() <= index_curr
    ) {
        return false;
    }

    const IMUData &imu_data_curr = imu_data_buff_.at(index_curr);
    const IMUData &imu_data_prev = imu_data_buff_.at(index_prev);

    T = imu_data_curr.time - imu_data_prev.time;

    Eigen::Vector3d linear_acc_curr = Eigen::Vector3d(
        imu_data_curr.linear_acceleration.x,
        imu_data_curr.linear_acceleration.y,
        imu_data_curr.linear_acceleration.z
    );
    linear_acc_curr = GetUnbiasedLinearAcc(linear_acc_curr, R_curr);
    Eigen::Vector3d linear_acc_prev = Eigen::Vector3d(
        imu_data_prev.linear_acceleration.x,
        imu_data_prev.linear_acceleration.y,
        imu_data_prev.linear_acceleration.z
    );
    linear_acc_prev = GetUnbiasedLinearAcc(linear_acc_prev, R_prev);
    
    // mid-value acc can improve error state prediction accuracy:
    velocity_delta = 0.5*T*(linear_acc_curr + linear_acc_prev);

    return true;
}

/**
 * @brief  update orientation with effective velocity change velocity_delta
 * @param  T, timestamp delta 
 * @param  velocity_delta, effective velocity change
 * @return void
 */
void ExtendedKalmanFilter::UpdatePosition(const double &T, const Eigen::Vector3d &velocity_delta) {
    X_.block<3, 1>(INDEX_POS, 0) += T*X_.block<3, 1>(INDEX_VEL, 0) + 0.5*T*velocity_delta;
    X_.block<3, 1>(INDEX_VEL, 0) += velocity_delta;
}

/**
 * @brief  update state estimation
 * @param  void
 * @return void
 */
void ExtendedKalmanFilter::UpdateStateEstimation(void) {
    // get deltas:
    Eigen::Vector3d angular_delta; 
    GetAngularDelta(1, 0, angular_delta);

    // update orientation:
    Eigen::Matrix3d R_curr, R_prev;
    UpdateOrientation(angular_delta, R_curr, R_prev);

    // get velocity delta:
    double T;
    Eigen::Vector3d velocity_delta;
    // save mid-value unbiased linear acc for error-state update:
    GetVelocityDelta(1, 0, R_curr, R_prev, T, velocity_delta);

    // update position:
    UpdatePosition(T, velocity_delta);

    // apply motion constraint:
    ApplyMotionConstraint();
}

/**
 * @brief  get block matrix for velocity update by orientation quaternion
 * @param  f_b, accel measurement
 * @param  q_nb, orientation quaternion
 * @return block matrix Fvq
 */
Eigen::Matrix<double, 3, 4> ExtendedKalmanFilter::GetFVelOri(
    const Eigen::Vector3d &f_b,
    const Eigen::Quaterniond &q_nb
) {
    // get F:
    Eigen::Matrix<double, 4, 3> T_Ff;
    T_Ff << +q_nb.w(), -q_nb.z(), +q_nb.y(),
            +q_nb.x(), +q_nb.y(), +q_nb.z(),
            -q_nb.y(), +q_nb.x(), +q_nb.w(),
            -q_nb.z(), -q_nb.w(), +q_nb.x();
    Eigen::Vector4d F = 2 * T_Ff * f_b;

    // get Fvq:
    Eigen::Matrix<double, 3, 4> Fvq;
    Fvq << +F(0), +F(1), +F(2), +F(3),
           -F(3), -F(2), +F(1), +F(0),
           +F(2), -F(3), -F(0), +F(1);

    return Fvq;
}

/**
 * @brief  get block matrix for orientation quaternion update by orientation quaternion
 * @param  w_b, gyro measurement
 * @return block matrix Fqq
 */
Eigen::Matrix<double, 4, 4> ExtendedKalmanFilter::GetFOriOri(
    const Eigen::Vector3d &w_b
) {
    // get Fqq:
    Eigen::Matrix<double, 4, 4> Fqq;
    Fqq <<      0.0, -w_b.x(), -w_b.y(), -w_b.z(),
           +w_b.x(),      0.0, +w_b.z(), -w_b.y(),
           +w_b.y(), -w_b.z(),      0.0, +w_b.x(),
           +w_b.z(), +w_b.y(), -w_b.x(),      0.0;

    return 0.5 * Fqq;
}

/**
 * @brief  get block matrix for orientation quaternion update by epsilon, angular velocity bias
 * @param  q_nb, orientation quaternion
 * @return block matrix Fqe
 */
Eigen::Matrix<double, 4, 3> ExtendedKalmanFilter::GetFOriEps(
    const Eigen::Quaterniond &q_nb
) {
    // get Fqe:
    Eigen::Matrix<double, 4, 3> Fqe;
    Fqe << -q_nb.x(), -q_nb.y(), -q_nb.z(),
           +q_nb.w(), -q_nb.z(), +q_nb.y(),
           +q_nb.z(), +q_nb.w(), -q_nb.x(),
           -q_nb.y(), +q_nb.x(), +q_nb.w();

    return 0.5 * Fqe;
}

/**
 * @brief  set process equation
 * @param  void
 * @return void
 */
void ExtendedKalmanFilter::SetProcessEquation(const IMUData &imu_data) {
    // parse IMU measurement:
    const Eigen::Vector3d f_b(
        imu_data.linear_acceleration.x,
        imu_data.linear_acceleration.y,
        imu_data.linear_acceleration.z
    );
    const Eigen::Vector3d w_b(
        imu_data.angular_velocity.x,
        imu_data.angular_velocity.y,
        imu_data.angular_velocity.z
    );

    // parse orientation:
    const Eigen::Quaterniond q_nb(
        X_(INDEX_ORI + 0, 0),
        X_(INDEX_ORI + 1, 0),
        X_(INDEX_ORI + 2, 0),
        X_(INDEX_ORI + 3, 0)
    );
    const Eigen::Matrix3d C_nb = q_nb.toRotationMatrix();

    //
    // EKF is linearized around VectorX::Zero
    //
    // a. set equation for velocity:
    F_.block<3, 4>(INDEX_VEL,         INDEX_ORI) = GetFVelOri(f_b, q_nb);
    F_.block<3, 3>(INDEX_VEL,  INDEX_ACCEL_BIAS) = B_.block<3, 3>(INDEX_VEL, 3) = C_nb;
    
    // b. set equation for orientation quaternion:
    F_.block<4, 4>(INDEX_ORI,         INDEX_ORI) = GetFOriOri(w_b);
    F_.block<4, 3>(INDEX_ORI,   INDEX_GYRO_BIAS) = B_.block<4, 3>(INDEX_ORI, 0) = GetFOriEps(q_nb);
}

/**
 * @brief  update covariance estimation
 * @param  void
 * @return void
 */
void ExtendedKalmanFilter::UpdateCovarianceEstimation(
    const IMUData &imu_data
) {
    // update process equation:
    SetProcessEquation(imu_data);

    // get discretized process equations:
    double T = imu_data.time - time_;

    // approximate to 1st order:
    MatrixF F = MatrixF::Identity() + T*F_;

    MatrixB B = T*B_;

    // perform Kalman prediction for covariance:
    P_ = F*P_*F.transpose() + B*Q_*B.transpose();
}

/**
 * @brief  get block matrix for observation by orientation quaternion
 * @param  m_n, measurement in navigation frame
 * @param  q_nb, orientation quaternion
 * @return block matrix Gq
 */
Eigen::Matrix<double, 3, 4> ExtendedKalmanFilter::GetGMOri(
    const Eigen::Vector3d &m_n,
    const Eigen::Quaterniond &q_nb
) {
    // get F:
    Eigen::Matrix<double, 4, 3> T_Gm;
    T_Gm << +q_nb.w(), +q_nb.z(), -q_nb.y(),
            +q_nb.x(), +q_nb.y(), +q_nb.z(),
            -q_nb.y(), +q_nb.x(), -q_nb.w(),
            -q_nb.z(), +q_nb.w(), +q_nb.x();
    Eigen::Vector4d G = 2 * T_Gm * m_n;

    // get Fvq:
    Eigen::Matrix<double, 3, 4> Gq;
    Gq << +G(0), +G(1), +G(2), +G(3),
          +G(3), -G(2), +G(1), -G(0),
          -G(2), -G(3), +G(0), +G(1);

    return Gq;
}

/**
 * @brief  correct state estimation using frontend pose
 * @param  T_nb, input frontend pose estimation
 * @return void
 */
void ExtendedKalmanFilter::CorrectStateEstimationPose(
    const Eigen::Matrix4d &T_nb
) {
    // parse measurement:
    YPose_.block<3, 1>(0, 0) = T_nb.block<3, 1>(0,3);
    Eigen::Quaterniond q_nb(T_nb.block<3, 3>(0,0));
    YPose_(3, 0) = q_nb.w();
    YPose_(4, 0) = q_nb.x();
    YPose_(5, 0) = q_nb.y();
    YPose_(6, 0) = q_nb.z();

    // build Kalman gain:
    MatrixRPose R = GPose_*P_*GPose_.transpose() + RPose_;
    MatrixKPose K = P_*GPose_.transpose()*R.inverse();

    // perform Kalman correct:
    P_ = (MatrixP::Identity() - K*GPose_)*P_;
    X_ = X_ + K*(YPose_ - GPose_*X_);

    // normalize quaternion:
    X_.block<4, 1>( INDEX_ORI, 0 ).normalize();
}

/**
 * @brief  correct state estimation using GNSS position
 * @param  T_nb, input GNSS position
 * @return void
 */
void ExtendedKalmanFilter::CorrectStateEstimationPosi(
    const Eigen::Matrix4d &T_nb
) {
    // parse measurement:
    YPosi_.block<3, 1>(0, 0) = T_nb.block<3, 1>(0,3);

    // build Kalman gain:
    MatrixRPosi R = GPosi_*P_*GPosi_.transpose() + RPosi_;
    MatrixKPosi K = P_*GPosi_.transpose()*R.inverse();

    // perform Kalman correct:
    P_ = (MatrixP::Identity() - K*GPosi_)*P_;
    X_ = X_ + K*(YPosi_ - GPosi_*X_);

    // normalize quaternion:
    X_.block<4, 1>( INDEX_ORI, 0 ).normalize();
}

/**
 * @brief  correct state estimation using frontend pose & body velocity measurement
 * @param  T_nb, input frontend pose estimation
 * @param  v_b, input odo
 * @return void
 */
void ExtendedKalmanFilter::CorrectStateEstimationPoseVel(
    const Eigen::Matrix4d &T_nb, 
    const Eigen::Vector3d &v_b
) {
    // parse measurement:
    YPoseVel_.block<3, 1>(0, 0) = T_nb.block<3, 1>(0,3);

    Eigen::Quaterniond q_nb_obs(T_nb.block<3, 3>(0,0));
    YPoseVel_(3, 0) = q_nb_obs.w();
    YPoseVel_(4, 0) = q_nb_obs.x();
    YPoseVel_(5, 0) = q_nb_obs.y();
    YPoseVel_(6, 0) = q_nb_obs.z();

    YPoseVel_.block<3, 1>(7, 0) = v_b;

    // iterative observation:
    for (size_t i = 0; i < 3; ++i) {
        // set observation equation:
        Eigen::Quaterniond q_nb_pred(
            X_(INDEX_ORI + 0, 0),
            X_(INDEX_ORI + 1, 0),
            X_(INDEX_ORI + 2, 0),
            X_(INDEX_ORI + 3, 0)
        );
        Eigen::Matrix3d C_nb_pred = q_nb_pred.toRotationMatrix();
        GPoseVel_.block<3, 3>( 7, INDEX_VEL ) = C_nb_pred.transpose();
        GPoseVel_.block<3, 4>( 7, INDEX_ORI ) = GetGMOri(X_.block<3, 1>(INDEX_VEL, 0), q_nb_pred);

        // build Kalman gain:
        MatrixRPoseVel R = GPoseVel_*P_*GPoseVel_.transpose() + RPoseVel_;
        MatrixKPoseVel K = P_*GPoseVel_.transpose()*R.inverse();
        VectorYPoseVel Y = VectorYPoseVel::Zero();
        Y.block<3, 1>(0, 0) = X_.block<3, 1>(INDEX_POS, 0);
        Y.block<4, 1>(3, 0) = X_.block<4, 1>(INDEX_ORI, 0);
        Y.block<3, 1>(7, 0) = C_nb_pred.transpose() * X_.block<3, 1>(INDEX_VEL, 0);

        // perform Kalman correct:
        P_ = (MatrixP::Identity() - K*GPoseVel_)*P_;
        X_ = X_ + K*(YPoseVel_ - Y);

        // normalize quaternion:
        X_.block<4, 1>( INDEX_ORI, 0 ).normalize();
    }
}

/**
 * @brief  correct state estimation using GNSS position and odometer measurement
 * @param  T_nb, input GNSS position 
 * @param  v_b, input odo
 * @return void
 */
void ExtendedKalmanFilter::CorrectStateEstimationPosiVel(
    const Eigen::Matrix4d &T_nb, const Eigen::Vector3d &v_b
) {
    // parse measurement:
    YPosiVel_.block<3, 1>(0, 0) = T_nb.block<3, 1>(0,3);
    YPosiVel_.block<3, 1>(3, 0) = v_b;

    // iterative observation:
    for (size_t i = 0; i < 1; ++i) {
        // set observation equation:
        Eigen::Quaterniond q_nb(
            X_(INDEX_ORI + 0, 0),
            X_(INDEX_ORI + 1, 0),
            X_(INDEX_ORI + 2, 0),
            X_(INDEX_ORI + 3, 0)
        );
        Eigen::Matrix3d C_nb = q_nb.toRotationMatrix();
        GPosiVel_.block<3, 3>( 3, INDEX_VEL ) = C_nb.transpose();
        GPosiVel_.block<3, 4>( 3, INDEX_ORI ) = GetGMOri(X_.block<3, 1>(INDEX_VEL, 0), q_nb);

        // build Kalman gain:
        MatrixRPosiVel R = GPosiVel_*P_*GPosiVel_.transpose() + RPosiVel_;
        MatrixKPosiVel K = P_*GPosiVel_.transpose()*R.inverse();
        VectorYPosiVel Y = X_.block<6, 1>(0, 0);
        Y.block<3, 1>(3, 0) = C_nb.transpose() * Y.block<3, 1>(3, 0);

        // perform Kalman correct:
        P_ = (MatrixP::Identity() - K*GPosiVel_)*P_;
        X_ = X_ + K*(YPosiVel_ - Y);
        
        // normalize quaternion:
        X_.block<4, 1>( INDEX_ORI, 0 ).normalize();
    }
}

/**
 * @brief  correct state estimation using GNSS position and magneto measurement
 * @param  T_nb, input GNSS position 
 * @param  B_b, input magneto
 * @return void
 */
void ExtendedKalmanFilter::CorrectStateEstimationPosiMag(
    const Eigen::Matrix4d &T_nb, const Eigen::Vector3d &B_b
) {
    // parse measurement:
    YPosiMag_.block<3, 1>(0, 0) = T_nb.block<3, 1>(0,3);
    YPosiMag_.block<3, 1>(3, 0) = B_b;

    // iterative observation:
    for (size_t i = 0; i < 1; ++i) {
        // set observation equation:
        const Eigen::Quaterniond q_nb(
            X_(INDEX_ORI + 0, 0),
            X_(INDEX_ORI + 1, 0),
            X_(INDEX_ORI + 2, 0),
            X_(INDEX_ORI + 3, 0)
        );
        Eigen::Matrix3d C_nb = q_nb.toRotationMatrix();
        GPosiMag_.block<3, 4>( 3, INDEX_ORI ) = GetGMOri(b_, q_nb);

        // build Kalman gain:
        MatrixRPosiMag R = GPosiMag_*P_*GPosiMag_.transpose() + RPosiMag_;
        MatrixKPosiMag K = P_*GPosiMag_.transpose()*R.inverse();
        VectorYPosiMag Y = VectorYPosiMag::Zero();
        Y.block<3, 1>(0, 0) = X_.block<3, 1>( INDEX_POS, 0);
        Y.block<3, 1>(3, 0) = C_nb.transpose() * b_;

        // perform Kalman correct:
        P_ = (MatrixP::Identity() - K*GPosiMag_)*P_;
        X_ = X_ + K*(YPosiMag_ - Y);

        // normalize quaternion:
        X_.block<4, 1>( INDEX_ORI, 0 ).normalize();
    }
}

/**
 * @brief  correct state estimation using GNSS position, odometer and magneto measurement
 * @param  T_nb, input GNSS position 
 * @param  v_b, input odo
 * @param  B_b, input magneto
 * @return void
 */
void ExtendedKalmanFilter::CorrectStateEstimationPosiVelMag(
    const Eigen::Matrix4d &T_nb, const Eigen::Vector3d &v_b, const Eigen::Vector3d &B_b
) {
    // parse measurement:
    YPosiVelMag_.block<3, 1>(0, 0) = T_nb.block<3, 1>(0,3);
    YPosiVelMag_.block<3, 1>(3, 0) = v_b;
    YPosiVelMag_.block<3, 1>(6, 0) = B_b;

    // iterative observation:
    for (size_t i = 0; i < 1; ++i) {
        // set observation equation:
        const Eigen::Quaterniond q_nb(
            X_(INDEX_ORI + 0, 0),
            X_(INDEX_ORI + 1, 0),
            X_(INDEX_ORI + 2, 0),
            X_(INDEX_ORI + 3, 0)
        );
        Eigen::Matrix3d C_nb = q_nb.toRotationMatrix();
        GPosiVelMag_.block<3, 3>( 3, INDEX_VEL ) = C_nb.transpose();
        GPosiVelMag_.block<3, 4>( 3, INDEX_ORI ) = GetGMOri(X_.block<3, 1>(INDEX_VEL, 0), q_nb);
        GPosiVelMag_.block<3, 4>( 6, INDEX_ORI ) = GetGMOri(b_, q_nb);

        // build Kalman gain:
        MatrixRPosiVelMag R = GPosiVelMag_*P_*GPosiVelMag_.transpose() + RPosiVelMag_;
        MatrixKPosiVelMag K = P_*GPosiVelMag_.transpose()*R.inverse();
        VectorYPosiVelMag Y = VectorYPosiVelMag::Zero();
        Y.block<3, 1>(0, 0) = X_.block<3, 1>( INDEX_POS, 0);
        Y.block<3, 1>(3, 0) = C_nb.transpose() * X_.block<3, 1>(INDEX_VEL, 0);
        Y.block<3, 1>(6, 0) = C_nb.transpose() * b_;
        
        // perform Kalman correct:
        P_ = (MatrixP::Identity() - K*GPosiVelMag_)*P_;
        X_ = X_ + K*(YPosiVelMag_ - Y);

        // normalize quaternion:
        X_.block<4, 1>( INDEX_ORI, 0 ).normalize();
    }
}

/**
 * @brief  correct state estimation
 * @param  measurement_type, measurement type
 * @param  measurement, input measurement
 * @return void
 */
void ExtendedKalmanFilter::CorrectStateEstimation(
    const MeasurementType &measurement_type, 
    const Measurement &measurement
) {
    switch ( measurement_type ) {
        case MeasurementType::POSE:
            CorrectStateEstimationPose(measurement.T_nb);
            break;
        case MeasurementType::POSE_VEL:
            CorrectStateEstimationPoseVel(measurement.T_nb, measurement.v_b);
            break;
        case MeasurementType::POSI:
            CorrectStateEstimationPosi(measurement.T_nb);
            break;
        case MeasurementType::POSI_VEL:
            CorrectStateEstimationPosiVel(measurement.T_nb, measurement.v_b);
            break;
        case MeasurementType::POSI_MAG:
            CorrectStateEstimationPosiMag(measurement.T_nb, measurement.B_b);
            break;
        case MeasurementType::POSI_VEL_MAG:
            CorrectStateEstimationPosiVelMag(measurement.T_nb, measurement.v_b, measurement.B_b);
            break;
        default:
            break;
    }

    // update bias:
    if ( IsCovStable(INDEX_GYRO_BIAS, 1.0e-6) )  gyro_bias_ = X_.block<3, 1>(INDEX_GYRO_BIAS, 0);
    if ( IsCovStable(INDEX_ACCEL_BIAS, 1.0e-6) ) accel_bias_ = X_.block<3, 1>(INDEX_ACCEL_BIAS, 0);
}

/**
 * @brief  is covariance stable
 * @param  INDEX_OFFSET, state index offset
 * @param  THRESH, covariance threshold, defaults to 1.0e-5
 * @return void
 */
bool ExtendedKalmanFilter::IsCovStable(
    const int INDEX_OFFSET,
    const double THRESH
) {
    if ( INDEX_ORI == INDEX_OFFSET ) {
        for (int i = 0; i < 4; ++i) {
            if ( P_(INDEX_OFFSET + i, INDEX_OFFSET + i) > THRESH ) {
                return false;
            }
        }
    } else {
        for (int i = 0; i < 3; ++i) {
            if ( P_(INDEX_OFFSET + i, INDEX_OFFSET + i) > THRESH ) {
                return false;
            }
        }
    }

    return true;
}

/**
 * @brief  reset filter state
 * @param  void
 * @return void
 */
void ExtendedKalmanFilter::ResetState(void) {
    // reset current state:
    X_ = VectorX::Zero();
}

/**
 * @brief  reset filter covariance
 * @param  void
 * @return void
 */
void ExtendedKalmanFilter::ResetCovariance(void) {
    P_ = MatrixP::Zero();
    
    P_.block<3, 3>(       INDEX_POS,        INDEX_POS) = COV.PRIOR.POSI*Eigen::Matrix3d::Identity();
    P_.block<3, 3>(       INDEX_VEL,        INDEX_VEL) = COV.PRIOR.VEL*Eigen::Matrix3d::Identity();
    // TODO: find a better way for quaternion orientation prior covariance assignment
    P_.block<4, 4>(       INDEX_ORI,        INDEX_ORI) = COV.PRIOR.ORI*Eigen::Matrix4d::Identity();
    P_.block<3, 3>( INDEX_GYRO_BIAS,  INDEX_GYRO_BIAS) = COV.PRIOR.EPSILON*Eigen::Matrix3d::Identity();
    P_.block<3, 3>(INDEX_ACCEL_BIAS, INDEX_ACCEL_BIAS) = COV.PRIOR.DELTA*Eigen::Matrix3d::Identity();
}

/**
 * @brief  get Q for pose measurement
 * @param  void
 * @return void
 */
void ExtendedKalmanFilter::GetQPose(Eigen::MatrixXd &Q) {
    // build observability matrix for position measurement:
    for (int i = 1; i < DIM_STATE; ++i) {
        QPose_.block<DIM_MEASUREMENT_POSE, DIM_STATE>(i*DIM_MEASUREMENT_POSE, 0) = (
            QPose_.block<DIM_MEASUREMENT_POSE, DIM_STATE>((i - 1)*DIM_MEASUREMENT_POSE, 0) * F_
        );
    }

    Q = QPose_;
}

/**
 * @brief  get Q for pose & body velocity measurement
 * @param  void
 * @return void
 */
void  ExtendedKalmanFilter::GetQPoseVel(Eigen::MatrixXd &Q) {
    // build observability matrix for position & velocity measurement:
    QPoseVel_.block<DIM_MEASUREMENT_POSE_VEL, DIM_STATE>(0, 0) = GPoseVel_;
    for (int i = 1; i < DIM_STATE; ++i) {
        QPoseVel_.block<DIM_MEASUREMENT_POSE_VEL, DIM_STATE>(i*DIM_MEASUREMENT_POSE_VEL, 0) = (
            QPoseVel_.block<DIM_MEASUREMENT_POSE_VEL, DIM_STATE>((i - 1)*DIM_MEASUREMENT_POSE_VEL, 0) * F_
        );
    }

    Q = QPoseVel_;
}

/**
 * @brief  get Q for GNSS position measurement
 * @param  void
 * @return QPosi
 */
void ExtendedKalmanFilter::GetQPosi(Eigen::MatrixXd &Q) {
    // build observability matrix for position measurement:
    for (int i = 1; i < DIM_STATE; ++i) {
        QPosi_.block<DIM_MEASUREMENT_POSI, DIM_STATE>(i*DIM_MEASUREMENT_POSI, 0) = (
            QPosi_.block<DIM_MEASUREMENT_POSI, DIM_STATE>((i - 1)*DIM_MEASUREMENT_POSI, 0) * F_
        );
    }  

    Q = QPosi_;
}

/**
 * @brief  get Q for GNSS position & body velocity measurement
 * @param  void
 * @return QPosiVel
 */
 void ExtendedKalmanFilter::GetQPosiVel(Eigen::MatrixXd &Q) {
    // build observability matrix for position & velocity measurement:
    QPosiVel_.block<DIM_MEASUREMENT_POSI_VEL, DIM_STATE>(0, 0) = GPosiVel_;
    for (int i = 1; i < DIM_STATE; ++i) {
        QPosiVel_.block<DIM_MEASUREMENT_POSI_VEL, DIM_STATE>(i*DIM_MEASUREMENT_POSI_VEL, 0) = (
            QPosiVel_.block<DIM_MEASUREMENT_POSI_VEL, DIM_STATE>((i - 1)*DIM_MEASUREMENT_POSI_VEL, 0) * F_
        );
    }

    Q = QPosiVel_;
}

/**
 * @brief  get Q for GNSS position & magneto measurement
 * @param  void
 * @return QPosiMag
 */
 void ExtendedKalmanFilter::GetQPosiMag(Eigen::MatrixXd &Q) {
    // build observability matrix for position measurement:
    QPosiMag_.block<DIM_MEASUREMENT_POSI_MAG, DIM_STATE>(0, 0) = GPosiMag_;
    for (int i = 1; i < DIM_STATE; ++i) {
        QPosiMag_.block<DIM_MEASUREMENT_POSI_MAG, DIM_STATE>(i*DIM_MEASUREMENT_POSI_MAG, 0) = (
            QPosiMag_.block<DIM_MEASUREMENT_POSI_MAG, DIM_STATE>((i - 1)*DIM_MEASUREMENT_POSI_MAG, 0) * F_
        );
    }

    Q = QPosiMag_;
}

/**
 * @brief  get Q for GNSS position, body velocity & magneto measurement
 * @param  void
 * @return QPosiVelMag
 */
 void ExtendedKalmanFilter::GetQPosiVelMag(Eigen::MatrixXd &Q) {
    // build observability matrix for position & velocity measurement:
    QPosVelMag_.block<DIM_MEASUREMENT_POSI_VEL_MAG, DIM_STATE>(0, 0) = GPosiVelMag_;
    for (int i = 1; i < DIM_STATE; ++i) {
        QPosVelMag_.block<DIM_MEASUREMENT_POSI_VEL_MAG, DIM_STATE>(i*DIM_MEASUREMENT_POSI_VEL_MAG, 0) = (
            QPosVelMag_.block<DIM_MEASUREMENT_POSI_VEL_MAG, DIM_STATE>((i - 1)*DIM_MEASUREMENT_POSI_VEL_MAG, 0) * F_
        );
    }
    Q = QPosVelMag_;
}

/**
 * @brief  update observability analysis
 * @param  measurement_type, measurement type
 * @return void
 */
void ExtendedKalmanFilter::UpdateObservabilityAnalysis(
    const double &time,
    const MeasurementType &measurement_type
) {
    // get Q:
    Eigen::MatrixXd Q;
    switch ( measurement_type ) {
        case MeasurementType::POSE:
            GetQPose(Q);
            break;
        case MeasurementType::POSE_VEL:
            GetQPoseVel(Q);
            break;
        case MeasurementType::POSI:
            GetQPosi(Q);
            break;
        case MeasurementType::POSI_VEL:
            GetQPosiVel(Q);
            break;
        case MeasurementType::POSI_MAG:
            GetQPosiMag(Q);
            break;
        case MeasurementType::POSI_VEL_MAG:
            GetQPosiVelMag(Q);
            break;
        default:
            break;
    }

    observability.time_.push_back(time);
    observability.Q_.push_back(Q);
}

/**
 * @brief  save observability analysis to persistent storage
 * @param  measurement_type, measurement type
 * @return void
 */
bool ExtendedKalmanFilter::SaveObservabilityAnalysis(
    const MeasurementType &measurement_type
) {
    // get fusion strategy:
    std::string type;
    switch ( measurement_type ) {
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
        case MeasurementType::POSI_MAG:
            type = std::string("position_magneto");
            break;
        case MeasurementType::POSI_VEL_MAG:
            type = std::string("position_velocity_magneto");
            break;
        default:
            return false;
            break;
    }

    // build Q_so:
    const int N = observability.Q_.at(0).rows();

    std::vector<std::vector<double>> q_data, q_so_data;

    Eigen::MatrixXd Qso(
        observability.Q_.size() * N,
        DIM_STATE
    );
    for (size_t i = 0; i < observability.Q_.size(); ++i) {
        const double &time = observability.time_.at(i);
        const Eigen::MatrixXd &Q = observability.Q_.at(i);

        Qso.block(i * N, 0, N, DIM_STATE) = Q;

        KalmanFilter::AnalyzeQ(DIM_STATE, time, Q, q_data);

        KalmanFilter::AnalyzeQ(DIM_STATE, time, Qso.block(0, 0, (i + 1)*N, DIM_STATE), q_so_data);
    }

    std::string q_data_csv = WORK_SPACE_PATH + "/slam_data/observability/" + type + ".csv";
    std::string q_so_data_csv = WORK_SPACE_PATH + "/slam_data/observability/" + type + "_som.csv";

    KalmanFilter::WriteAsCSV(DIM_STATE, q_data, q_data_csv);
    KalmanFilter::WriteAsCSV(DIM_STATE, q_so_data, q_so_data_csv);

    return true;
}

} // namespace lidar_localization