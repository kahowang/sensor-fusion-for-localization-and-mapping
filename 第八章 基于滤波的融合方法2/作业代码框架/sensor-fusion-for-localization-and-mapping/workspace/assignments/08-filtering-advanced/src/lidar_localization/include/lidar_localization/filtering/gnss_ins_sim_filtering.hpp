/*
 * @Description: Kalman filter based localization on GNSS-INS-Sim
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */
#ifndef LIDAR_LOCALIZATION_FILTERING_GNSS_INS_SIM_FILTERING_HPP_
#define LIDAR_LOCALIZATION_FILTERING_GNSS_INS_SIM_FILTERING_HPP_

#include <string>
#include <deque>
#include <unordered_map>

#include <Eigen/Dense>

#include <yaml-cpp/yaml.h>

#include "lidar_localization/EKFStd.h"

#include "lidar_localization/sensor_data/imu_data.hpp"
#include "lidar_localization/sensor_data/pos_vel_mag_data.hpp"
#include "lidar_localization/sensor_data/pose_data.hpp"

#include "lidar_localization/models/kalman_filter/kalman_filter.hpp"

namespace lidar_localization {

class GNSSINSSimFiltering {
  public:
    GNSSINSSimFiltering();

    bool Init(
      const Eigen::Matrix4f& init_pose,
      const Eigen::Vector3f &init_vel,
      const IMUData &init_imu_data
    );

    bool Update(
      const IMUData &imu_data
    );
    bool Correct(
      const IMUData &imu_data,
      const PosVelMagData &pos_vel_mag_data
    );

    // getters:
    bool HasInited() const { return has_inited_; }

    double GetTime(void) { return kalman_filter_ptr_->GetTime(); }
    Eigen::Matrix4f GetPose(void) { return current_pose_; }
    Eigen::Vector3f GetVel(void) { return current_vel_; }
    void GetOdometry(Eigen::Matrix4f &pose, Eigen::Vector3f &vel);
    void GetStandardDeviation(EKFStd &kf_std_msg);
    void SaveObservabilityAnalysis(void);
    
  private:
    bool InitWithConfig(void);
    bool InitFusion(const YAML::Node& config_node);

    // init pose setter:
    bool SetInitGNSS(const Eigen::Matrix4f& init_pose);
    bool SetInitPose(const Eigen::Matrix4f& init_pose);

  private:
    bool has_inited_ = false;

    // Kalman filter:
    struct {
      std::string FUSION_METHOD;

      std::unordered_map<std::string, KalmanFilter::MeasurementType> FUSION_STRATEGY_ID;
      KalmanFilter::MeasurementType FUSION_STRATEGY;
    } CONFIG;
    std::shared_ptr<KalmanFilter> kalman_filter_ptr_;
    KalmanFilter::Measurement current_measurement_;
    
    Eigen::Matrix4f current_gnss_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity(); 
    Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Vector3f current_vel_ = Eigen::Vector3f::Zero();
    KalmanFilter::Cov current_cov_;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_FILTERING_GNSS_INS_SIM_FILTERING_HPP_