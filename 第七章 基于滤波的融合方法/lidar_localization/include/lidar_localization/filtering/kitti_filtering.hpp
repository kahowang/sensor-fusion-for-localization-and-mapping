/*
 * @Description: IMU-lidar fusion for localization workflow
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */
#ifndef LIDAR_LOCALIZATION_FILTERING_KITTI_FILTERING_HPP_
#define LIDAR_LOCALIZATION_FILTERING_KITTI_FILTERING_HPP_

#include <deque>
#include <unordered_map>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/sensor_data/imu_data.hpp"
#include "lidar_localization/sensor_data/pos_vel_data.hpp"
#include "lidar_localization/sensor_data/pose_data.hpp"

#include "lidar_localization/models/cloud_filter/box_filter.hpp"
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"

#include "lidar_localization/models/scan_context_manager/scan_context_manager.hpp"

#include "lidar_localization/models/registration/registration_interface.hpp"

#include "lidar_localization/models/kalman_filter/kalman_filter.hpp"

namespace lidar_localization {

class KITTIFiltering {
public:
  KITTIFiltering();

  bool Init(const CloudData &init_scan, const Eigen::Vector3f &init_vel,
            const IMUData &init_imu_data);

  bool Init(const Eigen::Matrix4f &init_pose, const Eigen::Vector3f &init_vel,
            const IMUData &init_imu_data);

  bool Update(const IMUData &imu_data);
  bool Correct(const IMUData &imu_data, const CloudData &cloud_data,
               const PosVelData &pos_vel_data, Eigen::Matrix4f &cloud_pose);

  // getters:
  bool HasInited() const { return has_inited_; }
  bool HasNewGlobalMap() const { return has_new_global_map_; }
  bool HasNewLocalMap() const { return has_new_local_map_; }

  void GetGlobalMap(CloudData::CLOUD_PTR &global_map);
  CloudData::CLOUD_PTR &GetLocalMap() { return local_map_ptr_; }
  CloudData::CLOUD_PTR &GetCurrentScan() { return current_scan_ptr_; }

  double GetTime(void) { return kalman_filter_ptr_->GetTime(); }
  Eigen::Matrix4f GetPose(void) { return current_pose_; }
  Eigen::Vector3f GetVel(void) { return current_vel_; }
  void GetOdometry(Eigen::Matrix4f &pose, Eigen::Vector3f &vel);

private:
  bool InitWithConfig(void);
  // a. filter initializer:
  bool InitFilter(std::string filter_user,
                  std::shared_ptr<CloudFilterInterface> &filter_ptr,
                  const YAML::Node &config_node);
  bool InitLocalMapSegmenter(const YAML::Node &config_node);
  bool InitFilters(const YAML::Node &config_node);
  // b. map initializer:
  bool InitGlobalMap(const YAML::Node &config_node);
  // c. scan context manager initializer:
  bool InitScanContextManager(const YAML::Node &config_node);
  // d. frontend initializer:
  bool
  InitRegistration(std::shared_ptr<RegistrationInterface> &registration_ptr,
                   const YAML::Node &config_node);
  // e. IMU-lidar fusion initializer:
  bool InitFusion(const YAML::Node &config_node);

  // local map setter:
  bool ResetLocalMap(float x, float y, float z);

  // init pose setter:
  bool SetInitScan(const CloudData &init_scan);
  bool SetInitGNSS(const Eigen::Matrix4f &init_pose);
  bool SetInitPose(const Eigen::Matrix4f &init_pose);

private:
  std::string map_path_ = "";
  std::string scan_context_path_ = "";

  std::string loop_closure_method_ = "";

  // a. global map:
  std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;
  // b. local map:
  std::shared_ptr<BoxFilter> local_map_segmenter_ptr_;
  std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
  // c. current scan:
  std::shared_ptr<CloudFilterInterface> current_scan_filter_ptr_;

  // scan context manager:
  std::shared_ptr<ScanContextManager> scan_context_manager_ptr_;
  // frontend:
  std::shared_ptr<RegistrationInterface> registration_ptr_;
  // IMU-lidar Kalman filter:
  struct {
    std::string FUSION_METHOD;

    std::unordered_map<std::string, KalmanFilter::MeasurementType>
        FUSION_STRATEGY_ID;
    KalmanFilter::MeasurementType FUSION_STRATEGY;
  } CONFIG;
  std::shared_ptr<KalmanFilter> kalman_filter_ptr_;
  KalmanFilter::Measurement current_measurement_;

  CloudData::CLOUD_PTR global_map_ptr_;
  CloudData::CLOUD_PTR local_map_ptr_;
  CloudData::CLOUD_PTR current_scan_ptr_;

  Eigen::Matrix4f current_gnss_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();
  Eigen::Vector3f current_vel_ = Eigen::Vector3f::Zero();

  bool has_inited_ = false;
  bool has_new_global_map_ = false;
  bool has_new_local_map_ = false;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_FILTERING_KITTI_FILTERING_HPP_