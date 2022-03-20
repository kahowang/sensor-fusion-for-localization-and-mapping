/*
 * @Description: lidar localization frontend, interface
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:52:45
 */
#ifndef LIDAR_LOCALIZATION_MATCHING_FRONT_END_MATCHING_HPP_
#define LIDAR_LOCALIZATION_MATCHING_FRONT_END_MATCHING_HPP_

#include <deque>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/sensor_data/pose_data.hpp"

#include "lidar_localization/models/scan_context_manager/scan_context_manager.hpp"
#include "lidar_localization/models/registration/registration_interface.hpp"
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"
#include "lidar_localization/models/cloud_filter/box_filter.hpp"

namespace lidar_localization {

class Matching {
public:
    Matching();

    bool HasInited(void);
    bool HasNewGlobalMap(void);
    bool HasNewLocalMap(void);

    Eigen::Matrix4f GetInitPose(void);
    CloudData::CLOUD_PTR& GetGlobalMap(void);
    CloudData::CLOUD_PTR& GetLocalMap(void);
    CloudData::CLOUD_PTR& GetCurrentScan(void);

    bool Update(
      const CloudData& cloud_data, 
      Eigen::Matrix4f& laser_pose, Eigen::Matrix4f &map_matching_pose
    );

    bool SetGNSSPose(const Eigen::Matrix4f& init_pose);
    bool SetScanContextPose(const CloudData& init_scan);

private:
    bool InitWithConfig();
    // 
    // point cloud map & measurement processors:
    // 
    bool InitFilter(
      const YAML::Node &config_node, std::string filter_user, 
      std::shared_ptr<CloudFilterInterface>& filter_ptr
    );
    bool InitLocalMapSegmenter(const YAML::Node& config_node);
    bool InitPointCloudProcessors(const YAML::Node& config_node);
    //
    // global map:
    //
    bool InitGlobalMap(const YAML::Node& config_node);
    //
    // lidar frontend for relative pose estimation:
    //
    bool InitRegistration(const YAML::Node& config_node, std::shared_ptr<RegistrationInterface>& registration_ptr);
    //
    // map matcher:
    //
    bool InitScanContextManager(const YAML::Node& config_node);

    bool SetInitPose(const Eigen::Matrix4f& init_pose);
    bool ResetLocalMap(float x, float y, float z);

private:
    std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;

    std::shared_ptr<BoxFilter> local_map_segmenter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;

    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;

    CloudData::CLOUD_PTR global_map_ptr_;
    CloudData::CLOUD_PTR local_map_ptr_;
    CloudData::CLOUD_PTR current_scan_ptr_;

    std::shared_ptr<RegistrationInterface> registration_ptr_;

    std::shared_ptr<ScanContextManager> scan_context_manager_ptr_;

    Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f current_gnss_pose_ = Eigen::Matrix4f::Identity();

    bool has_inited_ = false;
    bool has_new_global_map_ = false;
    bool has_new_local_map_ = false;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MATCHING_FRONTEND_MATCHING_HPP_