/*
 * @Description: 闭环检测算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:52:45
 */
#ifndef LIDAR_LOCALIZATION_MAPPING_LOOP_CLOSING_LOOP_CLOSING_HPP_
#define LIDAR_LOCALIZATION_MAPPING_LOOP_CLOSING_LOOP_CLOSING_HPP_

#include <deque>
#include <Eigen/Dense>
#include <pcl/registration/ndt.h>
#include <yaml-cpp/yaml.h>

#include "lidar_localization/sensor_data/key_frame.hpp"
#include "lidar_localization/sensor_data/loop_pose.hpp"
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"
#include "lidar_localization/models/scan_context_manager/scan_context_manager.hpp"
#include "lidar_localization/models/registration/registration_interface.hpp"


namespace lidar_localization {
class LoopClosing {
  public:
    LoopClosing();

    bool Update(
      const CloudData &key_scan, 
      const KeyFrame &key_frame, 
      const KeyFrame &key_gnss
    );

    bool HasNewLoopPose();
    LoopPose& GetCurrentLoopPose();

    bool Save(void);

  private:
    bool InitWithConfig();
    bool InitParam(const YAML::Node& config_node);
    bool InitDataPath(const YAML::Node& config_node);
    bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);
    bool InitLoopClosure(const YAML::Node& config_node);
    bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
    
    bool DetectNearestKeyFrame(
      int& key_frame_index,
      float& yaw_change_in_rad
    );
    bool CloudRegistration(
      const int key_frame_index,
      const float yaw_change_in_rad
    );
    bool JointMap(
      const int key_frame_index, const float yaw_change_in_rad,
      CloudData::CLOUD_PTR& map_cloud_ptr, Eigen::Matrix4f& map_pose
    );
    bool JointScan(CloudData::CLOUD_PTR& scan_cloud_ptr, Eigen::Matrix4f& scan_pose);
    bool Registration(CloudData::CLOUD_PTR& map_cloud_ptr, 
                      CloudData::CLOUD_PTR& scan_cloud_ptr, 
                      Eigen::Matrix4f& scan_pose, 
                      Eigen::Matrix4f& result_pose);

  private:
    std::string key_frames_path_ = "";
    std::string scan_context_path_ = "";

    std::string loop_closure_method_ = "";

    int extend_frame_num_ = 3;
    int loop_step_ = 10;
    int diff_num_ = 100;
    float detect_area_ = 10.0;
    float fitness_score_limit_ = 2.0;

    std::shared_ptr<CloudFilterInterface> scan_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> map_filter_ptr_;
    std::shared_ptr<ScanContextManager> scan_context_manager_ptr_;
    std::shared_ptr<RegistrationInterface> registration_ptr_; 

    std::deque<KeyFrame> all_key_frames_;
    std::deque<KeyFrame> all_key_gnss_;

    LoopPose current_loop_pose_;
    bool has_new_loop_pose_ = false;
};
}

#endif