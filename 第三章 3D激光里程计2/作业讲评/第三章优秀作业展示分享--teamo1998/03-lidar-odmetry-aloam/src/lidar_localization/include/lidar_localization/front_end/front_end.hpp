/*
 * @Description: 前端里程计算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:52:45
 */
#ifndef LIDAR_LOCALIZATION_FRONT_END_FRONT_END_HPP_
#define LIDAR_LOCALIZATION_FRONT_END_FRONT_END_HPP_

#include <deque>

#include <Eigen/Dense>

#include <pcl/filters/voxel_grid.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include <yaml-cpp/yaml.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"

#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"

#include "lidar_localization/models/registration/icp_registration.hpp"
#include "lidar_localization/models/registration/icp_svd_registration.hpp"

#include "lidar_localization/models/registration/ndt_registration.hpp"

#include "lidar_localization/models/registration/sicp/scip_registration.hpp"

#include "lidar_localization/models/registration/icp_gn_registration.hpp"

#include "lidar_localization/models/registration/aloam/aloam_registration.hpp"

// 前端匹配的核心类
namespace lidar_localization {
class FrontEnd {
  public:
    // 帧的结构，包含位姿和对应的点云数据
    struct Frame { 
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        CloudData cloud_data;
    };

  public:
    FrontEnd();

    bool InitWithConfig();
    // 给定点云，更新点云对应的位姿
    bool Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose);
    // 初始化坐标系原点
    bool SetInitPose(const Eigen::Matrix4f& init_pose);

    bool SaveMap();
    bool GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr);
    bool GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr);
    bool GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr);

  private:
    // 初始化参数
    bool InitParam(const YAML::Node& config_node);
    // 初始化数据路径
    bool InitDataPath(const YAML::Node& config_node);
    // 初始化配准器
    bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
    // 初始化滤波器
    bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);
    
    bool UpdateWithNewFrame(const Frame& new_key_frame);

  private:
    std::string data_path_ = "";

    // 滤波器指针
    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> display_filter_ptr_;

    // 配准的基类指针，用于多态
    std::shared_ptr<RegistrationInterface> registration_ptr_; 

    // 保存局部和全局关键帧
    std::deque<Frame> local_map_frames_;
    std::deque<Frame> global_map_frames_;

    bool has_new_local_map_ = false;
    bool has_new_global_map_ = false;

    // 数据指针
    CloudData::CLOUD_PTR local_map_ptr_;
    CloudData::CLOUD_PTR global_map_ptr_;
    CloudData::CLOUD_PTR result_cloud_ptr_;

    // 当前帧
    Frame current_frame_;

    // 初始位姿
    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();

    // 提取关键帧的参数
    float key_frame_distance_ = 2.0;
    int local_frame_num_ = 20;

    bool Aloam = false;
};

}

#endif