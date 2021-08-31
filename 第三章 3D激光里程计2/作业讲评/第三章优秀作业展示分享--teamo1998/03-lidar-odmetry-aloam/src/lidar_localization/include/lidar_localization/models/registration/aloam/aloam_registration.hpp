#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_ALOAM_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_ALOAM_HPP_

#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <eigen3/Eigen/Dense>
#include <pcl/common/transforms.h>
#include <queue>
#include <deque>

#include "lidar_localization/models/registration/aloam/common.h"
#include "lidar_localization/models/registration/registration_interface.hpp"



namespace lidar_localization {

class ALOAMRegistration: public RegistrationInterface {
  public:
    ALOAMRegistration(const YAML::Node& node);
    
    ALOAMRegistration(
        float minimum_range,
        int scan_line
    );

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;

    // scan to scan 模式

    
    

  private:

    bool ExtractCornerandFlat(const CloudData::CLOUD_PTR& input);
    bool ScanToScan(const CloudData::CLOUD_PTR& input_source, 
                  const Eigen::Matrix4f& predict_pose, 
                  CloudData::CLOUD_PTR& result_cloud_ptr,
                  Eigen::Matrix4f& result_pose);


    bool ScanToMap(const CloudData::CLOUD_PTR& input_source, 
                  const Eigen::Matrix4f& predict_pose, 
                  CloudData::CLOUD_PTR& result_cloud_ptr,
                  Eigen::Matrix4f& result_pose);

    void removeClosedPointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
                              pcl::PointCloud<pcl::PointXYZ> &cloud_out, float thres);

    float minimum_range_;           // 雷达最小有效范围
    int scan_line_;                 // 雷达扫描线数


    int laserCloudCornerLastNum = 0;
    int laserCloudSurfLastNum = 0;
    int corner_correspondence = 0;
    int plane_correspondence = 0;
    // kd树
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsSharp;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsLessSharp;
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsFlat;
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlat;


    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerLast;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfLast;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFullRes;

    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_local_map;

    double sophus_param[6];
    CloudData::CLOUD_PTR input_target_;
    
    int Mode = 1;   // 1为to scan 2为 to map
    int frame_id = 0;
  

};


}

#endif

