/*
 * @Description: front end workflow
 * @Author: Ge Yao
 * @Date: 2021-01-30 22:38:22
 */
#ifndef LIDAR_LOCALIZATION_FRONT_END_HPP_
#define LIDAR_LOCALIZATION_FRONT_END_HPP_

#include <memory>

#include <vector>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"

#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "lidar_localization/models/loam/aloam_registration.hpp"

namespace lidar_localization {

class FrontEnd {
  public:
    FrontEnd(void);

    bool Update(
      CloudData::CLOUD corner_sharp,
      CloudData::CLOUD corner_less_sharp,
      CloudData::CLOUD surf_flat,
      CloudData::CLOUD surf_less_flat,
      Eigen::Matrix4f& lidar_odometry
    );

  private:
    struct CornerPointAssociation {
      int query_index;

      double ratio;

      int associated_x_index;
      int associated_y_index;
    };

    struct SurfacePointAssociation {
      int query_index;

      double ratio;
      
      int associated_x_index;
      int associated_y_index;
      int associated_z_index;
    };

  private:
    bool InitParam(const YAML::Node& config_node);
    bool InitKdTrees(void);

    bool TransformToStart(const CloudData::POINT &input, CloudData::POINT &output);

    bool AssociateCornerPoints(
      const CloudData::CLOUD &corner_sharp,
      std::vector<CornerPointAssociation> &corner_point_associations
    );
    bool AssociateSurfacePoints(
      const CloudData::CLOUD &surf_flat,
      std::vector<SurfacePointAssociation> &surface_point_associations
    );

    bool AddEdgeFactors(
        const CloudData::CLOUD &corner_sharp,
        const std::vector<CornerPointAssociation> &corner_point_associations,
        CeresALOAMRegistration &aloam_registration
    );
    bool AddPlaneFactors(
        const CloudData::CLOUD &surf_flat,
        const std::vector<SurfacePointAssociation> &surface_point_associations,
        CeresALOAMRegistration &aloam_registration
    );

    bool SetTargetPoints(
      const CloudData::CLOUD &corner_less_sharp,
      const CloudData::CLOUD &surf_less_flat
    );

    bool UpdateOdometry(Eigen::Matrix4f& lidar_odometry);

  private:
    struct {
      float scan_period;
      float distance_thresh;
      float scan_thresh;
    } config_;

    // target corner & plane feature points:
    struct {
      CloudData::CLOUD_PTR candidate_corner_ptr;
      pcl::KdTreeFLANN<CloudData::POINT>::Ptr corner;

      CloudData::CLOUD_PTR candidate_surface_ptr;
      pcl::KdTreeFLANN<CloudData::POINT>::Ptr surface;
    } kdtree_;

    // whether the front end is inited:
    bool inited_{false};

    // relative pose:
    Eigen::Quaternionf dq_;
    Eigen::Vector3f dt_;

    // odometry:
    Eigen::Quaternionf q_;
    Eigen::Vector3f t_;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_FRONT_END_HPP_