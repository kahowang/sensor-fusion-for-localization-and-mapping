/*
 * @Description: scan registration facade
 * @Author: Ge Yao
 * @Date: 2021-01-30 22:38:22
 */
#include "lidar_localization/front_end/front_end.hpp"

#include "glog/logging.h"

#include "lidar_localization/tools/file_manager.hpp"
#include "lidar_localization/global_defination/global_defination.h"

#include <limits>
#include <math.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

namespace lidar_localization {

FrontEnd::FrontEnd(void) {
    std::string config_file_path = WORK_SPACE_PATH + "/config/front_end/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    // set LOAM front end params:
    InitParam(config_node["front_end"]["param"]);

    // init kdtrees for feature point association:
    InitKdTrees();
}

bool FrontEnd::InitParam(const YAML::Node& config_node) {
    config_.scan_period = 0.10f;
    config_.distance_thresh = config_node["distance_thresh"].as<float>();
    config_.scan_thresh = config_node["scan_thresh"].as<float>();

    return true;
}

bool FrontEnd::InitKdTrees(void) {
    kdtree_.corner.reset(new pcl::KdTreeFLANN<CloudData::POINT>());
    kdtree_.surface.reset(new pcl::KdTreeFLANN<CloudData::POINT>());

    return true;
}

bool FrontEnd::TransformToStart(const CloudData::POINT &input, CloudData::POINT &output) {
    // interpolation ratio
    float ratio = (input.intensity - int(input.intensity)) / config_.scan_period;

    Eigen::Quaternionf dq = Eigen::Quaternionf::Identity().slerp(ratio, dq_);
    Eigen::Vector3f dt = ratio * dt_;
    Eigen::Vector3f p(input.x, input.y, input.z);
    Eigen::Vector3f undistorted = dq * p + dt;

    output.x = undistorted.x();
    output.y = undistorted.y();
    output.z = undistorted.z();
    output.intensity = input.intensity;

    return true;
}

bool FrontEnd::AssociateCornerPoints(
    const CloudData::CLOUD &corner_sharp,
    std::vector<CornerPointAssociation> &corner_point_associations
) {
    // find correspondence for corner features:
    const int num_query_points = corner_sharp.points.size();

    CloudData::POINT query_point;
    std::vector<int> result_indices;
    std::vector<float> result_squared_distances;
    
    corner_point_associations.clear();
    for (int i = 0; i < num_query_points; ++i)
    {
        TransformToStart(corner_sharp.points[i], query_point);

        // find nearest corner in previous scan:
        kdtree_.corner->nearestKSearch(query_point, 1, result_indices, result_squared_distances);

        if (result_squared_distances[0] < config_.distance_thresh)
        {
            CornerPointAssociation corner_point_association;

            corner_point_association.query_index = i;
            corner_point_association.ratio = (query_point.intensity - int(query_point.intensity)) / config_.scan_period;

            // set the first associated point as the closest point:
            corner_point_association.associated_x_index = result_indices[0];

            // search the second associated point in nearby scans:
            auto get_scan_id = [](const CloudData::POINT &point) { return static_cast<int>(point.intensity); };

            int query_scan_id = get_scan_id(corner_sharp.points[corner_point_association.query_index]);

            float min_distance = std::numeric_limits<float>::infinity();
            int num_candidate_points = kdtree_.candidate_corner_ptr->points.size();
            const auto &candidate_point = kdtree_.candidate_corner_ptr->points;

            // search in the direction of increasing scan line
            for (int j = corner_point_association.associated_x_index + 1; j < num_candidate_points; ++j)
            {   
                // get current scan id:
                int curr_scan_id = get_scan_id(candidate_point[j]);

                // if in the same scan line, skip:
                if ( curr_scan_id <= query_scan_id )
                    continue;

                // if outside nearby scans, stop:
                if ( curr_scan_id > (query_scan_id + config_.scan_thresh) )
                    break;

                // calculate deviation:
                Eigen::Vector3f deviation{
                    candidate_point[j].x - query_point.x,
                    candidate_point[j].y - query_point.y,
                    candidate_point[j].z - query_point.z
                };
                float curr_distance = deviation.squaredNorm();

                // update associated point:
                if ( curr_distance < min_distance )
                {
                    min_distance = curr_distance;
                    corner_point_association.associated_y_index = j;
                }
            }

            // search in the direction of decreasing scan line
            for (int j = corner_point_association.associated_x_index - 1; j >= 0; --j)
            {
                // get current scan id:
                int curr_scan_id = get_scan_id(candidate_point[j]);

                // if in the same scan line, skip:
                if ( curr_scan_id >= query_scan_id )
                    continue;

                // if outside nearby scans, stop:
                if ( curr_scan_id < (query_scan_id - config_.scan_thresh))
                    break;

                // calculate deviation:
                Eigen::Vector3f deviation{
                    candidate_point[j].x - query_point.x,
                    candidate_point[j].y - query_point.y,
                    candidate_point[j].z - query_point.z
                };
                float curr_distance = deviation.squaredNorm();

                // update associated point:
                if ( curr_distance < min_distance )
                {
                    min_distance = curr_distance;
                    corner_point_association.associated_y_index = j;
                }
            }

            corner_point_associations.push_back(corner_point_association);
        }
    }

    return true;
}

bool FrontEnd::AssociateSurfacePoints(
    const CloudData::CLOUD &surf_flat,
    std::vector<SurfacePointAssociation> &surface_point_associations
) {
    const int num_query_points = surf_flat.points.size();

    CloudData::POINT query_point;
    std::vector<int> result_indices;
    std::vector<float> result_squared_distances;
    
    surface_point_associations.clear();
    for (int i = 0; i < num_query_points; ++i)
    {
        TransformToStart(surf_flat.points[i], query_point);

        // find nearest surface point in previous scan:
        kdtree_.surface->nearestKSearch(query_point, 1, result_indices, result_squared_distances);

        if (result_squared_distances[0] < config_.distance_thresh)
        {
            SurfacePointAssociation surface_point_association;

            surface_point_association.query_index = i;
            surface_point_association.ratio = (query_point.intensity - int(query_point.intensity)) / config_.scan_period;

            // set the first associated point as the closest point:
            surface_point_association.associated_x_index = result_indices[0];

            // search the second & third associated point in nearby scans:
            auto get_scan_id = [](const CloudData::POINT &point) { return static_cast<int>(point.intensity); };

            int query_scan_id = get_scan_id(surf_flat.points[surface_point_association.query_index]);

            float min_distance_y = std::numeric_limits<float>::infinity();
            float min_distance_z = std::numeric_limits<float>::infinity();
            int num_candidate_points = kdtree_.candidate_surface_ptr->points.size();
            const auto &candidate_point = kdtree_.candidate_surface_ptr->points;

            // search in the direction of increasing scan line
            for (int j = surface_point_association.associated_x_index + 1; j < num_candidate_points; ++j)
            {   
                // get current scan id:
                int curr_scan_id = get_scan_id(candidate_point[j]);

                // if outside nearby scans, stop:
                if ( curr_scan_id > (query_scan_id + config_.scan_thresh) )
                    break;

                // calculate deviation:
                Eigen::Vector3f deviation{
                    candidate_point[j].x - query_point.x,
                    candidate_point[j].y - query_point.y,
                    candidate_point[j].z - query_point.z
                };
                float curr_distance = deviation.squaredNorm();

                // update the associated surface point, not above current scan:
                if ( curr_scan_id <= query_scan_id && curr_distance < min_distance_y )
                {
                    min_distance_y = curr_distance;
                    surface_point_association.associated_y_index = j;
                }

                // update the associated surface point, above current scan:
                else if ( curr_scan_id > query_scan_id && curr_distance < min_distance_z )
                {
                    min_distance_z = curr_distance;
                    surface_point_association.associated_z_index = j;
                }
            }

            // search in the direction of decreasing scan line
            for (int j = surface_point_association.associated_x_index - 1; j >= 0; --j)
            {   
                // get current scan id:
                int curr_scan_id = get_scan_id(candidate_point[j]);

                // if outside nearby scans, stop:
                if ( curr_scan_id < (query_scan_id - config_.scan_thresh) )
                    break;

                // calculate deviation:
                Eigen::Vector3f deviation{
                    candidate_point[j].x - query_point.x,
                    candidate_point[j].y - query_point.y,
                    candidate_point[j].z - query_point.z
                };
                float curr_distance = deviation.squaredNorm();

                // update the associated surface point, not above current scan:
                if ( curr_scan_id >= query_scan_id && curr_distance < min_distance_y )
                {
                    min_distance_y = curr_distance;
                    surface_point_association.associated_y_index = j;
                }
                else if ( curr_scan_id < query_scan_id && curr_distance < min_distance_z )
                {
                    // find nearer point
                    min_distance_z = curr_distance;
                    surface_point_association.associated_z_index = j;
                }
            }

            surface_point_associations.push_back(surface_point_association);
        }
    }

    return true;
}

bool FrontEnd::AddEdgeFactors(
    const CloudData::CLOUD &corner_sharp,
    const std::vector<CornerPointAssociation> &corner_point_associations,
    CeresALOAMRegistration &aloam_registration
) {
    for (const auto &corner_point_association: corner_point_associations) {
        Eigen::Vector3d source{
            corner_sharp.points[corner_point_association.query_index].x,
            corner_sharp.points[corner_point_association.query_index].y,
            corner_sharp.points[corner_point_association.query_index].z
        };

        Eigen::Vector3d target_x{
            kdtree_.candidate_corner_ptr->points[corner_point_association.associated_x_index].x,
            kdtree_.candidate_corner_ptr->points[corner_point_association.associated_x_index].y,
            kdtree_.candidate_corner_ptr->points[corner_point_association.associated_x_index].z
        };
    
        Eigen::Vector3d target_y{
            kdtree_.candidate_corner_ptr->points[corner_point_association.associated_y_index].x,
            kdtree_.candidate_corner_ptr->points[corner_point_association.associated_y_index].y,
            kdtree_.candidate_corner_ptr->points[corner_point_association.associated_y_index].z
        };

        aloam_registration.AddEdgeFactor(
            source,
            target_x, target_y,
            corner_point_association.ratio
        );
    }

    return true;
}

bool FrontEnd::AddPlaneFactors(
    const CloudData::CLOUD &surf_flat,
    const std::vector<SurfacePointAssociation> &surface_point_associations,
    CeresALOAMRegistration &aloam_registration
) {
    for (const auto &surface_point_association: surface_point_associations) {
        Eigen::Vector3d source{
            surf_flat.points[surface_point_association.query_index].x,
            surf_flat.points[surface_point_association.query_index].y,
            surf_flat.points[surface_point_association.query_index].z
        };

        Eigen::Vector3d target_x{
            kdtree_.candidate_surface_ptr->points[surface_point_association.associated_x_index].x,
            kdtree_.candidate_surface_ptr->points[surface_point_association.associated_x_index].y,
            kdtree_.candidate_surface_ptr->points[surface_point_association.associated_x_index].z
        };
    
        Eigen::Vector3d target_y{
            kdtree_.candidate_surface_ptr->points[surface_point_association.associated_y_index].x,
            kdtree_.candidate_surface_ptr->points[surface_point_association.associated_y_index].y,
            kdtree_.candidate_surface_ptr->points[surface_point_association.associated_y_index].z
        };

        Eigen::Vector3d target_z{
            kdtree_.candidate_surface_ptr->points[surface_point_association.associated_z_index].x,
            kdtree_.candidate_surface_ptr->points[surface_point_association.associated_z_index].y,
            kdtree_.candidate_surface_ptr->points[surface_point_association.associated_z_index].z
        };

        aloam_registration.AddPlaneFactor(
            source,
            target_x, target_y, target_z,
            surface_point_association.ratio
        );
    }

    return true;
}

bool FrontEnd::SetTargetPoints(
    const CloudData::CLOUD &corner_less_sharp,
    const CloudData::CLOUD &surf_less_flat
) {
    kdtree_.candidate_corner_ptr.reset(new CloudData::CLOUD());
    pcl::copyPointCloud(corner_less_sharp, *kdtree_.candidate_corner_ptr);
    kdtree_.corner->setInputCloud(kdtree_.candidate_corner_ptr);

    kdtree_.candidate_surface_ptr.reset(new CloudData::CLOUD());
    pcl::copyPointCloud(surf_less_flat, *kdtree_.candidate_surface_ptr);
    kdtree_.surface->setInputCloud(kdtree_.candidate_surface_ptr);
    
    if ( !inited_ ) {
        inited_ = true;
    }

    return true;
}

bool FrontEnd::UpdateOdometry(Eigen::Matrix4f& lidar_odometry) {
    LOG(WARNING) << dq_.toRotationMatrix() << std::endl;
    LOG(WARNING) << dt_ << std::endl;

    q_ = q_ * dq_;
    t_ = q_ * dt_ + t_;

    lidar_odometry.block<3, 3>(0, 0) = q_.toRotationMatrix();
    lidar_odometry.block<3, 1>(0, 3) = t_;

    return true;
}

bool FrontEnd::Update(
    CloudData::CLOUD corner_sharp,
    CloudData::CLOUD corner_less_sharp,
    CloudData::CLOUD surf_flat,
    CloudData::CLOUD surf_less_flat,
    Eigen::Matrix4f& lidar_odometry
) {
    // feature point association:
    if ( inited_ ) {
        std::vector<CornerPointAssociation> corner_point_associations;
        AssociateCornerPoints(corner_sharp, corner_point_associations);

        std::vector<SurfacePointAssociation> surface_point_associations;
        AssociateSurfacePoints(surf_flat, surface_point_associations);

        if (corner_point_associations.size() + surface_point_associations.size() < 10) {
            return false;
        }

        // build problem:
        CeresALOAMRegistration aloam_registration(dq_, dt_);
        AddEdgeFactors(corner_sharp, corner_point_associations, aloam_registration);
        AddPlaneFactors(surf_flat, surface_point_associations, aloam_registration);

        // get relative pose:
        aloam_registration.Optimize();
        aloam_registration.GetOptimizedRelativePose(dq_, dt_);

        // update odometry:
        UpdateOdometry(lidar_odometry);
    }

    // set target feature points for next association:
    SetTargetPoints(corner_less_sharp, surf_less_flat);

    return true;
}

} // namespace lidar_localization