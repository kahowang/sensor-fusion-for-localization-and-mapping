/*
 * @Description: scan registration facade
 * @Author: Ge Yao
 * @Date: 2021-01-30 22:38:22
 */
#include "lidar_localization/scan_registration/scan_registration.hpp"

#include "glog/logging.h"

#include "lidar_localization/tools/file_manager.hpp"
#include "lidar_localization/global_defination/global_defination.h"

#include <eigen3/Eigen/src/Core/VectorwiseOp.h>
#include <math.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace lidar_localization {

ScanRegistration::ScanRegistration(void) {
    std::string config_file_path = WORK_SPACE_PATH + "/config/front_end/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    // set LOAM scan registration params:
    InitParam(config_node["scan_registration"]["param"]);

    // init filters:
    InitFilters(config_node["scan_registration"]["param"]["filter"]);
}

bool ScanRegistration::InitParam(const YAML::Node& config_node) {
    config_.scan_period = config_node["scan_period"].as<float>();

    config_.num_scans = config_node["num_scans"].as<int>();
    if ( !(16 == config_.num_scans || 32 == config_.num_scans || 64 == config_.num_scans) ) {
        LOG(WARNING) << "Wrong scan number " << config_.num_scans << "!";
        return false;
    }

    config_.min_range = config_node["min_range"].as<float>();

    config_.neighborhood_size = config_node["neighborhood_size"].as<int>();

    config_.num_sectors = config_node["num_sectors"].as<int>();

    return true;
}

bool ScanRegistration::InitFilters(const YAML::Node& config_node) {
    surf_less_flat_filter_ptr_ = std::make_unique<pcl::VoxelGrid<CloudData::POINT>>();

    float leaf_size_x = config_node["surf_less_flat"]["leaf_size"][0].as<float>();
    float leaf_size_y = config_node["surf_less_flat"]["leaf_size"][1].as<float>();
    float leaf_size_z = config_node["surf_less_flat"]["leaf_size"][2].as<float>();

    surf_less_flat_filter_ptr_->setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);

    return true;
}

bool ScanRegistration::Update(
    const CloudData& input_cloud, 
    CloudData::CLOUD_PTR &output_cloud,
    CloudData::CLOUD_PTR &corner_sharp,
    CloudData::CLOUD_PTR &corner_less_sharp,
    CloudData::CLOUD_PTR &surf_flat,
    CloudData::CLOUD_PTR &surf_less_flat
) {
    // filter input point cloud:
    CloudData::CLOUD filtered_cloud;
    FilterByRange(*input_cloud.cloud_ptr, filtered_cloud);

    // sort point cloud by scan:
    output_cloud.reset(new CloudData::CLOUD());
    SortPointCloudByScan(filtered_cloud, *output_cloud);

    // get feature points:
    GetFeaturePoints(*output_cloud, corner_sharp, corner_less_sharp, surf_flat, surf_less_flat);

    return true;
}

bool ScanRegistration::FilterByRange(
    const CloudData::CLOUD &input_cloud, 
    CloudData::CLOUD &output_cloud
) {
    // first drop all NaNs:
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(input_cloud, output_cloud, indices);

    // then filter by range:
    size_t j = 0;

    for (size_t i = 0; i < output_cloud.points.size(); ++i)
    {
        Eigen::Vector3d point{output_cloud.points[i].x, output_cloud.points[i].y, output_cloud.points[i].z};

        // filter by range:
        if ( point.norm() < config_.min_range ) {
            continue;
        }

        output_cloud.points[j] = output_cloud.points[i];
        j++;
    }

    if (j != output_cloud.points.size())
    {
        output_cloud.points.resize(j);
    }

    output_cloud.height = 1;
    output_cloud.width = static_cast<size_t>(j);
    output_cloud.is_dense = true;

    return true;
}

bool ScanRegistration::GetScanId(const float &angle, int &scan_id) {
    if (16 == config_.num_scans)
    {
        scan_id = int((angle + 15) / 2 + 0.5);

        if (scan_id > (config_.num_scans - 1) || scan_id < 0)
        {
            return false;
        }
    }
    else if (32 == config_.num_scans)
    {
        scan_id = int((angle + 92.0/3.0) * 3.0 / 4.0);

        if (scan_id > (config_.num_scans - 1) || scan_id < 0)
        {
            return false;
        }
    }
    else if (64 == config_.num_scans)
    {   
        if (angle >= -8.83) {
            scan_id = int((2 - angle) * 3.0 + 0.5);
        }
        else {
            scan_id = config_.num_scans / 2 + int((-8.83 - angle) * 2.0 + 0.5);
        }
    
        // use [0 50]  > 50 remove outlies 
        if (angle > 2 || angle < -24.33 || scan_id > 50 || scan_id < 0)
        {
            return false;
        }
    }

    return true;
}

float ScanRegistration::GetCurvature(const CloudData::CLOUD &cloud, int point_index) {
    Eigen::Vector3f d = Eigen::Vector3f::Zero();

    for (int i = -config_.neighborhood_size; i <= config_.neighborhood_size; ++i) {
        if ( 0 != i ) {
            d.x() += cloud.points[point_index + i].x - cloud.points[point_index].x;
            d.y() += cloud.points[point_index + i].y - cloud.points[point_index].y;
            d.z() += cloud.points[point_index + i].z - cloud.points[point_index].z;
        }
    }

    return d.squaredNorm();
}

bool ScanRegistration::SortPointCloudByScan(const CloudData::CLOUD &input_cloud, CloudData::CLOUD &output_cloud) {
    // num. points:
    int input_size = input_cloud.points.size();
    // start & end measurements:
    const auto &start_point = input_cloud.points[0];
    const auto &end_point = input_cloud.points[input_size - 1];
    float start_yaw = -atan2(start_point.y, start_point.x);
    float end_yaw = -atan2(end_point.y, end_point.x) + 2 * M_PI;

    if (end_yaw - start_yaw > 3 * M_PI)
    {
        end_yaw -= 2 * M_PI;
    }
    else if (end_yaw - start_yaw < M_PI)
    {
        end_yaw += 2 * M_PI;
    }

    bool half_passed = false;
    std::vector<CloudData::CLOUD> scan(config_.num_scans);
    for (int i = 0; i < input_size; i++)
    {
        CloudData::POINT point;
        
        point.x = input_cloud.points[i].x;
        point.y = input_cloud.points[i].y;
        point.z = input_cloud.points[i].z;

        // get scan ID:
        float angle = 180.0f / M_PI * atan(point.z / hypot(point.x, point.y));
        int scan_id = 0;
        if ( !GetScanId(angle, scan_id) ) {
            continue;
        }

        // get current measurement yaw:
        float curr_yaw = -atan2(point.y, point.x);
        if ( !half_passed )
        { 
            if (curr_yaw < start_yaw - M_PI / 2)
            {
                curr_yaw += 2 * M_PI;
            }
            else if (curr_yaw > start_yaw + M_PI * 3 / 2)
            {
                curr_yaw -= 2 * M_PI;
            }

            if (curr_yaw - start_yaw > M_PI)
            {
                half_passed = true;
            }
        }
        else
        {
            curr_yaw += 2 * M_PI;
            if (curr_yaw < end_yaw - M_PI * 3 / 2)
            {
                curr_yaw += 2 * M_PI;
            }
            else if (curr_yaw > end_yaw + M_PI / 2)
            {
                curr_yaw -= 2 * M_PI;
            }
        }

        // set intensity. this field will be used by motion compensation:
        float scan_time = config_.scan_period * (curr_yaw - start_yaw) / (end_yaw - start_yaw);
        point.intensity = scan_id + scan_time;

        scan[scan_id].push_back(point); 
    }

    // set start & end index for each scan:
    for (int i = 0; i < config_.num_scans; i++)
    { 
        index_.scan.start[i] = output_cloud.size() + config_.neighborhood_size;

        output_cloud += scan[i];
        
        index_.scan.end[i] = output_cloud.size() - config_.neighborhood_size - 1;
    }

    // calculate curvature for each point:
    for (int i = config_.neighborhood_size; i < static_cast<int>(output_cloud.size()) - config_.neighborhood_size; ++i) {
        index_.point.curvature[i] = GetCurvature(output_cloud, i);
        index_.point.index[i] = i;
        index_.point.picked[i] = 0;
        index_.point.label[i] = 0;
    }

    return true;
}

bool ScanRegistration::PickInNeighborhood(const CloudData::CLOUD &cloud, const int point_index, const float thresh) {
    // forward:
    for (int i = 1; i <= config_.neighborhood_size; ++i)
    {
        Eigen::Vector3f d{
            cloud.points[point_index + i].x - cloud.points[point_index + i - 1].x,
            cloud.points[point_index + i].y - cloud.points[point_index + i - 1].y,
            cloud.points[point_index + i].z - cloud.points[point_index + i - 1].z
        };

        if ( d.squaredNorm() > thresh )
        {
            break;
        }

        // mark as picked:
        index_.point.picked[point_index + i] = 1;
    }

    // backward:
    for (int i = -1; i >= -config_.neighborhood_size; --i)
    {
        Eigen::Vector3f d{
            cloud.points[point_index + i].x - cloud.points[point_index + i + 1].x,
            cloud.points[point_index + i].y - cloud.points[point_index + i + 1].y,
            cloud.points[point_index + i].z - cloud.points[point_index + i + 1].z
        };
        if ( d.squaredNorm() > thresh )
        {
            break;
        }

        // mark as picked:
        index_.point.picked[point_index + i] = 1;
    }

    return true;
}

bool ScanRegistration::GetFeaturePoints(
    const CloudData::CLOUD &cloud, 
    CloudData::CLOUD_PTR &corner_sharp,
    CloudData::CLOUD_PTR &corner_less_sharp,
    CloudData::CLOUD_PTR &surf_flat,
    CloudData::CLOUD_PTR &surf_less_flat
) {
    corner_sharp.reset(new CloudData::CLOUD());
    corner_less_sharp.reset(new CloudData::CLOUD());
    surf_flat.reset(new CloudData::CLOUD());
    surf_less_flat.reset(new CloudData::CLOUD());

    for (int i = 0; i < config_.num_scans; ++i)
    {
        if( index_.scan.end[i] - index_.scan.start[i] < config_.num_sectors)
            continue;

        CloudData::CLOUD_PTR surf_less_flat_scan_ptr(new CloudData::CLOUD());
        for (int j = 0; j < config_.num_sectors; ++j)
        {
            int sector_start = index_.scan.start[i] + j * (index_.scan.end[i] - index_.scan.start[i]) / config_.num_sectors; 
            int sector_end = index_.scan.start[i] + (j + 1) * (index_.scan.end[i] - index_.scan.start[i]) / config_.num_sectors - 1;

            // sort point measurements in current sector by curvature:
            auto comp = [&](int i, int j){ return index_.point.curvature[i] < index_.point.curvature[j]; };
            std::sort(index_.point.index + sector_start, index_.point.index + sector_end + 1, comp);

            // mark top k most sharp as corner:
            int num_corners = 0;
            for (int k = sector_end; k >= sector_start; --k)
            {
                int ind = index_.point.index[k]; 

                if (index_.point.picked[ind] == 0 && index_.point.curvature[ind] > 0.1)
                {

                    ++num_corners;

                    if (num_corners <= 2)
                    {                        
                        index_.point.label[ind] = FeaturePoint::CORNER_SHARP;
                        corner_sharp->push_back(cloud.points[ind]);
                        corner_less_sharp->push_back(cloud.points[ind]);
                    }
                    else if (num_corners <= 20)
                    {                        
                        index_.point.label[ind] = FeaturePoint::CORNER_LESS_SHARP; 
                        corner_less_sharp->push_back(cloud.points[ind]);
                    }
                    else
                    {
                        break;
                    }

                    index_.point.picked[ind] = 1; 

                    PickInNeighborhood(cloud, ind, 0.05f);
                }
            }

            // mark top k most flat as surface:
            int num_surf = 0;
            for (int k = sector_start; k <= sector_end; k++)
            {
                int ind = index_.point.index[k];

                if (index_.point.picked[ind] == 0 && index_.point.curvature[ind] < 0.1)
                {

                    index_.point.label[ind] = FeaturePoint::SURF_FLAT; 
                    // surf_flat->push_back(cloud.points[ind]);

                    num_surf++;
                    if (num_surf >= 4)
                    { 
                        break;
                    }

                    index_.point.picked[ind] = 1;

                    PickInNeighborhood(cloud, ind, 0.05f);
                }
            }

            // mark the remaining as less flat surface:
            for (int k = sector_start; k <= sector_end; k++)
            {
                if (index_.point.label[k] <= 0)
                {
                    surf_less_flat_scan_ptr->push_back(cloud.points[k]);
                }
            }
        }

        surf_less_flat_filter_ptr_->setInputCloud(surf_less_flat_scan_ptr);

        CloudData::CLOUD surf_less_flat_scan_downsampled;
        surf_less_flat_filter_ptr_->filter(surf_less_flat_scan_downsampled);

        *surf_less_flat += surf_less_flat_scan_downsampled;
    }

    return true;
}

} // namespace lidar_localization