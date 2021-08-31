/*
 * @Description: LOAM front end facade
 * @Author: Ge Yao
 * @Date: 2021-01-30 22:38:22
 */
#include "lidar_localization/front_end/front_end_flow.hpp"

#include "glog/logging.h"

#include "lidar_localization/tools/file_manager.hpp"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh) {
    std::string config_file_path = WORK_SPACE_PATH + "/config/front_end/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    // subscriber to registered scans:
    InitSubscribers(nh, config_node["scan_registration"]["publisher"]);

    // LOAM front end workflow:
    front_end_ptr_ = std::make_unique<FrontEnd>();

    InitPublishers(nh, config_node["front_end"]["publisher"]);
}

bool FrontEndFlow::InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node) {
    filtered_cloud_sub_ptr_ = std::make_unique<CloudSubscriber>(
        nh, 
        config_node["filtered"]["topic_name"].as<std::string>(), 
        config_node["filtered"]["queue_size"].as<int>()
    );

    corner_points_sharp_sub_ptr_ = std::make_unique<CloudSubscriber>(
        nh, 
        config_node["sharp"]["topic_name"].as<std::string>(), 
        config_node["sharp"]["queue_size"].as<int>()
    );

    corner_points_less_sharp_sub_ptr_ = std::make_unique<CloudSubscriber>(
        nh, 
        config_node["less_sharp"]["topic_name"].as<std::string>(), 
        config_node["less_sharp"]["queue_size"].as<int>()
    );

    surf_points_flat_sub_ptr_ = std::make_unique<CloudSubscriber>(
        nh, 
        config_node["flat"]["topic_name"].as<std::string>(), 
        config_node["flat"]["queue_size"].as<int>()
    );

    surf_points_less_flat_sub_ptr_ = std::make_unique<CloudSubscriber>(
        nh, 
        config_node["less_flat"]["topic_name"].as<std::string>(), 
        config_node["less_flat"]["queue_size"].as<int>()
    );

    return true;
}

bool FrontEndFlow::InitPublishers(ros::NodeHandle& nh, const YAML::Node& config_node) {
    odom_scan_to_scan_pub_ptr_ = std::make_unique<OdometryPublisher>(
        nh, 
        config_node["odometry"]["topic_name"].as<std::string>(),
        config_node["odometry"]["frame_id"].as<std::string>(),
        config_node["odometry"]["child_frame_id"].as<std::string>(),
        config_node["odometry"]["queue_size"].as<int>()
    );

    return true;
}

bool FrontEndFlow::Run(void) {
    if (!ReadData()) {
        return false;
    }

    while( HasData() ) {
        // fetch registered scans:
        ValidData();

        // update scan-to-scan odometry:
        UpdateData();

        // publish data:
        PublishData();
    }

    return true;
}

bool FrontEndFlow::ReadData(void) {
    filtered_cloud_sub_ptr_->ParseData(filtered_cloud_buff_);
    corner_points_sharp_sub_ptr_->ParseData(corner_points_sharp_buff_);
    corner_points_less_sharp_sub_ptr_->ParseData(corner_points_less_sharp_buff_);
    surf_points_flat_sub_ptr_->ParseData(surf_points_flat_buff_);
    surf_points_less_flat_sub_ptr_->ParseData(surf_points_less_flat_buff_);

    return true;
}

bool FrontEndFlow::HasData(void) {
    if ( 
        filtered_cloud_buff_.empty() || 
        corner_points_sharp_buff_.empty() || 
        corner_points_less_sharp_buff_.empty() ||
        surf_points_flat_buff_.empty() ||
        surf_points_less_flat_buff_.empty() 
    ) {
        return false;
    }

    return true;
}

bool FrontEndFlow::ValidData() {
    filtered_cloud_ = filtered_cloud_buff_.front();

    corner_points_sharp_ = corner_points_sharp_buff_.front();
    corner_points_less_sharp_ = corner_points_less_sharp_buff_.front();
    surf_points_flat_ = surf_points_flat_buff_.front();
    surf_points_less_flat_ = surf_points_less_flat_buff_.front();

    double d_time = filtered_cloud_.time - corner_points_sharp_.time;
    if (d_time < -0.05) {
        filtered_cloud_buff_.pop_front();
        return false;
    }

    if (d_time > 0.05) {
        corner_points_sharp_buff_.pop_front();
        corner_points_less_sharp_buff_.pop_front();
        surf_points_flat_buff_.pop_front();
        surf_points_less_flat_buff_.pop_front();
        return false;
    }

    filtered_cloud_buff_.pop_front();

    corner_points_sharp_buff_.pop_front();
    corner_points_less_sharp_buff_.pop_front();
    surf_points_flat_buff_.pop_front();
    surf_points_less_flat_buff_.pop_front();

    return true;
}

bool FrontEndFlow::UpdateData(void) {
    front_end_ptr_->Update(
        *corner_points_sharp_.cloud_ptr,
        *corner_points_less_sharp_.cloud_ptr,
        *surf_points_flat_.cloud_ptr,
        *surf_points_less_flat_.cloud_ptr,
        odometry_
    );

    return true;
}

bool FrontEndFlow::PublishData(void) {
    LOG(WARNING) << "Publish scan-scan odom" << std::endl;
    odom_scan_to_scan_pub_ptr_->Publish(odometry_);

    return true;
}

} // namespace lidar_localization