/*
 * @Description: scan registration facade
 * @Author: Ge Yao
 * @Date: 2021-01-30 22:38:22
 */
#include "lidar_localization/scan_registration/scan_registration_flow.hpp"

#include "glog/logging.h"

#include "lidar_localization/tools/file_manager.hpp"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

ScanRegistrationFlow::ScanRegistrationFlow(ros::NodeHandle& nh) {
    std::string config_file_path = WORK_SPACE_PATH + "/config/front_end/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    // subscriber to raw Velodyne measurements:
    InitSubscribers(nh, config_node["scan_registration"]["subscriber"]);

    // scan registration workflow:
    scan_registration_ptr_ = std::make_unique<ScanRegistration>();

    filtered_cloud_data_.reset(new CloudData::CLOUD());
    corner_sharp_.reset(new CloudData::CLOUD());
    corner_less_sharp_.reset(new CloudData::CLOUD());
    surf_flat_.reset(new CloudData::CLOUD());
    surf_less_flat_.reset(new CloudData::CLOUD());

    // publishers of registered scans:
    InitPublishers(nh, config_node["scan_registration"]["publisher"]);
}

bool ScanRegistrationFlow::InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node) {
    cloud_sub_ptr_ = std::make_unique<CloudSubscriber>(
        nh, 
        config_node["velodyne"]["topic_name"].as<std::string>(), 
        config_node["velodyne"]["queue_size"].as<int>()
    );

    return true;
}

bool ScanRegistrationFlow::InitPublishers(ros::NodeHandle& nh, const YAML::Node& config_node) {
    // corner points:
    filtered_cloud_pub_ptr_ = std::make_unique<CloudPublisher>(
        nh, 
        config_node["filtered"]["topic_name"].as<std::string>(), 
        config_node["filtered"]["frame_id"].as<std::string>(),
        static_cast<size_t>(config_node["filtered"]["queue_size"].as<int>())
    );
    corner_points_sharp_pub_ptr_ = std::make_unique<CloudPublisher>(
        nh, 
        config_node["sharp"]["topic_name"].as<std::string>(), 
        config_node["sharp"]["frame_id"].as<std::string>(),
        static_cast<size_t>(config_node["sharp"]["queue_size"].as<int>())
    );
    corner_points_less_sharp_pub_ptr_ = std::make_unique<CloudPublisher>(
        nh, 
        config_node["less_sharp"]["topic_name"].as<std::string>(), 
        config_node["less_sharp"]["frame_id"].as<std::string>(),
        static_cast<size_t>(config_node["less_sharp"]["queue_size"].as<int>())
    );
    // surface points:
    surf_points_flat_pub_ptr_ = std::make_unique<CloudPublisher>(
        nh, 
        config_node["flat"]["topic_name"].as<std::string>(), 
        config_node["flat"]["frame_id"].as<std::string>(),
        static_cast<size_t>(config_node["flat"]["queue_size"].as<int>())
    );
    surf_points_less_flat_pub_ptr_ = std::make_unique<CloudPublisher>(
        nh, 
        config_node["less_flat"]["topic_name"].as<std::string>(), 
        config_node["less_flat"]["frame_id"].as<std::string>(),
        static_cast<size_t>(config_node["less_flat"]["queue_size"].as<int>())
    );
    // removed points:
    removed_points_pub_ptr_ = std::make_unique<CloudPublisher>(
        nh, 
        config_node["removed"]["topic_name"].as<std::string>(), 
        config_node["removed"]["frame_id"].as<std::string>(),
        static_cast<size_t>(config_node["removed"]["queue_size"].as<int>())
    );

    return true;
}

bool ScanRegistrationFlow::Run(void) {
    if (!ReadData()) {
        return false;
    }

    while( HasData() ) {
        // fetch Velodyne measurement:
        ValidData();

        // update velodyne measurement:
        UpdateData();

        // publish data:
        PublishData();
    }

    return true;
}

bool ScanRegistrationFlow::ReadData(void) {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);

    if ( cloud_data_buff_.empty() ) {
        return false;
    }

    return true;
}

bool ScanRegistrationFlow::HasData(void) {
    if ( cloud_data_buff_.empty() ) {
        return false;
    }

    return true;
}

bool ScanRegistrationFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();

    cloud_data_buff_.pop_front();

    return true;
}

bool ScanRegistrationFlow::UpdateData(void) {
    scan_registration_ptr_->Update(
        current_cloud_data_, 
        filtered_cloud_data_,
        corner_sharp_,
        corner_less_sharp_,
        surf_flat_,
        surf_less_flat_
    );

    return true;
}

bool ScanRegistrationFlow::PublishData(void) {
    filtered_cloud_pub_ptr_->Publish(filtered_cloud_data_, current_cloud_data_.time);

    corner_points_sharp_pub_ptr_->Publish(corner_sharp_, current_cloud_data_.time);
    corner_points_less_sharp_pub_ptr_->Publish(corner_less_sharp_, current_cloud_data_.time);
    surf_points_flat_pub_ptr_->Publish(surf_flat_, current_cloud_data_.time);
    surf_points_less_flat_pub_ptr_->Publish(surf_less_flat_, current_cloud_data_.time);

    return true;
}

} // namespace lidar_localization