/*
 * @Description: Lidar measurement preprocess workflow
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */
#ifndef LIDAR_LOCALIZATION_DATA_PRETREAT_LIDAR_PREPROCESS_FLOW_HPP_
#define LIDAR_LOCALIZATION_DATA_PRETREAT_LIDAR_PREPROCESS_FLOW_HPP_

#include <ros/ros.h>
// subscriber
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/imu_subscriber.hpp"
#include "lidar_localization/subscriber/velocity_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include "lidar_localization/tf_listener/tf_listener.hpp"
// publisher
#include "lidar_localization/publisher/lidar_measurement_publisher.hpp"
// models
#include "lidar_localization/models/scan_adjust/distortion_adjust.hpp"

namespace lidar_localization {

class LidarPreprocessFlow {
  public:
    LidarPreprocessFlow(ros::NodeHandle& nh, std::string cloud_topic);

    bool Run();

  private:
    bool ReadData();
    bool InitCalibration();
    bool InitGNSS();
    bool HasData();
    bool ValidData();
    bool TransformData();
    bool PublishData();

  private:
    // subscriber
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
    std::shared_ptr<TFListener> lidar_to_imu_ptr_;
    // publisher
    std::shared_ptr<LidarMeasurementPublisher> lidar_measurement_pub_ptr_;
    
    // models
    std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_;

    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();

    std::deque<CloudData> cloud_data_buff_;
    std::deque<IMUData> imu_data_buff_;
    std::deque<VelocityData> velocity_data_buff_;
    std::deque<GNSSData> gnss_data_buff_;

    CloudData current_cloud_data_;
    IMUData current_imu_data_;
    VelocityData current_velocity_data_;
    GNSSData current_gnss_data_;

    Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_DATA_PRETREAT_LIDAR_PREPROCESS_FLOW_HPP_