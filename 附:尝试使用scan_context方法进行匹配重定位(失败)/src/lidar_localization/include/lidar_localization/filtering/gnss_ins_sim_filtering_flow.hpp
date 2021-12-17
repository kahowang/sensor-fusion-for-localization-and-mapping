/*
 * @Description: Kalman filter based localization on GNSS-INS-Sim workflow
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */
#ifndef LIDAR_LOCALIZATION_FILTERING_GNSS_INS_SIM_FILTERING_FLOW_HPP_
#define LIDAR_LOCALIZATION_FILTERING_GNSS_INS_SIM_FILTERING_FLOW_HPP_

#include <ros/ros.h>

#include "lidar_localization/EKFStd.h"

// subscribers:
// a. IMU:
#include "lidar_localization/subscriber/imu_subscriber.hpp"
// b. synced GNSS-odo measurements:
#include "lidar_localization/subscriber/pos_vel_mag_subscriber.hpp"
// c. reference pose:
#include "lidar_localization/subscriber/odometry_subscriber.hpp"

// publishers:
#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/publisher/tf_broadcaster.hpp"

// filtering instance:
#include "lidar_localization/filtering/gnss_ins_sim_filtering.hpp"

#include "glog/logging.h"

namespace lidar_localization {

class GNSSINSSimFilteringFlow {
  public:
    GNSSINSSimFilteringFlow(ros::NodeHandle& nh);
    bool Run();

    // save odometry for evo evaluation:
    bool SaveOdometry(void);
    // save TOM measurements for observability analysis:
    bool SaveObservabilityAnalysis(void);

  private:
    bool ReadData();
    bool HasInited();
    
    bool HasData();

    bool HasIMUData(void) { 
      return ( !imu_data_buff_.empty() ); 
    }
    bool HasPosVelMagData(void) { 
      return ( !pos_vel_mag_data_buff_.empty() );
    }
    bool HasIMUComesFirst(void) const { 
      return imu_data_buff_.front().time < pos_vel_mag_data_buff_.front().time; 
    }

    bool ValidIMUData();
    bool ValidPosVelMagData();

    bool InitLocalization();
    
    bool UpdateLocalization();
    bool CorrectLocalization();

    bool PublishFusionOdom();
    bool PublishFusionStandardDeviation();

    bool UpdateOdometry(const double &time);
    /**
     * @brief  save pose in KITTI format for evo evaluation
     * @param  pose, input pose
     * @param  ofs, output file stream
     * @return true if success otherwise false
     */
    bool SavePose(
        const Eigen::Matrix4f& pose, 
        std::ofstream& ofs
    );

  private:
    // subscriber:
    // a. IMU:
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::deque<IMUData> imu_data_buff_;
    // b. synced GNSS-odo-mag measurement:
    std::shared_ptr<PosVelMagSubscriber> pos_vel_mag_sub_ptr_;
    std::deque<PosVelMagData> pos_vel_mag_data_buff_;
    // c. reference trajectory:
    std::shared_ptr<OdometrySubscriber> ref_pose_sub_ptr_;
    std::deque<PoseData> ref_pose_data_buff_;

    // publisher:
    // a. odometry:
    std::shared_ptr<OdometryPublisher> fused_odom_pub_ptr_;
    // b. tf:
    std::shared_ptr<TFBroadCaster> imu_tf_pub_ptr_;
    // c. standard deviation:
    ros::Publisher fused_std_pub_;
    // filtering instance:
    std::shared_ptr<GNSSINSSimFiltering> filtering_ptr_;

    IMUData current_imu_data_;
    PosVelMagData current_pos_vel_mag_data_;
    PoseData current_ref_pose_data_;
    
    // fused odometry:
    Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f fused_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Vector3f fused_vel_ = Eigen::Vector3f::Zero();
    EKFStd fused_std_;

    // trajectory for evo evaluation:
    struct {
      size_t N = 0;

      std::deque<double> time_;
      std::deque<Eigen::Matrix4f> fused_;
      std::deque<Eigen::Matrix4f> gnss_;
      std::deque<Eigen::Matrix4f> ref_;
    } trajectory;
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_FILTERING_GNSS_INS_SIM_FILTERING_FLOW_HPP_