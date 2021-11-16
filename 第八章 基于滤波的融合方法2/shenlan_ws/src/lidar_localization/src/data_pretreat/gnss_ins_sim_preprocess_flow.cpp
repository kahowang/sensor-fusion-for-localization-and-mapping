/*
 * @Description: GNSS-INS-Sim measurement preprocessing workflow
 * @Author: Ge Yao
 * @Date: 2020-11-21 15:39:24
 */
#include "lidar_localization/data_pretreat/gnss_ins_sim_preprocess_flow.hpp"

#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

GNSSINSSimPreprocessFlow::GNSSINSSimPreprocessFlow(
    ros::NodeHandle& nh
) {
    // subscribers:
    // a. GNSS-INS-Sim IMU:
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/sim/sensor/imu", 1000000);
    // b. GNSS-INS-Sim GNSS:
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/sim/sensor/gps/fix", 1000000);
    // c. GNSS-INS-Sim odo:
    odo_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/sim/sensor/odo", 1000000);
    // d. GNSS-INS-Sim magneto:
    mag_sub_ptr_ = std::make_shared<MagneticFieldSubscriber>(nh, "/sim/sensor/imu/mag", 1000000);
    // e. reference trajectory:
    ref_pose_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/reference_pose", 100000);

    // publishers:
    imu_pub_ptr_ = std::make_shared<IMUPublisher>(nh, "/synced_imu", "/imu_link", 100);
    pos_vel_mag_pub_ptr_ = std::make_shared<PosVelMagPublisher>(nh, "/synced_pos_vel_mag", "/map", "/imu_link", 100);
    gnss_pose_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_gnss_pose", "/map", "/imu_link", 100);
    ref_pose_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_reference_pose", "/map", "/imu_link", 100);
}

bool GNSSINSSimPreprocessFlow::Run() {
    if (!ReadData())
        return false;
    
    while( InitGNSS() && HasData() ) {
        if (!ValidData()) {
            LOG(INFO) << "Invalid data. Skip." << std::endl;
            continue;
        }

        TransformData();
        PublishData();
    }

    return true;
}

bool GNSSINSSimPreprocessFlow::ReadData() {
    // pipe sensor measurements into buffer:
    imu_sub_ptr_->ParseData(imu_data_buff_);
    gnss_sub_ptr_->ParseData(gnss_data_buff_);
    odo_sub_ptr_->ParseData(odo_data_buff_);
    mag_sub_ptr_->ParseData(mag_data_buff_);
    ref_pose_sub_ptr_->ParseData(ref_pose_data_buff_);

    return true;
}

bool GNSSINSSimPreprocessFlow::InitGNSS() {
    static bool gnss_inited = false;

    if ( !gnss_inited && !gnss_data_buff_.empty() ) {
        GNSSData gnss_data = gnss_data_buff_.front();
        gnss_data.InitOriginPosition();
        gnss_inited = true;

        LOG(INFO) << "Init local map frame at: " 
                  << gnss_data.latitude << ", "
                  << gnss_data.longitude << ", "
                  << gnss_data.altitude << std::endl;
    }

    return gnss_inited;
}

bool GNSSINSSimPreprocessFlow::HasData() {
    if (
        imu_data_buff_.size() == 0 ||
        gnss_data_buff_.size() == 0 ||
        odo_data_buff_.size() == 0 ||
        mag_data_buff_.size() == 0 ||
        ref_pose_data_buff_.size() == 0 
    ) {
        return false;
    }

    return true;
}

bool GNSSINSSimPreprocessFlow::ValidData() {
    current_imu_data_ = imu_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();
    current_odo_data_ = odo_data_buff_.front();
    current_mag_data_ = mag_data_buff_.front();
    current_ref_pose_data_ = ref_pose_data_buff_.front();

    double diff_gnss_time = current_imu_data_.time - current_gnss_data_.time;
    double diff_odo_time = current_imu_data_.time - current_odo_data_.time;
    double diff_mag_time = current_imu_data_.time - current_mag_data_.time;
    double diff_ref_pose_time = current_imu_data_.time - current_ref_pose_data_.time;

    //
    // this check assumes the frequency of GNSS-INS-Sim is 400Hz:
    //
    if ( diff_gnss_time < -0.00125 || diff_odo_time < -0.00125 || diff_ref_pose_time < -0.00125 || diff_mag_time < -0.00125 ) {
        imu_data_buff_.pop_front();
        return false;
    }

    if (diff_gnss_time > 0.00125) {
        gnss_data_buff_.pop_front();
        return false;
    }

    if (diff_odo_time > 0.00125) {
        odo_data_buff_.pop_front();
        return false;
    }

    if (diff_mag_time > 0.00125) {
        mag_data_buff_.pop_front();
        return false;
    }

    if (diff_ref_pose_time > 0.00125) {
        ref_pose_data_buff_.pop_front();
        return false;
    }

    imu_data_buff_.pop_front();
    gnss_data_buff_.pop_front();
    odo_data_buff_.pop_front();
    mag_data_buff_.pop_front();
    ref_pose_data_buff_.pop_front();

    return true;
}

bool GNSSINSSimPreprocessFlow::TransformData() {
    // a. get synced GNSS-odo measurement:
    current_gnss_data_.UpdateXYZ();
    
    gnss_pose_ = Eigen::Matrix4f::Identity();

    gnss_pose_(0,3) = +current_gnss_data_.local_N;
    gnss_pose_(1,3) = +current_gnss_data_.local_E;
    gnss_pose_(2,3) = -current_gnss_data_.local_U;

    pos_vel_mag_.pos.x() = +current_gnss_data_.local_N;
    pos_vel_mag_.pos.y() = +current_gnss_data_.local_E;
    pos_vel_mag_.pos.z() = -current_gnss_data_.local_U;

    pos_vel_mag_.vel.x() = current_odo_data_.linear_velocity.x;
    pos_vel_mag_.vel.y() = current_odo_data_.linear_velocity.y;
    pos_vel_mag_.vel.z() = current_odo_data_.linear_velocity.z;

    pos_vel_mag_.mag.x() = current_mag_data_.magnetic_field.x;
    pos_vel_mag_.mag.y() = current_mag_data_.magnetic_field.y;
    pos_vel_mag_.mag.z() = current_mag_data_.magnetic_field.z;

    // b. transform reference pose position from LLA to xyz:
    GNSSData ref_position;

    ref_position.latitude  = current_ref_pose_data_.pose(0, 3);
    ref_position.longitude = current_ref_pose_data_.pose(1, 3);
    ref_position.altitude  = current_ref_pose_data_.pose(2, 3);

    ref_position.UpdateXYZ();

    current_ref_pose_data_.pose(0,3) = +ref_position.local_N;
    current_ref_pose_data_.pose(1,3) = +ref_position.local_E;
    current_ref_pose_data_.pose(2,3) = -ref_position.local_U;

    return true;
}

bool GNSSINSSimPreprocessFlow::PublishData() {
    imu_pub_ptr_->Publish(current_imu_data_, current_imu_data_.time);

    pos_vel_mag_pub_ptr_->Publish(pos_vel_mag_, current_imu_data_.time);

    gnss_pose_pub_ptr_->Publish(gnss_pose_, current_imu_data_.time);
    ref_pose_pub_ptr_->Publish(current_ref_pose_data_.pose, current_ref_pose_data_.vel, current_imu_data_.time);

    return true;
}

} // namespace lidar_localization