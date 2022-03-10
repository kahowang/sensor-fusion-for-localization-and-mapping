/*
 * @Description: synced PosVelMagData publisher
 * @Author: Ge Yao
 * @Date: 2020-11-21 15:39:24
 */
#include "lidar_localization/publisher/pos_vel_mag_publisher.hpp"

namespace lidar_localization {

PosVelMagPublisher::PosVelMagPublisher(
    ros::NodeHandle& nh, 
    std::string topic_name, 
    std::string base_frame_id,
    std::string child_frame_id,
    int buff_size
)
    :nh_(nh) {

    publisher_ = nh_.advertise<PosVelMag>(topic_name, buff_size);
    pos_vel_mag_msg_.header.frame_id = base_frame_id;
    pos_vel_mag_msg_.child_frame_id = child_frame_id;
}

void PosVelMagPublisher::Publish(
    const PosVelMagData &pos_vel_mag_data, 
    const double &time
) {
    ros::Time ros_time(time);
    PublishData(pos_vel_mag_data, ros_time);
}

void PosVelMagPublisher::Publish(
    const PosVelMagData &pos_vel_mag_data
) {
    PublishData(pos_vel_mag_data, ros::Time::now());
}

bool PosVelMagPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}

void PosVelMagPublisher::PublishData(
    const PosVelMagData &pos_vel_mag_data, 
    ros::Time time
) {
    pos_vel_mag_msg_.header.stamp = time;

    // a. set position
    pos_vel_mag_msg_.position.x = pos_vel_mag_data.pos.x();
    pos_vel_mag_msg_.position.y = pos_vel_mag_data.pos.y();
    pos_vel_mag_msg_.position.z = pos_vel_mag_data.pos.z();

    // b. set velocity:
    pos_vel_mag_msg_.velocity.x = pos_vel_mag_data.vel.x();
    pos_vel_mag_msg_.velocity.y = pos_vel_mag_data.vel.y();
    pos_vel_mag_msg_.velocity.z = pos_vel_mag_data.vel.z();    

    // c. set magnetic field:
    pos_vel_mag_msg_.magnetic_field.x = pos_vel_mag_data.mag.x();
    pos_vel_mag_msg_.magnetic_field.y = pos_vel_mag_data.mag.y();
    pos_vel_mag_msg_.magnetic_field.z = pos_vel_mag_data.mag.z();

    publisher_.publish(pos_vel_mag_msg_);
}

} // namespace lidar_localization