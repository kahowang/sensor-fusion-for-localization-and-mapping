/*
 * @Description: Subscribe to PosVelMag messages
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */

#include "lidar_localization/subscriber/pos_vel_mag_subscriber.hpp"
#include "glog/logging.h"

namespace lidar_localization{

PosVelMagSubscriber::PosVelMagSubscriber(
    ros::NodeHandle& nh, 
    std::string topic_name, 
    size_t buff_size
)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &PosVelMagSubscriber::msg_callback, this);
}

void PosVelMagSubscriber::msg_callback(const PosVelMagConstPtr& pos_vel_mag_msg_ptr) {
    buff_mutex_.lock();

    PosVelMagData pos_vel_mag_data;
    pos_vel_mag_data.time = pos_vel_mag_msg_ptr->header.stamp.toSec();

    // a. set the position:
    pos_vel_mag_data.pos.x() = pos_vel_mag_msg_ptr->position.x;
    pos_vel_mag_data.pos.y() = pos_vel_mag_msg_ptr->position.y;
    pos_vel_mag_data.pos.z() = pos_vel_mag_msg_ptr->position.z;

    // b. set the body frame velocity:
    pos_vel_mag_data.vel.x() = pos_vel_mag_msg_ptr->velocity.x;
    pos_vel_mag_data.vel.y() = pos_vel_mag_msg_ptr->velocity.y;
    pos_vel_mag_data.vel.z() = pos_vel_mag_msg_ptr->velocity.z;

    // b. set the body frame magnetic field:
    pos_vel_mag_data.mag.x() = pos_vel_mag_msg_ptr->magnetic_field.x;
    pos_vel_mag_data.mag.y() = pos_vel_mag_msg_ptr->magnetic_field.y;
    pos_vel_mag_data.mag.z() = pos_vel_mag_msg_ptr->magnetic_field.z;

    new_pos_vel_mag_data_.push_back(pos_vel_mag_data);
    
    buff_mutex_.unlock();
}

void PosVelMagSubscriber::ParseData(std::deque<PosVelMagData>& pos_vel_mag_data_buff) {
    buff_mutex_.lock();

    if ( new_pos_vel_mag_data_.size() > 0 ) {
        pos_vel_mag_data_buff.insert(
            pos_vel_mag_data_buff.end(), 
            new_pos_vel_mag_data_.begin(), new_pos_vel_mag_data_.end()
        );
        new_pos_vel_mag_data_.clear();
    }

    buff_mutex_.unlock();
}

} // namespace lidar_localization