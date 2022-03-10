/*
 * @Description: Subscribe to PosVel messages
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */

#include "lidar_localization/subscriber/pos_vel_subscriber.hpp"
#include "glog/logging.h"

namespace lidar_localization{

PosVelSubscriber::PosVelSubscriber(
    ros::NodeHandle& nh, 
    std::string topic_name, 
    size_t buff_size
)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &PosVelSubscriber::msg_callback, this);
}

void PosVelSubscriber::msg_callback(const PosVelConstPtr& pos_vel_msg_ptr) {
    buff_mutex_.lock();

    PosVelData pos_vel_data;
    pos_vel_data.time = pos_vel_msg_ptr->header.stamp.toSec();

    // a. set the position:
    pos_vel_data.pos.x() = pos_vel_msg_ptr->position.x;
    pos_vel_data.pos.y() = pos_vel_msg_ptr->position.y;
    pos_vel_data.pos.z() = pos_vel_msg_ptr->position.z;

    // b. set the body frame velocity:
    pos_vel_data.vel.x() = pos_vel_msg_ptr->velocity.x;
    pos_vel_data.vel.y() = pos_vel_msg_ptr->velocity.y;
    pos_vel_data.vel.z() = pos_vel_msg_ptr->velocity.z;

    new_pos_vel_data_.push_back(pos_vel_data);
    
    buff_mutex_.unlock();
}

void PosVelSubscriber::ParseData(std::deque<PosVelData>& pos_vel_data_buff) {
    buff_mutex_.lock();

    if ( new_pos_vel_data_.size() > 0 ) {
        pos_vel_data_buff.insert(
            pos_vel_data_buff.end(), 
            new_pos_vel_data_.begin(), new_pos_vel_data_.end()
        );
        new_pos_vel_data_.clear();
    }

    buff_mutex_.unlock();
}

} // namespace lidar_localization