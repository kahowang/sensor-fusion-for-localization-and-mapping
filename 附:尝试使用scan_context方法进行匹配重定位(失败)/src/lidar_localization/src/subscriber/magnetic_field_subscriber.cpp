/*
 * @Description: Subscribe to magnetic field measurement
 * @Author: Ge Yao
 * @Date: 2020-11-25 23:07:14
 */

#include "lidar_localization/subscriber/magnetic_field_subscriber.hpp"

#include "glog/logging.h"

namespace lidar_localization{

MagneticFieldSubscriber::MagneticFieldSubscriber(
    ros::NodeHandle& nh, 
    std::string topic_name, 
    size_t buff_size
) :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &MagneticFieldSubscriber::msg_callback, this);
}

void MagneticFieldSubscriber::msg_callback(
    const sensor_msgs::MagneticFieldConstPtr& mag_field_msg_ptr
) {
    buff_mutex_.lock();

    MagneticFieldData mag_field_data;
    mag_field_data.time = mag_field_msg_ptr->header.stamp.toSec();

    mag_field_data.magnetic_field.x = mag_field_msg_ptr->magnetic_field.x;
    mag_field_data.magnetic_field.y = mag_field_msg_ptr->magnetic_field.y;
    mag_field_data.magnetic_field.z = mag_field_msg_ptr->magnetic_field.z;

    new_mag_field_data_.push_back(mag_field_data);

    buff_mutex_.unlock();
}

void MagneticFieldSubscriber::ParseData(
    std::deque<MagneticFieldData>& mag_field_data_buff
) {
    buff_mutex_.lock();

    if ( new_mag_field_data_.size() > 0 ) {
        mag_field_data_buff.insert(mag_field_data_buff.end(), new_mag_field_data_.begin(), new_mag_field_data_.end());
        new_mag_field_data_.clear();
    }

    buff_mutex_.unlock();
}

} // namespace lidar_localization