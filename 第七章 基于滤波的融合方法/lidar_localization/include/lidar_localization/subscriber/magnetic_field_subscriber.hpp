/*
 * @Description: Subscribe to magnetic field measurement
 * @Author: Ge Yao
 * @Date: 2020-11-25 23:07:14
 */

#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_MAGNETIC_FIELD_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_MAGNETIC_FIELD_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "sensor_msgs/MagneticField.h"

#include "lidar_localization/sensor_data/magnetic_field_data.hpp"

namespace lidar_localization {

class MagneticFieldSubscriber {
  public:
    MagneticFieldSubscriber(
      ros::NodeHandle& nh, 
      std::string topic_name, 
      size_t buff_size
    );
    MagneticFieldSubscriber() = default;
    void ParseData(std::deque<MagneticFieldData>& mag_field_data_buff);

  private:
    void msg_callback(const sensor_msgs::MagneticFieldConstPtr& mag_field_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<MagneticFieldData> new_mag_field_data_;

    std::mutex buff_mutex_; 
};

} // namespace 

#endif // LIDAR_LOCALIZATION_SUBSCRIBER_MAGNETIC_FIELD_SUBSCRIBER_HPP_