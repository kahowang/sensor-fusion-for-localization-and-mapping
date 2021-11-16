/*
 * @Description: Subscribe to PosVelMag messages
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_POS_VEL_MAG_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_POS_VEL_MAG_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "lidar_localization/PosVelMag.h"
#include "lidar_localization/sensor_data/pos_vel_mag_data.hpp"

namespace lidar_localization {

class PosVelMagSubscriber {
  public:
    PosVelMagSubscriber(
      ros::NodeHandle& nh, 
      std::string topic_name, 
      size_t buff_size
    );
    PosVelMagSubscriber() = default;
    void ParseData(std::deque<PosVelMagData>& pos_vel_mag_data_buff);

  private:
    void msg_callback(const PosVelMagConstPtr& pos_vel_mag_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<PosVelMagData> new_pos_vel_mag_data_;

    std::mutex buff_mutex_; 
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_SUBSCRIBER_POS_VEL_MAG_SUBSCRIBER_HPP_