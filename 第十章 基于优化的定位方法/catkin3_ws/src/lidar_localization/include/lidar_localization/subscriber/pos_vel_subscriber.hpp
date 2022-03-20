/*
 * @Description: Subscribe to PosVel messages
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_POS_VEL_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_POS_VEL_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "lidar_localization/PosVel.h"
#include "lidar_localization/sensor_data/pos_vel_data.hpp"

namespace lidar_localization {

class PosVelSubscriber {
  public:
    PosVelSubscriber(
      ros::NodeHandle& nh, 
      std::string topic_name, 
      size_t buff_size
    );
    PosVelSubscriber() = default;
    void ParseData(std::deque<PosVelData>& pos_vel_data_buff);

  private:
    void msg_callback(const PosVelConstPtr& pos_vel_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<PosVelData> new_pos_vel_data_;

    std::mutex buff_mutex_; 
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_SUBSCRIBER_POS_VEL_SUBSCRIBER_HPP_