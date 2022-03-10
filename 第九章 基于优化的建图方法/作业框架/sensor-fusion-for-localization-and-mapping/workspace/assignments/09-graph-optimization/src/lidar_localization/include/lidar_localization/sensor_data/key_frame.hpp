/*
 * @Description: LIO key frame
 * @Author: Ren Qian
 * @Date: 2020-02-28 19:13:26
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_KEY_FRAME_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_KEY_FRAME_HPP_

#include <Eigen/Dense>

#include "lidar_localization/models/graph_optimizer/g2o/vertex/vertex_prvag.hpp"

namespace lidar_localization {

struct KeyFrame {
public:
    double time = 0.0;

    // key frame ID:
    unsigned int index = 0;
    
    // a. position & orientation:
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    // b. velocity:
    struct {
      Eigen::Vector3f v = Eigen::Vector3f::Zero();
      Eigen::Vector3f w = Eigen::Vector3f::Zero();
    } vel;
    // c. bias:
    struct {
      // c.1. accelerometer:
      Eigen::Vector3f accel = Eigen::Vector3f::Zero();
      // c.2. gyroscope:
      Eigen::Vector3f gyro = Eigen::Vector3f::Zero();
    } bias;

    KeyFrame() {}

    explicit KeyFrame(const int vertex_id, const g2o::PRVAG &prvag) {
      // set time:
      time = prvag.time;
      // set seq. ID:
      index = vertex_id;
      // set state:
      pose.block<3, 1>(0, 3) = prvag.pos.cast<float>();
      pose.block<3, 3>(0, 0) = prvag.ori.matrix().cast<float>();
      vel.v = prvag.vel.cast<float>();
      bias.accel = prvag.b_a.cast<float>();
      bias.gyro = prvag.b_g.cast<float>();
    }

    Eigen::Quaternionf GetQuaternion() const;
    Eigen::Vector3f GetTranslation() const;
};

}

#endif