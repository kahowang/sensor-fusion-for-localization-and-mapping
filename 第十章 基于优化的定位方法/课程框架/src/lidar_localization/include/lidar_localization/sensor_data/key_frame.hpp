/*
 * @Description: LIO key frame
 * @Author: Ren Qian
 * @Date: 2020-02-28 19:13:26
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_KEY_FRAME_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_KEY_FRAME_HPP_

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

#include "lidar_localization/models/graph_optimizer/g2o/vertex/vertex_prvag.hpp"

namespace lidar_localization {

struct KeyFrame {
public:
    static const int INDEX_P = 0;
    static const int INDEX_R = 3;
    static const int INDEX_V = 6;
    static const int INDEX_A = 9;
    static const int INDEX_G = 12;

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

    explicit KeyFrame(const int param_index, const double &T, const double *prvag) {
      // set time:
      time = T;
      // set seq. ID:
      index = param_index;
      // set state:
      Eigen::Map<const Eigen::Vector3d>     pos(prvag + INDEX_P);
      Eigen::Map<const Eigen::Vector3d> log_ori(prvag + INDEX_R);
      Eigen::Map<const Eigen::Vector3d>       v(prvag + INDEX_V);
      Eigen::Map<const Eigen::Vector3d>     b_a(prvag + INDEX_A);
      Eigen::Map<const Eigen::Vector3d>     b_g(prvag + INDEX_G);

      pose.block<3, 1>(0, 3) = pos.cast<float>();
      pose.block<3, 3>(0, 0) = Sophus::SO3d::exp(log_ori).matrix().cast<float>();

      vel.v = v.cast<float>();
      
      bias.accel = b_a.cast<float>();
      bias.gyro = b_g.cast<float>();
    }

    Eigen::Quaternionf GetQuaternion() const;
    Eigen::Vector3f GetTranslation() const;
};

}

#endif