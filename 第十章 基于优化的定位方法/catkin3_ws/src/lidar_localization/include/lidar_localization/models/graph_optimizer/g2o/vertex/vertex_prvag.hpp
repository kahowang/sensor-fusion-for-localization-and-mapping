/*
 * @Description: g2o vertex for LIO extended pose
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
#ifndef LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_VERTEX_VERTEX_PRVAG_HPP_
#define LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_VERTEX_VERTEX_PRVAG_HPP_

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

#include <g2o/core/base_vertex.h>

#include <mutex>

namespace g2o {

struct PRVAG {
    static const int INDEX_POS = 0;
    static const int INDEX_ORI = 3;
    static const int INDEX_VEL = 6;
    static const int INDEX_B_A = 9;
    static const int INDEX_B_G = 12;

    PRVAG() {}

    explicit PRVAG(const double *data) {
        pos = Eigen::Vector3d(data[INDEX_POS + 0], data[INDEX_POS + 1], data[INDEX_POS + 2]);
        ori = Sophus::SO3d::exp(
              Eigen::Vector3d(data[INDEX_ORI + 0], data[INDEX_ORI + 1], data[INDEX_ORI + 2])
        );
        vel = Eigen::Vector3d(data[INDEX_VEL + 0], data[INDEX_VEL + 1], data[INDEX_VEL + 2]);
        b_a = Eigen::Vector3d(data[INDEX_B_A + 0], data[INDEX_B_A + 1], data[INDEX_B_A + 2]);
        b_g = Eigen::Vector3d(data[INDEX_B_G + 0], data[INDEX_B_G + 1], data[INDEX_B_G + 2]);
    }

    void write(double *data) const {
        // get orientation in so3:
        auto log_ori = ori.log();

        for (size_t i = 0; i < 3; ++i) {
            data[INDEX_POS + i] = pos(i);
            data[INDEX_ORI + i] = log_ori(i);
            data[INDEX_VEL + i] = vel(i);
            data[INDEX_B_A + i] = b_a(i);
            data[INDEX_B_G + i] = b_g(i);
        }
    }

    double time = 0.0;

    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Sophus::SO3d ori = Sophus::SO3d();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d b_a = Eigen::Vector3d::Zero();
    Eigen::Vector3d b_g = Eigen::Vector3d::Zero();
};

class VertexPRVAG : public g2o::BaseVertex<15, PRVAG> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void setToOriginImpl() override {
        _estimate = PRVAG();
    }

    virtual void oplusImpl(const double *update) override {
        _estimate.pos += Eigen::Vector3d(
            update[PRVAG::INDEX_POS + 0], update[PRVAG::INDEX_POS + 1], update[PRVAG::INDEX_POS + 2]
        );
        _estimate.ori = _estimate.ori * Sophus::SO3d::exp(
            Eigen::Vector3d(
                update[PRVAG::INDEX_ORI + 0], update[PRVAG::INDEX_ORI + 1], update[PRVAG::INDEX_ORI + 2]
            )
        );
        _estimate.vel += Eigen::Vector3d(
            update[PRVAG::INDEX_VEL + 0], update[PRVAG::INDEX_VEL + 1], update[PRVAG::INDEX_VEL + 2]
        );

        Eigen::Vector3d d_b_a_i(
            update[PRVAG::INDEX_B_A + 0], update[PRVAG::INDEX_B_A + 1], update[PRVAG::INDEX_B_A + 2]
        );
        Eigen::Vector3d d_b_g_i(
            update[PRVAG::INDEX_B_G + 0], update[PRVAG::INDEX_B_G + 1], update[PRVAG::INDEX_B_G + 2]
        );

        _estimate.b_a += d_b_a_i;
        _estimate.b_g += d_b_g_i;

        updateDeltaBiases(d_b_a_i, d_b_g_i);
    }

    bool isUpdated(void) const { return _is_updated; }

    void updateDeltaBiases(
        const Eigen::Vector3d &d_b_a_i, 
        const Eigen::Vector3d &d_b_g_i
    ) {
        std::lock_guard<std::mutex> l(_m);

        _is_updated = true;

        _d_b_a_i += d_b_a_i;
        _d_b_g_i += d_b_g_i;
    }

    void getDeltaBiases(Eigen::Vector3d &d_b_a_i, Eigen::Vector3d &d_b_g_i) {
        std::lock_guard<std::mutex> l(_m);

        d_b_a_i = _d_b_a_i;
        d_b_g_i = _d_b_g_i;

        _d_b_a_i = _d_b_g_i = Eigen::Vector3d::Zero();

        _is_updated = false;
    }

    virtual bool read(std::istream &in) { return true; }

    virtual bool write(std::ostream &out) const { return true; }

private:
    std::mutex _m;
    bool _is_updated = false;

    Eigen::Vector3d _d_b_a_i = Eigen::Vector3d::Zero();
    Eigen::Vector3d _d_b_g_i = Eigen::Vector3d::Zero();
};

} // namespace g2o

#endif // LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_VERTEX_VERTEX_PRVAG_HPP_
