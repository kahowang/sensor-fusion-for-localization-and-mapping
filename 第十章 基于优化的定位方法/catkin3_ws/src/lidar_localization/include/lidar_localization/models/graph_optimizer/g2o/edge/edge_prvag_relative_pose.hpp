/*
 * @Description: g2o edge for LIO lidar frontend relative pose measurement
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
#ifndef LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_PRVAG_RELATIVE_POSE_HPP_
#define LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_PRVAG_RELATIVE_POSE_HPP_

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include "lidar_localization/models/graph_optimizer/g2o/vertex/vertex_prvag.hpp"

#include <g2o/core/base_binary_edge.h>

typedef Eigen::Matrix<double, 6, 1> Vector6d;

namespace g2o {

class EdgePRVAGRelativePose : public g2o::BaseBinaryEdge<6, Vector6d, g2o::VertexPRVAG, g2o::VertexPRVAG> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const int INDEX_P = 0;
    static const int INDEX_R = 3;

	EdgePRVAGRelativePose() 
	: g2o::BaseBinaryEdge<6, Vector6d, g2o::VertexPRVAG, g2o::VertexPRVAG>() {
	}

	virtual void computeError() override {
        const g2o::VertexPRVAG* v0 = static_cast<const g2o::VertexPRVAG*>(_vertices[0]);
        const g2o::VertexPRVAG* v1 = static_cast<const g2o::VertexPRVAG*>(_vertices[1]);

		const Eigen::Vector3d &pos_i = v0->estimate().pos;
		const Sophus::SO3d    &ori_i = v0->estimate().ori;

		const Eigen::Vector3d &pos_j = v1->estimate().pos;
		const Sophus::SO3d    &ori_j = v1->estimate().ori;

		const Eigen::Vector3d &pos_ij = _measurement.block<3, 1>(INDEX_P, 0);
		const Eigen::Vector3d &ori_ij = _measurement.block<3, 1>(INDEX_R, 0);

		_error.block(INDEX_P, 0, 3, 1) = ori_i.inverse() * (pos_j - pos_i) - pos_ij;
		_error.block(INDEX_R, 0, 3, 1) = (Sophus::SO3d::exp(ori_ij).inverse()*ori_i.inverse()*ori_j).log();
    }

    virtual void setMeasurement(const Vector6d& m) override {
		_measurement = m;
	}

	virtual bool read(std::istream& is) override {		
		Vector6d v;
		is >> v(INDEX_P + 0) >> v(INDEX_P + 1) >> v(INDEX_P + 2)
		   >> v(INDEX_R + 0) >> v(INDEX_R + 1) >> v(INDEX_R + 2);

    	setMeasurement(v);

		for (int i = 0; i < information().rows(); ++i) {
			for (int j = i; j < information().cols(); ++j) {
				is >> information()(i, j);
				// update cross-diagonal element:
				if (i != j) {
					information()(j, i) = information()(i, j);
				}
			}
		}

		return true;
	}

	virtual bool write(std::ostream& os) const override {
    	Vector6d v = _measurement;

		os << v(INDEX_P + 0) << " " <<v(INDEX_P + 1) << " " << v(INDEX_P + 2) << " "
		   << v(INDEX_R + 0) << " " <<v(INDEX_R + 1) << " " << v(INDEX_R + 2) << " ";

		for (int i = 0; i < information().rows(); ++i)
			for (int j = i; j < information().cols(); ++j)
				os << " " << information()(i, j);

		return os.good();
	}
};

} // namespace g2o

#endif // LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_PRVAG_RELATIVE_POSE_HPP_
