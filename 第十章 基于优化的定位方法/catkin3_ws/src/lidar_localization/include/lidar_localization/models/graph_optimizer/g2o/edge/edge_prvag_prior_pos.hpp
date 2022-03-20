/*
 * @Description: g2o edge for LIO GNSS measurement
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
#ifndef LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_PRVAG_PRIOR_POS_HPP_
#define LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_PRVAG_PRIOR_POS_HPP_

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include "lidar_localization/models/graph_optimizer/g2o/vertex/vertex_prvag.hpp"

#include <g2o/core/base_unary_edge.h>

namespace g2o {

class EdgePRVAGPriorPos : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexPRVAG> {
  public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgePRVAGPriorPos() 
	 : g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexPRVAG>() {
	}

	virtual void computeError() override {
		const g2o::VertexPRVAG* v = static_cast<const g2o::VertexPRVAG*>(_vertices[0]);

		const Eigen::Vector3d &estimate = v->estimate().pos;

		_error = estimate - _measurement;
	}

    virtual void setMeasurement(const Eigen::Vector3d& m) override {
		_measurement = m;
	}

	virtual bool read(std::istream& is) override {
    	Eigen::Vector3d v;

		is >> v(0) >> v(1) >> v(2);

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
    	Eigen::Vector3d v = _measurement;

		os << v(0) << " " << v(1) << " " << v(2) << " ";
		
		for (int i = 0; i < information().rows(); ++i)
			for (int j = i; j < information().cols(); ++j)
				os << " " << information()(i, j);

		return os.good();
	}
};

} // namespace g2o

#endif // LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_PRVAG_PRIOR_POS_HPP_
