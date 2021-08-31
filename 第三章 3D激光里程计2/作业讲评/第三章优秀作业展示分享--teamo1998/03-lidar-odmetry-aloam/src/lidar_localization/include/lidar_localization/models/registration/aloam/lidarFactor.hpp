#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <sophus/common.hpp>


// 点到线的残差
struct LidarEdgeFactor
{
	LidarEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
					Eigen::Vector3d last_point_b_)
		: curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_){}

	template <typename T>
	bool operator()(const T * const parameters, T *residual) const
	{
		Eigen::Matrix<T, 6, 1> lie;
		lie << T(parameters[0]),T(parameters[1]),T(parameters[2]),T(parameters[3]),T(parameters[4]),T(parameters[5]);

        Sophus::SE3<T> trans = Sophus::SE3<T>::exp(lie);

		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};

		Eigen::Matrix<T, 3, 1> lp;

		lp = trans * cp;

		Eigen::Matrix<T,3,1> lpa{T(last_point_a.x()),T(last_point_a.y()),T(last_point_a.z())};
		Eigen::Matrix<T,3,1> lpb{T(last_point_b.x()),T(last_point_b.y()),T(last_point_b.z())};

		Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
		Eigen::Matrix<T, 3, 1> de = lpa - lpb;

		residual[0] = nu.norm()/de.norm();

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
									   const Eigen::Vector3d last_point_b_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarEdgeFactor, 1, 6>(
			new LidarEdgeFactor(curr_point_, last_point_a_, last_point_b_)));
	}

	Eigen::Vector3d curr_point, last_point_a, last_point_b;
};

// 点到面的特征
struct LidarPlaneFactor
{
	LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
					 Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_)
		: curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),
		  last_point_m(last_point_m_)
	{
		ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
		ljm_norm.normalize();
	}

	template <typename T>
	bool operator()(const T * const parameters, T *residuals) const
	{
		Eigen::Matrix<T, 6, 1> lie;
		lie << T(parameters[0]),T(parameters[1]),T(parameters[2]),T(parameters[3]),T(parameters[4]),T(parameters[5]);

        Sophus::SE3<T> trans = Sophus::SE3<T>::exp(lie);

		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};

		Eigen::Matrix<T,3,1> lpj{T(last_point_j.x()),T(last_point_j.y()),T(last_point_j.z())};
		Eigen::Matrix<T,3,1> lpl{T(last_point_l.x()),T(last_point_l.y()),T(last_point_l.z())};
		Eigen::Matrix<T,3,1> lpm{T(last_point_m.x()),T(last_point_m.y()),T(last_point_m.z())};

        Eigen::Matrix<T,3,1> lpi = trans * cp;
        Eigen::Matrix<T,3,1> pipj = lpi - lpj;
        Eigen::Matrix<T,3,1> ljm_norm = 
            (lpl - lpj).cross(lpm - lpj);
        ljm_norm.normalize();

        residuals[0] = pipj.dot(ljm_norm);

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_j_,
									   const Eigen::Vector3d last_point_l_, const Eigen::Vector3d last_point_m_
										)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarPlaneFactor, 1, 6>(
			new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_, last_point_m_)));
	}

	Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
	Eigen::Vector3d ljm_norm;
	double s;
};

struct LidarPlaneNormFactor
{

	LidarPlaneNormFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_,
						 double negative_OA_dot_norm_) : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_),
														 negative_OA_dot_norm(negative_OA_dot_norm_) {}

	template <typename T>
	bool operator()(const T * const parameters, T *residuals) const
	{
		Eigen::Matrix<T, 6, 1> lie;
		lie << T(parameters[0]),T(parameters[1]),T(parameters[2]),T(parameters[3]),T(parameters[4]),T(parameters[5]);

        Sophus::SE3<T> trans = Sophus::SE3<T>::exp(lie);

		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};

		Eigen::Matrix<T, 3, 1> point_w;

		point_w = trans * cp;

		Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z()));

		residuals[0] = norm.dot(point_w) + T(negative_OA_dot_norm);
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d plane_unit_norm_,
									   const double negative_OA_dot_norm_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarPlaneNormFactor, 1, 6>(
			new LidarPlaneNormFactor(curr_point_, plane_unit_norm_, negative_OA_dot_norm_)));
	}

	Eigen::Vector3d curr_point;
	Eigen::Vector3d plane_unit_norm;
	double negative_OA_dot_norm;
};


// struct LidarDistanceFactor
// {

// 	LidarDistanceFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d closed_point_) 
// 						: curr_point(curr_point_), closed_point(closed_point_){}

// 	template <typename T>
// 	bool operator()(const T *q, const T *t, T *residual) const
// 	{
// 		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
// 		Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
// 		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
// 		Eigen::Matrix<T, 3, 1> point_w;
// 		point_w = q_w_curr * cp + t_w_curr;


// 		residual[0] = point_w.x() - T(closed_point.x());
// 		residual[1] = point_w.y() - T(closed_point.y());
// 		residual[2] = point_w.z() - T(closed_point.z());
// 		return true;
// 	}

// 	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d closed_point_)
// 	{
// 		return (new ceres::AutoDiffCostFunction<
// 				LidarDistanceFactor, 3, 4, 3>(
// 			new LidarDistanceFactor(curr_point_, closed_point_)));
// 	}

// 	Eigen::Vector3d curr_point;
// 	Eigen::Vector3d closed_point;
// };




class SophusLidarEdgeFactor : public ceres::SizedCostFunction<1, 6> {
public:
    SophusLidarEdgeFactor(
        Eigen::Vector3d& cur_point, 
        Eigen::Vector3d& last_point_a, 
        Eigen::Vector3d& last_point_b) : 
            cur_point_(cur_point), 
    		last_point_a_(last_point_a), 
    		last_point_b_(last_point_b) {}
    
    virtual ~SophusLidarEdgeFactor() {}

    virtual bool Evaluate(double const * const * parameters, 
                          double *residuals, double **jacobians) const {
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> lie(parameters[0]);
        Sophus::SE3d T = Sophus::SE3d::exp(lie);

        Eigen::Vector3d lp = T * cur_point_;
        Eigen::Vector3d de = last_point_a_ - last_point_b_;
        Eigen::Vector3d nu = (lp - last_point_b_).cross(lp - last_point_a_);
        residuals[0] = nu.norm() / de.norm();

        if (jacobians != nullptr) {
            if (jacobians[0] != nullptr) {
                Eigen::Matrix3d lp_hat = Sophus::SO3d::hat(lp);
                Eigen::Matrix<double, 3, 6> dp_dse3;
                (dp_dse3.block<3, 3>(0, 0)).setIdentity();
                dp_dse3.block<3, 3>(0, 3) = -lp_hat;

                Eigen::Map<Eigen::Matrix<double, 1, 6>> J_se3(jacobians[0]);
                J_se3.setZero();
                
                Eigen::Matrix<double, 1, 3> de_dp = 
                    (nu / (de.norm() * nu.norm())).transpose() * Sophus::SO3d::hat(de);

                J_se3.block<1, 6>(0, 0) = de_dp * dp_dse3;
            }
        }
        return true;
    }

    Eigen::Vector3d cur_point_;
    Eigen::Vector3d last_point_a_;
    Eigen::Vector3d last_point_b_;
};

class SophusLidarPlaneFactor : public ceres::SizedCostFunction<1, 6> {
public:
    SophusLidarPlaneFactor(
        Eigen::Vector3d curr_point, 
        Eigen::Vector3d last_point_j,
        Eigen::Vector3d last_point_l, 
        Eigen::Vector3d last_point_m) : 
        curr_point_(curr_point), 
        last_point_j_(last_point_j), 
        last_point_l_(last_point_l), 
        last_point_m_(last_point_m) { }

    virtual ~SophusLidarPlaneFactor() {}
    
    virtual bool Evaluate(double const *const *parameters, 
                          double *residuals, 
                          double **jacobians) const {
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> lie(parameters[0]);
        Sophus::SE3d T = Sophus::SE3d::exp(lie);
        Eigen::Vector3d lpi = T * curr_point_;
        Eigen::Vector3d pipj = lpi - last_point_j_;
        Eigen::Vector3d ljm_norm = 
            (last_point_l_ - last_point_j_).cross(last_point_m_ - last_point_j_);
        ljm_norm.normalize();

        residuals[0] = pipj.dot(ljm_norm);
        // std::cout<<"Residual : "<<residuals[0]<<std::endl;
        if (jacobians != nullptr) {
            if (jacobians[0] != nullptr) {
                Eigen::Matrix3d lp_hat = Sophus::SO3d::hat(lpi);
                Eigen::Matrix<double, 3, 6> dp_dse3;
                (dp_dse3.block<3, 3>(0, 0)).setIdentity();
                dp_dse3.block<3, 3>(0, 3) = -lp_hat;

                Eigen::Matrix<double, 1, 6> J = ljm_norm.transpose() * dp_dse3;
                jacobians[0][0] = J(0);
                jacobians[0][1] = J(1);
                jacobians[0][2] = J(2);
                jacobians[0][3] = J(3);
                jacobians[0][4] = J(4);
                jacobians[0][5] = J(5);
            }
        }
        return true;
    }
    Eigen::Vector3d curr_point_, last_point_j_, last_point_l_, last_point_m_;
    
};

class PoseSE3Parameterization : public ceres::LocalParameterization {
public:
    PoseSE3Parameterization() {}

    virtual ~PoseSE3Parameterization() {}

    virtual bool Plus(const double *x, 
                      const double *delta, 
                      double *x_plus_delta) const {
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> lie(x);
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> delta_lie(delta);

        Sophus::SE3d T = Sophus::SE3d::exp(lie);
        Sophus::SE3d delta_T = Sophus::SE3d::exp(delta_lie);
        Eigen::Matrix<double, 6, 1> x_plus_delta_lie = (delta_T * T).log();
        
        for (size_t i = 0; i < 6; i++) {
            x_plus_delta[i] = x_plus_delta_lie(i, 0);
        }
        return true;
    }

    virtual bool ComputeJacobian(const double *x, double *jacobian) const {
        ceres::MatrixRef(jacobian, 6, 6) = ceres::Matrix::Identity(6, 6);
        return true;
    }

    virtual int GlobalSize() const {return Sophus::SE3d::DoF;}
    virtual int LocalSize() const {return Sophus::SE3d::DoF;}
};