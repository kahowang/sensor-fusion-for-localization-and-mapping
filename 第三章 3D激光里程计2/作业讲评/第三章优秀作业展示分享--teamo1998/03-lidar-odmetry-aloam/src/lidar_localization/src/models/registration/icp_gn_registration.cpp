/*
 * @Description: ICP SVD lidar odometry
 * @Author: Ge Yao
 * @Date: 2020-10-24 21:46:45
 */

#include <pcl/common/transforms.h>

#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include "sophus/se3.hpp"

#include "glog/logging.h"

#include "lidar_localization/models/registration/icp_gn_registration.hpp"

namespace lidar_localization {

ICPGNRegistration::ICPGNRegistration(
    const YAML::Node& node
) : input_target_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>()) {   // 构建一个kdtree用于搜索最近邻
    // parse params:
    // 获取配准器的参数
    float max_corr_dist = node["max_corr_dist"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    float euc_fitness_eps = node["euc_fitness_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
}

ICPGNRegistration::ICPGNRegistration(
    float max_corr_dist, 
    float trans_eps, 
    float euc_fitness_eps, 
    int max_iter
) : input_target_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>()) {
    SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
}

// 设置参数
bool ICPGNRegistration::SetRegistrationParam(
    float max_corr_dist, 
    float trans_eps, 
    float euc_fitness_eps, 
    int max_iter
) {
    // set params:
    max_corr_dist_ = max_corr_dist;
    trans_eps_ = trans_eps;
    euc_fitness_eps_ = euc_fitness_eps;
    max_iter_ = max_iter;

    LOG(INFO) << "ICP GN params:" << std::endl
              << "max_corr_dist: " << max_corr_dist_ << ", "
              << "trans_eps: " << trans_eps_ << ", "
              << "euc_fitness_eps: " << euc_fitness_eps_ << ", "
              << "max_iter: " << max_iter_ 
              << std::endl << std::endl;

    return true;
}

bool ICPGNRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    input_target_ = input_target;
    // 设置kdtree的输入点云
    input_target_kdtree_->setInputCloud(input_target_);

    return true;
}

// 重载父类的虚函数
bool ICPGNRegistration::ScanMatch(
    const CloudData::CLOUD_PTR& input_source, 
    const Eigen::Matrix4f& predict_pose, 
    CloudData::CLOUD_PTR& result_cloud_ptr,
    Eigen::Matrix4f& result_pose
) {
    input_source_ = input_source;

    // pre-process input source:
    CloudData::CLOUD_PTR transformed_input_source(new CloudData::CLOUD());
    pcl::transformPointCloud(*input_source_, *transformed_input_source, predict_pose);

    // init estimation:
    transformation_.setIdentity(); // 初值
    //
    // TODO: first option -- implement all computing logic on your own
    //
    // do estimation:
    int curr_iter = 0;
    while (curr_iter < max_iter_) {
        // TODO: apply current estimation:
        CloudData::CLOUD_PTR current_input_soucre(new CloudData::CLOUD());
        pcl::transformPointCloud(*transformed_input_source,*current_input_soucre,transformation_);

        // 构建H矩阵和B矩阵
        Eigen::Matrix<float, 6, 6> Hessian = Eigen::Matrix<float, 6, 6>::Zero();
        Eigen::Matrix<float, 6, 1> B = Eigen::Matrix<float, 6, 1>::Zero();

        const float MAX_CORR_DIST_SQR = max_corr_dist_ * max_corr_dist_;

        for (size_t j = 0; j < current_input_soucre->size(); ++j) {
            const CloudData::POINT &origin_point = transformed_input_source->points[j];

            //删除距离为无穷点
            if (!pcl::isFinite(origin_point)) {
                continue;
            }
            
            // 变换后的点
            const CloudData::POINT &transformed_point = current_input_soucre->at(j);


            std::vector<float> resultant_distances;
            std::vector<int> indices;
            // 在目标点云中搜索距离当前点最近的一个点
            input_target_kdtree_->nearestKSearch(transformed_point, 1, indices, resultant_distances);

            // 根据阈值剔除外点
            if (resultant_distances.front() > MAX_CORR_DIST_SQR) {
                continue;
            }

            // 最近点 
            Eigen::Vector3f nearest_point = Eigen::Vector3f(input_target_->at(indices.front()).x,
                                                            input_target_->at(indices.front()).y,
                                                            input_target_->at(indices.front()).z);

            // 变换后的点
            Eigen::Vector3f point_eigen(transformed_point.x, transformed_point.y, transformed_point.z);

            // 变换前的点
            Eigen::Vector3f origin_point_eigen(origin_point.x, origin_point.y, origin_point.z);

            // 误差
            Eigen::Vector3f error = point_eigen - nearest_point;

            Eigen::Matrix<float, 3, 6> Jacobian = Eigen::Matrix<float, 3, 6>::Zero();
            //构建雅克比矩阵
            Jacobian.leftCols(3) = Eigen::Matrix3f::Identity();
            Jacobian.rightCols(3) = -transformation_.block<3, 3>(0, 0) * Sophus::SO3f::hat(origin_point_eigen);

            //构建海森矩阵
            Hessian += Jacobian.transpose() * Jacobian;
            B += -Jacobian.transpose() * error;
        }
        
        if (Hessian.determinant() == 0) {
            continue;
        }
        Eigen::Matrix<float, 6, 1> delta_x = Hessian.inverse() * B;

        if(!IsSignificant(delta_x,trans_eps_)){
            break;
        }

        // 平移部分
        transformation_.block<3, 1>(0, 3) = transformation_.block<3, 1>(0, 3) + delta_x.head(3);

        // 旋转部分
        transformation_.block<3, 3>(0, 0) *= Sophus::SO3f::exp(delta_x.tail(3)).matrix();

        ++curr_iter;
    }

    // set output:
    result_pose = transformation_ * predict_pose;
    pcl::transformPointCloud(*input_source_, *result_cloud_ptr, result_pose);
    
    return true;
}

bool ICPGNRegistration::IsSignificant(
    const Eigen::Matrix<float, 6, 1> delta_x,
    const float trans_eps
) {
    // a. translation magnitude -- norm:
    float translation_magnitude = delta_x.head(3).norm();
    // b. rotation magnitude -- angle:
    float rotation_magnitude = fabs(
        acos(
            (Sophus::SO3f::exp(delta_x.tail(3)).matrix().trace() - 1.0f) / 2.0f
        )
    );

    return (
        (translation_magnitude > trans_eps) || 
        (rotation_magnitude > trans_eps)
    );
}

}
