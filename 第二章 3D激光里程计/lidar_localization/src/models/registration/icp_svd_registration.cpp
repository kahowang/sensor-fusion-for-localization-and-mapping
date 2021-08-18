/*
 * @Description: ICP SVD lidar odometry
 * @Author: Ge Yao
 * @Date: 2020-10-24 21:46:45
 */

#include <pcl/common/transforms.h>

#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include "glog/logging.h"

#include "lidar_localization/models/registration/icp_svd_registration.hpp"

namespace lidar_localization {

ICPSVDRegistration::ICPSVDRegistration(
    const YAML::Node& node
) : input_target_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>()) {
    // parse params:
    float max_corr_dist = node["max_corr_dist"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    float euc_fitness_eps = node["euc_fitness_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
}

ICPSVDRegistration::ICPSVDRegistration(
    float max_corr_dist, 
    float trans_eps, 
    float euc_fitness_eps, 
    int max_iter
) : input_target_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>()) {
    SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
}

bool ICPSVDRegistration::SetRegistrationParam(
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

    LOG(INFO) << "ICP SVD params:" << std::endl
              << "max_corr_dist: " << max_corr_dist_ << ", "
              << "trans_eps: " << trans_eps_ << ", "
              << "euc_fitness_eps: " << euc_fitness_eps_ << ", "
              << "max_iter: " << max_iter_ 
              << std::endl << std::endl;

    return true;
}

bool ICPSVDRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    input_target_ = input_target;
    input_target_kdtree_->setInputCloud(input_target_);

    return true;
}

bool ICPSVDRegistration::ScanMatch(
    const CloudData::CLOUD_PTR& input_source,   //  输入待配准点云
    const Eigen::Matrix4f& predict_pose,                       // 预测位姿
    CloudData::CLOUD_PTR& result_cloud_ptr,           //  转换位姿后的点云
    Eigen::Matrix4f& result_pose                                         // 转换后的位姿   
) {
    input_source_ = input_source;

    // pre-process input source:
    CloudData::CLOUD_PTR transformed_input_source(new CloudData::CLOUD());
    pcl::transformPointCloud(*input_source_, *transformed_input_source, predict_pose);

    // init estimation:
    transformation_.setIdentity();
    
    //
    // TODO: first option -- implement all computing logic on your own
    //
    // do estimation:
    int curr_iter = 0;
    while (curr_iter < max_iter_) {                                                //  最大迭代次数
        // TODO: apply current estimation:
                // apply current estimation:
    CloudData::CLOUD_PTR curr_input_source(new CloudData::CLOUD());
    pcl::transformPointCloud(*transformed_input_source, *curr_input_source, transformation_);

        // TODO: get correspondence:
    std::vector<Eigen::Vector3f> xs;
    std::vector<Eigen::Vector3f> ys;

        // TODO: do not have enough correspondence -- break:
    if (GetCorrespondence(curr_input_source,xs,ys) <  3)     //  寻找最邻近点的点对，当匹配点少于3个退出
            break;

        // TODO: update current transform:
    Eigen::Matrix4f  delta_transformation;
    GetTransform(xs, ys, delta_transformation);

        // TODO: whether the transformation update is significant:
    if(!IsSignificant(delta_transformation, trans_eps_))      // 最大旋转矩阵
        break;
        // TODO: update transformation:
    transformation_ = delta_transformation *  transformation_;

        ++curr_iter;
    }

    // set output:
    result_pose = transformation_ * predict_pose;

    // 归一化
    Eigen::Quaternionf  qr(result_pose.block<3,3>(0,0));
    qr.normalize();
    Eigen::Vector3f  t  = result_pose.block<3,1>(0,3);
    result_pose.setIdentity();
    result_pose.block<3,3>(0,0) = qr.toRotationMatrix();
    result_pose.block<3,1>(0,3) = t;
    pcl::transformPointCloud(*input_source_, *result_cloud_ptr, result_pose);
    
    return true;
}

size_t ICPSVDRegistration::GetCorrespondence(
    const CloudData::CLOUD_PTR &input_source, 
    std::vector<Eigen::Vector3f> &xs,
    std::vector<Eigen::Vector3f> &ys
) {
    const float MAX_CORR_DIST_SQR = max_corr_dist_ * max_corr_dist_;

    size_t num_corr = 0;

    // TODO: set up point correspondence
    for(size_t  i =0; i < input_source->points.size(); ++i){
                std::vector<int>  corr_ind;    // index
                std::vector<float>  corr_sq_dis;     // correspondence_square_dis 
                input_target_kdtree_->nearestKSearch(
                        input_source->at(i),
                        1,
                        corr_ind, corr_sq_dis
                );       // kdtree  搜索
                
                if(corr_sq_dis.at(0) >  MAX_CORR_DIST_SQR)
                    continue;
                
            // add  correspondence:
            Eigen::Vector3f x(
                        input_target_->at(corr_ind.at(0)).x,
                        input_target_->at(corr_ind.at(0)).y,
                        input_target_->at(corr_ind.at(0)).z
            );
            Eigen::Vector3f y(
                        input_source->at(i).x,
                        input_source->at(i).y,
                        input_source->at(i).z
            );
            xs.push_back(x);
            ys.push_back(y);
            
            ++num_corr;
    }

    return num_corr;
}

void ICPSVDRegistration::GetTransform(
    const std::vector<Eigen::Vector3f> &xs,
    const std::vector<Eigen::Vector3f> &ys,
    Eigen::Matrix4f &transformation_
) {
    const size_t N = xs.size();

    // find centroids of mu_x and mu_y:
    Eigen::Vector3f mu_x = Eigen::Vector3f::Zero();
    Eigen::Vector3f mu_y = Eigen::Vector3f::Zero();
    for (size_t i = 0; i < N; ++i) {
        mu_x += xs.at(i);
        mu_y += ys.at(i);
    }
    mu_x /= N; 
    mu_y /= N;

    // build H:
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    for (size_t i = 0; i < N; ++i) {
        H += (ys.at(i) - mu_y) * (xs.at(i) - mu_x).transpose();
    }

    // solve R:
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();

    // solve t:
    Eigen::Vector3f t = mu_x - R * mu_y;

    // set output:
    transformation_.setIdentity();
    transformation_.block<3, 3>(0, 0) = R;
    transformation_.block<3, 1>(0, 3) = t;
}
bool ICPSVDRegistration::IsSignificant(
    const Eigen::Matrix4f &transformation,
    const float trans_eps
) {
    // a. translation magnitude -- norm:
    float translation_magnitude = transformation.block<3, 1>(0, 3).norm();
    // b. rotation magnitude -- angle:
    float rotation_magnitude = fabs(
        acos(
            (transformation.block<3, 3>(0, 0).trace() - 1.0f) / 2.0f
        )
    );

    return (
        (translation_magnitude > trans_eps) || 
        (rotation_magnitude > trans_eps)
    );
}

} // namespace lidar_localization