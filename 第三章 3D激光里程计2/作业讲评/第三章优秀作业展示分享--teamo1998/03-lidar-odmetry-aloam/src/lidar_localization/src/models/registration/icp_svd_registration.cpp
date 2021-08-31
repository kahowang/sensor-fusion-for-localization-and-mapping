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
) : input_target_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>()) {   // 构建一个kdtree用于搜索最近邻
    // parse params:
    // 获取配准器的参数
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

// 设置参数
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
    // 设置kdtree的输入点云
    input_target_kdtree_->setInputCloud(input_target_);

    return true;
}

// 重载父类的虚函数
bool ICPSVDRegistration::ScanMatch(
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
    transformation_.setIdentity();
    
    //
    // TODO: first option -- implement all computing logic on your own
    //
    // do estimation:
    int curr_iter = 0;
    while (curr_iter < max_iter_) {
        // TODO: apply current estimation:
        CloudData::CLOUD_PTR current_input_soucre(new CloudData::CLOUD());
        pcl::transformPointCloud(*transformed_input_source,*current_input_soucre,transformation_);
        // TODO: get correspondence:
        // 寻找相关的点对
        std::vector<Eigen::Vector3f> xs;
        std::vector<Eigen::Vector3f> ys;
        // TODO: do not have enough correspondence -- break:
        // 如果相关点对数目小于一定阈值，则认为无法匹配
        if(GetCorrespondence(current_input_soucre,xs,ys) < 3){
            break;
        }
        // TODO: update current transform:
        // 利用SVD分解获取R和T
        Eigen::Matrix4f delta;
        GetTransform(xs,ys,delta);

        // TODO: whether the transformation update is significant:
        // 判断delta是否符合更新要求
        if(!IsSignificant(delta,trans_eps_)){
            break;
        }

        // TODO: update transformation:
        // 更新估计值
        transformation_ = delta * transformation_;

        ++curr_iter;
    }

    // set output:
    result_pose = transformation_ * predict_pose;
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

    // 遍历当前帧点云中的每一个点
    for(size_t i = 0; i < input_source->points.size(); ++i){
        std::vector<int> index;
        std::vector<float> dist;

        // 搜索在目标点云中的最近点
        input_target_kdtree_->nearestKSearch(input_source->at(i),
                                            1,
                                            index,
                                            dist);

        // 判断最近点是否在距离阈值内
        if(dist.at(0) > MAX_CORR_DIST_SQR){
            continue;
        }

        // 将最近点加入相关点对
        Eigen::Vector3f x(input_target_->at(index.at(0)).x,
                            input_target_->at(index.at(0)).y,
                            input_target_->at(index.at(0)).z);

        Eigen::Vector3f y(input_source->at(i).x,
                            input_source->at(i).y,
                            input_source->at(i).z);

        xs.push_back(x);
        ys.push_back(y);

        ++num_corr;

    }

    // TODO: set up point correspondence

    return num_corr;
}

// 通过SVD分解计算R和t
void ICPSVDRegistration::GetTransform(
    const std::vector<Eigen::Vector3f> &xs,
    const std::vector<Eigen::Vector3f> &ys,
    Eigen::Matrix4f &transformation_
) {
    const size_t N = xs.size();

    // TODO: find centroids of mu_x and mu_y:
    // 计算均值
    Eigen::Vector3f mu_x = Eigen::Vector3f::Zero();
    Eigen::Vector3f mu_y = Eigen::Vector3f::Zero();
    for(size_t i = 0; i < N; ++i){
        mu_x += xs.at(i);
        mu_y += ys.at(i);
    }
    mu_x /= N;
    mu_y /= N;
    // TODO: build H:
    // 构建H
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    for(size_t i = 0; i < N ;++i){
        H += (ys.at(i) - mu_y) * (xs.at(i) - mu_x).transpose();
    }

    // TODO: solve R:
    // 求解R
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(H,Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3f R = svd.matrixV()*svd.matrixU().transpose();

    // TODO: solve t:
    // 求解t
    Eigen::Vector3f t = mu_x - R * mu_y;

    // TODO: set output:
    // 组织输出形式
    transformation_.setIdentity();
    transformation_.block(0,0,3,3) = R;
    transformation_.block(0,3,3,1) = t;
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