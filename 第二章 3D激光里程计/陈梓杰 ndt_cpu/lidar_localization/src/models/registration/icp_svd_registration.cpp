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

#include <ros/ros.h>

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
        CloudData::CLOUD_PTR temp_cloud(new CloudData::CLOUD());
        pcl::transformPointCloud(*transformed_input_source, *temp_cloud, transformation_);

        // TODO: get correspondence:
        std::vector<Eigen::Vector3f> source_point_set;
        std::vector<Eigen::Vector3f> target_point_set;
        
        // TODO: do not have enough correspondence -- break:
        if(GetCorrespondence(temp_cloud, source_point_set, target_point_set) < 3){
            ROS_WARN("icp_svd: correspondence less than 3");
            break;
        }

        // TODO: update current transform:
        Eigen::Matrix4f delta_T=Eigen::Matrix4f::Identity(); // 初始化时一定要赋值 不然有bug
        GetTransform(source_point_set, target_point_set, delta_T);

        // TODO: whether the transformation update is significant:
        if(!IsSignificant(delta_T, trans_eps_))
            break;

        // TODO: update transformation:
        transformation_ = delta_T * transformation_;

        ++curr_iter;
    }

    // set output:
    result_pose = transformation_ * predict_pose;
    Eigen::Matrix3f R=result_pose.block<3,3>(0,0);
    Eigen::Quaternionf q(R);
    q.normalize();
    result_pose.block<3,3>(0,0)=q.toRotationMatrix();
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
    for(size_t i=0; i<input_source->size(); i++){
        std::vector<int> correspondence_ind;
        std::vector<float> correspondence_dis;

        input_target_kdtree_->nearestKSearch(input_source->points[i], 1, correspondence_ind, correspondence_dis);

        if(correspondence_dis.at(0) < MAX_CORR_DIST_SQR){

            Eigen::Vector3f source_point = Eigen::Vector3f(
                input_source->points[i].x,  input_source->points[i].y,  input_source->points[i].z);

            Eigen::Vector3f target_point = Eigen::Vector3f(
                input_target_->at(correspondence_ind.at(0)).x,
                input_target_->at(correspondence_ind.at(0)).y,
                input_target_->at(correspondence_ind.at(0)).z);

            xs.push_back(source_point);
            ys.push_back(target_point);

            num_corr++;
        }
    }

    return num_corr;
}

void ICPSVDRegistration::GetTransform(
    const std::vector<Eigen::Vector3f> &xs,
    const std::vector<Eigen::Vector3f> &ys,
    Eigen::Matrix4f &transformation_
) {
    const size_t N = xs.size();

    // TODO: find centroids of mu_x and mu_y:
    Eigen::Vector3f mu_x = Eigen::Vector3f::Zero();
    Eigen::Vector3f mu_y = Eigen::Vector3f::Zero();
    for(size_t i=0; i<N; i++){
        mu_x += xs.at(i);
        mu_y += ys.at(i);
    }
    mu_x /= N; // source
    mu_y /= N; // target

    // TODO: build H:
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    for(size_t i=0; i<N; i++){
        Eigen::Vector3f temp_x, temp_y;

        temp_x = xs.at(i) - mu_x;
        temp_y = ys.at(i) - mu_y;

        H += temp_x * temp_y.transpose() ; 
    }

    // TODO: solve R:
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();
    Eigen::Matrix3f R = V * U.transpose();

    Eigen::Quaternionf q(R);
    q.normalize();
    R = q.toRotationMatrix();

    if(R.determinant() == -1){
        ROS_WARN("R is reflection...");
        V(2,2) = -V(2,2);
        R = V * U.transpose();
        q = Eigen::Quaternionf(R);
        q.normalize();
        R = q.toRotationMatrix();
    }

    // TODO: solve t:
    Eigen::Vector3f t = mu_y - R*mu_x;

    // TODO: set output:
    transformation_.block<3,3>(0,0) = R;
    transformation_.block<3,1>(0,3) = t;
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

