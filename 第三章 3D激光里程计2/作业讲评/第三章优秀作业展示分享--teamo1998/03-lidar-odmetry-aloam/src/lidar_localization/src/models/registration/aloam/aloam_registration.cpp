#include "lidar_localization/models/registration/aloam/aloam_registration.hpp"
#include "lidar_localization/models/registration/aloam/lidarFactor.hpp"

constexpr double SCAN_PERIOD = 0.1;
constexpr double DISTANCE_SQ_THRESHOLD = 25;
constexpr double NEARBY_SCAN = 2.5;

namespace lidar_localization {


float cloudCurvature[1500000];
int cloudSortInd[1500000];
int cloudNeighborPicked[1500000];
int cloudLabel[1500000];

bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }
// 构造函数
ALOAMRegistration::ALOAMRegistration(const YAML::Node& node):kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>()),
                                                            kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>()),
                                                            cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZI>()),
                                                            cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZI>()),
                                                            surfPointsFlat(new pcl::PointCloud<pcl::PointXYZI>()),
                                                            surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZI>()),
                                                            laserCloudCornerLast(new pcl::PointCloud<pcl::PointXYZI>()),
                                                            laserCloudSurfLast(new pcl::PointCloud<pcl::PointXYZI>()),
                                                            laserCloudFullRes(new pcl::PointCloud<pcl::PointXYZI>()),
                                                            kdtree_local_map(new pcl::KdTreeFLANN<pcl::PointXYZ>()){
    minimum_range_ = node["minimum_range"].as<float>();
    scan_line_ = node["scan_line"].as<int>();

    Sophus::SE3d pose_se3;
    pose_se3.setRotationMatrix(Eigen::Matrix3d::Identity());
    pose_se3.trans(Eigen::Vector3d::Zero());
    Eigen::Matrix<double, 6, 1> pose_vec = pose_se3.log();
    sophus_param[0] = pose_vec(0, 0);
    sophus_param[1] = pose_vec(1, 0);
    sophus_param[2] = pose_vec(2, 0);
    sophus_param[3] = pose_vec(3, 0);
    sophus_param[4] = pose_vec(4, 0);
    sophus_param[5] = pose_vec(5, 0);

    // 初始化指针
    
}

ALOAMRegistration::ALOAMRegistration(
        float minimum_range,
        int scan_line):kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>()),
                        kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>()),
                        cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZI>()),
                        cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZI>()),
                        surfPointsFlat(new pcl::PointCloud<pcl::PointXYZI>()),
                        surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZI>()),
                        laserCloudCornerLast(new pcl::PointCloud<pcl::PointXYZI>()),
                        laserCloudSurfLast(new pcl::PointCloud<pcl::PointXYZI>()),
                        laserCloudFullRes(new pcl::PointCloud<pcl::PointXYZI>()),
                        kdtree_local_map(new pcl::KdTreeFLANN<pcl::PointXYZ>()){
    
    minimum_range_ = minimum_range;
    scan_line_ = scan_line;
    Sophus::SE3d pose_se3;
    pose_se3.setRotationMatrix(Eigen::Matrix3d::Identity());
    pose_se3.trans(Eigen::Vector3d::Zero());
    Eigen::Matrix<double, 6, 1> pose_vec = pose_se3.log();
    sophus_param[0] = pose_vec(0, 0);
    sophus_param[1] = pose_vec(1, 0);
    sophus_param[2] = pose_vec(2, 0);
    sophus_param[3] = pose_vec(3, 0);
    sophus_param[4] = pose_vec(4, 0);
    sophus_param[5] = pose_vec(5, 0);
}

// TODO
bool ALOAMRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    // 第一帧点云
    input_target_ = input_target;
    if(frame_id == 0){
        // 提取角点和面点
        ExtractCornerandFlat(input_target_);

        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTemp = cornerPointsLessSharp;
        cornerPointsLessSharp = laserCloudCornerLast;
        laserCloudCornerLast = laserCloudTemp;

        laserCloudTemp = surfPointsLessFlat;
        surfPointsLessFlat = laserCloudSurfLast;
        laserCloudSurfLast = laserCloudTemp;

        laserCloudCornerLastNum = laserCloudCornerLast->points.size();
        laserCloudSurfLastNum = laserCloudSurfLast->points.size();
        kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
        kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
        frame_id ++;
    }
    else{
        // 使用局部地图进行配准
        kdtree_local_map->setInputCloud(input_target_);
        Mode = 0;
    }
    return true;
}

bool ALOAMRegistration::ScanMatch(
    const CloudData::CLOUD_PTR& input_source, 
    const Eigen::Matrix4f& predict_pose, 
    CloudData::CLOUD_PTR& result_cloud_ptr,
    Eigen::Matrix4f& result_pose
) {
    // scan to scan 模式
    if(Mode){
        frame_id ++;
        return ScanToScan(input_source,
                        predict_pose,
                        result_cloud_ptr,
                        result_pose);
    }
    else{
        frame_id ++;
        return ScanToMap(input_source,
                        predict_pose,
                        result_cloud_ptr,
                        result_pose);
    }

}

bool ALOAMRegistration::ScanToMap(const CloudData::CLOUD_PTR& input_source, 
                  const Eigen::Matrix4f& predict_pose, 
                  CloudData::CLOUD_PTR& result_cloud_ptr,
                  Eigen::Matrix4f& result_pose){
    
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;


    CloudData::CLOUD_PTR input_source_ = input_source;
    CloudData::CLOUD_PTR transformed_input_source(new CloudData::CLOUD());
    pcl::transformPointCloud(*input_source_, *transformed_input_source, predict_pose);

    ExtractCornerandFlat(transformed_input_source);

    int cornerPointsSharpNum = cornerPointsSharp->points.size();
    int surfPointsFlatNum = surfPointsFlat->points.size();

    Sophus::SE3d pose_se3;
    pose_se3.setRotationMatrix(Eigen::Matrix3d::Identity());
    pose_se3.trans(Eigen::Vector3d::Zero());
    Eigen::Matrix<double, 6, 1> pose_vec = pose_se3.log();
    sophus_param[0] = pose_vec(0, 0);
    sophus_param[1] = pose_vec(1, 0);
    sophus_param[2] = pose_vec(2, 0);
    sophus_param[3] = pose_vec(3, 0);
    sophus_param[4] = pose_vec(4, 0);
    sophus_param[5] = pose_vec(5, 0);


     for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)     // 循环两次计算
    {

        // 鲁棒核函数
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.05);

        ceres::Problem::Options problem_options;

        ceres::Problem problem(problem_options);

        problem.AddParameterBlock(sophus_param, 6);
        problem.SetParameterization(sophus_param, new PoseSE3Parameterization());

        pcl::PointXYZ pointSel;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        int corner_num = 0;

        // find correspondence for corner features
        // 计算点特征
        for (int i = 0; i < cornerPointsSharpNum; ++i)
        {
            pointSel.x = cornerPointsSharp->points[i].x;
            pointSel.y = cornerPointsSharp->points[i].y;
            pointSel.z = cornerPointsSharp->points[i].z;

            kdtree_local_map->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            if (pointSearchSqDis[4] < 1.0)
            { 
                std::vector<Eigen::Vector3d> nearCorners;
                Eigen::Vector3d center(0, 0, 0);
                for (int j = 0; j < 5; j++)
                {
                    Eigen::Vector3d tmp(input_target_->points[pointSearchInd[j]].x,
                                        input_target_->points[pointSearchInd[j]].y,
                                        input_target_->points[pointSearchInd[j]].z);
                    center = center + tmp;
                    nearCorners.push_back(tmp);
                }
                center = center / 5.0;

                Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
                for (int j = 0; j < 5; j++)
                {
                    Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                    covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
                }

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

                // if is indeed line feature
                // note Eigen library sort eigenvalues in increasing order
                Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
                Eigen::Vector3d curr_point(pointSel.x, pointSel.y, pointSel.z);
                if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
                { 
                    Eigen::Vector3d point_on_line = center;
                    Eigen::Vector3d point_a, point_b;
                    point_a = 0.1 * unit_direction + point_on_line;
                    point_b = -0.1 * unit_direction + point_on_line;

                    ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b);
                    problem.AddResidualBlock(cost_function, loss_function, sophus_param);
                    corner_num++;	
                }							
            }

        }

        // 计算点到平面的误差
        // find correspondence for plane features
        int surf_num = 0;
        for (int i = 0; i < surfPointsFlatNum; ++i)
        {   pointSel.x = surfPointsFlat->points[i].x;
            pointSel.y = surfPointsFlat->points[i].y;
            pointSel.z = surfPointsFlat->points[i].z;

            kdtree_local_map->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            Eigen::Matrix<double, 5, 3> matA0;
            Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
            if (pointSearchSqDis[4] < 1.0)
            {
                
                for (int j = 0; j < 5; j++)
                {
                    matA0(j, 0) = input_target_->points[pointSearchInd[j]].x;
                    matA0(j, 1) = input_target_->points[pointSearchInd[j]].y;
                    matA0(j, 2) = input_target_->points[pointSearchInd[j]].z;
                    //printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
                }
                // find the norm of plane
                Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                double negative_OA_dot_norm = 1 / norm.norm();
                norm.normalize();

                // Here n(pa, pb, pc) is unit norm of plane
                bool planeValid = true;
                for (int j = 0; j < 5; j++)
                {
                    // if OX * n > 0.2, then plane is not fit well
                    if (fabs(norm(0) * input_target_->points[pointSearchInd[j]].x +
                                norm(1) * input_target_->points[pointSearchInd[j]].y +
                                norm(2) * input_target_->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                    {
                        planeValid = false;
                        break;
                    }
                }
                Eigen::Vector3d curr_point(pointSel.x, pointSel.y, pointSel.z);
                if (planeValid)
                {
                    ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
                    problem.AddResidualBlock(cost_function, loss_function,sophus_param );
                    surf_num++;
                }
            }
            
        }
        if ((corner_num +surf_num) < 10)
        {
                printf("less correspondence! *************************************************\n");
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 4;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        
    }


    Eigen::Matrix4f transformation;
    Eigen::Matrix<double, 6, 1> vec;
    vec(0, 0) = sophus_param[0];
    vec(1, 0) = sophus_param[1];
    vec(2, 0) = sophus_param[2];
    vec(3, 0) = sophus_param[3];
    vec(4, 0) = sophus_param[4];
    vec(5, 0) = sophus_param[5];
    transformation = Sophus::SE3d::exp(vec).matrix().cast<float>();

    Eigen::Matrix3f R = transformation.block<3,3>(0,0);
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(R,Eigen::ComputeThinU | Eigen::ComputeThinV);
    R = svd.matrixU()*svd.matrixV().transpose();
    transformation.block<3,3>(0,0) = R;

    result_pose = transformation * predict_pose;



    pcl::transformPointCloud(*input_source_, *result_cloud_ptr, result_pose);

    pcl::transformPointCloud(*cornerPointsLessSharp, *cornerPointsLessSharp, transformation);
    pcl::transformPointCloud(*surfPointsLessFlat, *surfPointsLessFlat, transformation);

    
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTemp = cornerPointsLessSharp;
    cornerPointsLessSharp = laserCloudCornerLast;
    laserCloudCornerLast = laserCloudTemp;

    laserCloudTemp = surfPointsLessFlat;
    surfPointsLessFlat = laserCloudSurfLast;
    laserCloudSurfLast = laserCloudTemp;

    laserCloudCornerLastNum = laserCloudCornerLast->points.size();
    laserCloudSurfLastNum = laserCloudSurfLast->points.size();
    kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
    kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

    Mode = 1;

}

bool ALOAMRegistration::ScanToScan(const CloudData::CLOUD_PTR& input_source, 
                  const Eigen::Matrix4f& predict_pose, 
                  CloudData::CLOUD_PTR& result_cloud_ptr,
                  Eigen::Matrix4f& result_pose){

    CloudData::CLOUD_PTR input_source_ = input_source;
    CloudData::CLOUD_PTR transformed_input_source(new CloudData::CLOUD());
    pcl::transformPointCloud(*input_source_, *transformed_input_source, predict_pose);

    ExtractCornerandFlat(transformed_input_source);

    int cornerPointsSharpNum = cornerPointsSharp->points.size();
    int surfPointsFlatNum = surfPointsFlat->points.size();


    Sophus::SE3d pose_se3;
    pose_se3.setRotationMatrix(Eigen::Matrix3d::Identity());
    pose_se3.trans(Eigen::Vector3d::Zero());
    Eigen::Matrix<double, 6, 1> pose_vec = pose_se3.log();
    sophus_param[0] = pose_vec(0, 0);
    sophus_param[1] = pose_vec(1, 0);
    sophus_param[2] = pose_vec(2, 0);
    sophus_param[3] = pose_vec(3, 0);
    sophus_param[4] = pose_vec(4, 0);
    sophus_param[5] = pose_vec(5, 0);
    
     for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)     // 循环两次计算
    {
        corner_correspondence = 0;
        plane_correspondence = 0;

        // 鲁棒核函数
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.01);

        ceres::Problem::Options problem_options;

        ceres::Problem problem(problem_options);

        problem.AddParameterBlock(sophus_param, 6);
        problem.SetParameterization(sophus_param, new PoseSE3Parameterization());

        pcl::PointXYZI pointSel;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // find correspondence for corner features
        // 计算点特征
        for (int i = 0; i < cornerPointsSharpNum; ++i)
        {
            // 将激光点云转换到当前帧的起始坐标系
            pointSel = cornerPointsSharp->points[i];
            kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

            int closestPointInd = -1, minPointInd2 = -1;
            if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
            {
                closestPointInd = pointSearchInd[0];    // 最近点
                int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);  // 判断最近点的层数

                double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
                // search in the direction of increasing scan line
                // 在前一层和后一层寻找特征点
                for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j)
                {
                    // if in the same scan line, continue
                    if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)
                        continue;

                    // if not in nearby scans, end the loop
                    if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                        break;

                    double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                            (laserCloudCornerLast->points[j].x - pointSel.x) +
                                        (laserCloudCornerLast->points[j].y - pointSel.y) *
                                            (laserCloudCornerLast->points[j].y - pointSel.y) +
                                        (laserCloudCornerLast->points[j].z - pointSel.z) *
                                            (laserCloudCornerLast->points[j].z - pointSel.z);

                    if (pointSqDis < minPointSqDis2)
                    {
                        // find nearer point
                        minPointSqDis2 = pointSqDis;
                        minPointInd2 = j;
                    }
                }

                // search in the direction of decreasing scan line
                for (int j = closestPointInd - 1; j >= 0; --j)
                {
                    // if in the same scan line, continue
                    if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)
                        continue;

                    // if not in nearby scans, end the loop
                    if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                        break;

                    double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                            (laserCloudCornerLast->points[j].x - pointSel.x) +
                                        (laserCloudCornerLast->points[j].y - pointSel.y) *
                                            (laserCloudCornerLast->points[j].y - pointSel.y) +
                                        (laserCloudCornerLast->points[j].z - pointSel.z) *
                                            (laserCloudCornerLast->points[j].z - pointSel.z);

                    if (pointSqDis < minPointSqDis2)
                    {
                        // find nearer point
                        minPointSqDis2 = pointSqDis;
                        minPointInd2 = j;
                    }
                }
            }
            // 构建残差块
            if (minPointInd2 >= 0) // both closestPointInd and minPointInd2 is valid
            {
                Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                            cornerPointsSharp->points[i].y,
                                            cornerPointsSharp->points[i].z);
                Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
                                                laserCloudCornerLast->points[closestPointInd].y,
                                                laserCloudCornerLast->points[closestPointInd].z);
                Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
                                                laserCloudCornerLast->points[minPointInd2].y,
                                                laserCloudCornerLast->points[minPointInd2].z);

                // 构建误差函数
                ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b);

                // ceres::CostFunction *cost_function = new SophusLidarEdgeFactor(curr_point, last_point_a, last_point_b);
                problem.AddResidualBlock(cost_function, loss_function, sophus_param);
                corner_correspondence++;    // transformation.setIdentity();
              
            }
        }

        // 计算点到平面的误差
        // find correspondence for plane features
        for (int i = 0; i < surfPointsFlatNum; ++i)
        {   pointSel = surfPointsFlat->points[i];
            kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

            int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
            if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
            {
                closestPointInd = pointSearchInd[0];

                // get closest point's scan ID
                int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
                double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

                // search in the direction of increasing scan line
                for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j)
                {
                    // if not in nearby scans, end the loop
                    if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                        break;

                    double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                            (laserCloudSurfLast->points[j].x - pointSel.x) +
                                        (laserCloudSurfLast->points[j].y - pointSel.y) *
                                            (laserCloudSurfLast->points[j].y - pointSel.y) +
                                        (laserCloudSurfLast->points[j].z - pointSel.z) *
                                            (laserCloudSurfLast->points[j].z - pointSel.z);

                    // if in the same or lower scan line
                    if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                    {
                        minPointSqDis2 = pointSqDis;
                        minPointInd2 = j;
                    }
                    // if in the higher scan line
                    else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                    {
                        minPointSqDis3 = pointSqDis;
                        minPointInd3 = j;
                    }
                }

                // search in the direction of decreasing scan line
                for (int j = closestPointInd - 1; j >= 0; --j)
                {
                    // if not in nearby scans, end the loop
                    if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                        break;

                    double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                            (laserCloudSurfLast->points[j].x - pointSel.x) +
                                        (laserCloudSurfLast->points[j].y - pointSel.y) *
                                            (laserCloudSurfLast->points[j].y - pointSel.y) +
                                        (laserCloudSurfLast->points[j].z - pointSel.z) *
                                            (laserCloudSurfLast->points[j].z - pointSel.z);

                    // if in the same or higher scan line
                    if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                    {
                        minPointSqDis2 = pointSqDis;
                        minPointInd2 = j;
                    }
                    else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                    {
                        // find nearer point
                        minPointSqDis3 = pointSqDis;
                        minPointInd3 = j;
                    }
                }

                if (minPointInd2 >= 0 && minPointInd3 >= 0)
                {

                    Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                                surfPointsFlat->points[i].y,
                                                surfPointsFlat->points[i].z);
                    Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                                    laserCloudSurfLast->points[closestPointInd].y,
                                                    laserCloudSurfLast->points[closestPointInd].z);
                    Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                                    laserCloudSurfLast->points[minPointInd2].y,
                                                    laserCloudSurfLast->points[minPointInd2].z);
                    Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                                    laserCloudSurfLast->points[minPointInd3].y,
                                                    laserCloudSurfLast->points[minPointInd3].z);

                    // 构建残差块
                    ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c);
                    // ceres::CostFunction *cost_function =  new SophusLidarPlaneFactor(curr_point, last_point_a, last_point_b, last_point_c);
                    problem.AddResidualBlock(cost_function, loss_function, sophus_param);

                    plane_correspondence++;
                }
            }
        }
        if ((corner_correspondence + plane_correspondence) < 10)
        {
                printf("less correspondence! *************************************************\n");
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 4;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        
    }
    
    
    Eigen::Matrix4f transformation;
    Eigen::Matrix<double, 6, 1> vec;
    vec(0, 0) = sophus_param[0];
    vec(1, 0) = sophus_param[1];
    vec(2, 0) = sophus_param[2];
    vec(3, 0) = sophus_param[3];
    vec(4, 0) = sophus_param[4];
    vec(5, 0) = sophus_param[5];
    transformation = Sophus::SE3d::exp(vec).matrix().cast<float>();

    Eigen::Matrix3f R = transformation.block<3,3>(0,0);
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(R,Eigen::ComputeThinU | Eigen::ComputeThinV);
    R = svd.matrixU()*svd.matrixV().transpose();
    transformation.block<3,3>(0,0) = R;
    

    result_pose = transformation * predict_pose;

    pcl::transformPointCloud(*input_source_, *result_cloud_ptr, result_pose);
// 

    pcl::transformPointCloud(*cornerPointsLessSharp, *cornerPointsLessSharp, transformation);
    pcl::transformPointCloud(*surfPointsLessFlat, *surfPointsLessFlat, transformation);
// 

    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTemp = cornerPointsLessSharp;
    cornerPointsLessSharp = laserCloudCornerLast;
    laserCloudCornerLast = laserCloudTemp;

    laserCloudTemp = surfPointsLessFlat;
    surfPointsLessFlat = laserCloudSurfLast;
    laserCloudSurfLast = laserCloudTemp;

    laserCloudCornerLastNum = laserCloudCornerLast->points.size();
    laserCloudSurfLastNum = laserCloudSurfLast->points.size();
    kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
    kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

    Mode = 1;
    return true;
       
}


void ALOAMRegistration::removeClosedPointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
                              pcl::PointCloud<pcl::PointXYZ> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

// 提取线特征和面特征
bool ALOAMRegistration::ExtractCornerandFlat(const CloudData::CLOUD_PTR& input){
    

    // 保存对应线数所对应的点的id
    std::vector<int> scanStartInd(scan_line_, 0);
    std::vector<int> scanEndInd(scan_line_, 0);

    std::vector<int> indices;

    pcl::PointCloud<pcl::PointXYZ> laserCloudIn = *input;
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    removeClosedPointCloud(laserCloudIn, laserCloudIn, 0.1); // 移除距离太近的点，这个值后期加入参数列表

    int cloudSize = laserCloudIn.points.size();
    int count = cloudSize;
    pcl::PointXYZI point;

    // 用来记录每个点云属于的激光束
    std::vector<pcl::PointCloud<pcl::PointXYZI>> laserCloudScans(scan_line_);

    // 遍历每个点
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;

        // 计算每个激光点的俯仰角
        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

        // 根据不同的雷达结构判断激光点属于哪一层激光
        if (scan_line_ == 16)
        {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (scan_line_ - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (scan_line_ == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (scan_line_ - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (scan_line_ == 64)
        {   
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = scan_line_ / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies 
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else
        {
            printf("wrong scan number\n");
            ROS_BREAK();
        }
        point.intensity = scanID;
        laserCloudScans[scanID].push_back(point); 
    }

    // 有效点数量
    cloudSize = count;
    //printf("points size %d \n", cloudSize);

    // 对每一束激光剔除前5个点和倒数5个点
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
    for (int i = 0; i < scan_line_; i++)
    { 
        scanStartInd[i] = laserCloud->size() + 5;       // 标记有效起点
        // 将分束的激光加入新的激光点云中
        *laserCloud += laserCloudScans[i];
        scanEndInd[i] = laserCloud->size() - 6;         // 标记有效终点
    }

    // 这里的计算没有考虑无效的点，无效点使用上面的标记来剔除
    for (int i = 5; i < cloudSize - 5; i++)
    { 
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;      // 曲率
        cloudSortInd[i] = i;                                                    // index
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
    }

    pcl::PointCloud<pcl::PointXYZI> cornerPointsSharp_;
    pcl::PointCloud<pcl::PointXYZI> cornerPointsLessSharp_;
    pcl::PointCloud<pcl::PointXYZI> surfPointsFlat_;
    pcl::PointCloud<pcl::PointXYZI> surfPointsLessFlat_;

    // 根据曲率将激光电分类
    for (int i = 0; i < scan_line_; i++)
    {
        if( scanEndInd[i] - scanStartInd[i] < 6)        // 代表这一层的有效激光点太少
            continue;
        pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZI>);
        for (int j = 0; j < 6; j++)                     // 将一束的激光点均匀分成6块区域用于均匀的提取特征点
        {
            // 每一块区域的起始点和终点
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; 
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            // 根据曲率对索引进行排序
            std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);

            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--)                  // 根据曲率从大到小遍历点云
            {
                int ind = cloudSortInd[k]; 

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 0.1)              // 如果曲率大于0.1，且未被选择过
                {

                    largestPickedNum++;
                    if (largestPickedNum <= 2)              // 曲率最大的两个点为cornerpointsharp
                    {                        
                        cloudLabel[ind] = 2;
                        cornerPointsSharp_.push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp_.push_back(laserCloud->points[ind]);
                    }
                    else if (largestPickedNum <= 20)        // 曲率前20大的作为cornerpointslesssharp
                    {                        
                        cloudLabel[ind] = 1; 
                        cornerPointsLessSharp_.push_back(laserCloud->points[ind]);
                    }
                    else
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;            // 表示这个点已经被选取过
                    
                    // 对周围的曲率小于0.05的点也标记为已经被选取过
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            // 选取平面点
            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++)                          // 从小到达遍历
            {
                int ind = cloudSortInd[k];                          

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1)
                {

                    cloudLabel[ind] = -1; 
                    surfPointsFlat_.push_back(laserCloud->points[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)                     // 选取曲率最小的四个点作为surfpointsflat
                    { 
                        break;
                    }

                    // 标记周围的点
                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    { 
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            // 剩下的所有点全部作为surfpointslessflat
            for (int k = sp; k <= ep; k++)
            {
                if (cloudLabel[k] <= 0)
                {
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }

        pcl::PointCloud<pcl::PointXYZI> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;

        // 对lesssurfplat下采样，因为点太多
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        surfPointsLessFlat_ += surfPointsLessFlatScanDS;
        //surfPointsLessFlat_ += *surfPointsLessFlatScan;
        

    }
    cornerPointsLessSharp->clear();
    *cornerPointsLessSharp = cornerPointsLessSharp_;

    cornerPointsSharp->clear();
    *cornerPointsSharp = cornerPointsSharp_;

    surfPointsFlat->clear();
    *surfPointsFlat = surfPointsFlat_;

    surfPointsLessFlat->clear();
    *surfPointsLessFlat = surfPointsLessFlat_;


    //std::cout << "sharp:" << cornerPointsSharp->points.size() << "less shape" << cornerPointsLessSharp->points.size() << std::endl;
    //std::cout << "flat:" << surfPointsFlat->points.size() << "less flat" << surfPointsLessFlat->points.size() << std::endl;
    return true;

}


}
