# 多传感器融合定位 第二章 3D激光里程计

参考博客：

[深蓝学院 《多传感器融合定位》 第1章作业](https://blog.csdn.net/hitljy/article/details/109227799?spm=1001.2014.3001.5501)

参考代码：

[Geyao 前端里程计代码](https://github.com/AlexGeControl/Sensor-Fusion/tree/master/workspace/assignments/01-lidar-odometry)

代码下载 ： [https://github.com/kahowang/sensor-fusion-for-localization-and-mapping]( https://github.com/kahowang/sensor-fusion-for-localization-and-mapping)

## 点云地图构建框架

![框架](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11%E6%A1%86%E6%9E%B6.png)

FILE: front_end_flow.cpp

### FrontEndFlow::Run()

```cpp
bool FrontEndFlow::Run() {
    if (!ReadData()) {        //  获取来自subscribe的原始数据存储在std::deque中，同步各传感器数据
        return false;
    }
        
    if (!InitCalibration())  {     // 获取 lidar_to_imu_  坐标变换矩阵
        return false;   
    }


    if (!InitGNSS()) {                 //  初始化GNSS 数据
        return false; 
    }

    while(HasData()) {           
        if (!ValidData()) {
            continue;
        }
            
        UpdateGNSSOdometry();      //  更新GNSS 位置，并通过lidar_to_imu_ 变换到雷达坐标系下
        if (UpdateLaserOdometry()) {    //  更新激光里程计
            PublishData();
            SaveTrajectory();
        } else {
            LOG(INFO) << "UpdateLaserOdometry failed!" << std::endl;
        }
    }

    return true;
}
```

###  FrontEndFlow::UpdateLaserOdometry()

```cpp
bool FrontEndFlow::UpdateLaserOdometry() {
    static bool front_end_pose_inited = false;

    if (!front_end_pose_inited) {
        front_end_pose_inited = true;
        front_end_ptr_->SetInitPose(gnss_odometry_);
    }

    laser_odometry_ = Eigen::Matrix4f::Identity();
    return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
}
```

###   FrontEnd::Update()

更新里程计

```cpp
bool FrontEnd::Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose) {
    current_frame_.cloud_data.time = cloud_data.time;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, indices);

    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());    
    frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr, filtered_cloud_ptr);    //  滤波，选用 pcl::VoxelGrid<CloudData::POINT>

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = init_pose_;

    // 局部地图容器中没有关键帧，代表是第一帧数据
    // 此时把当前帧数据作为第一个关键帧，并更新局部地图容器和全局地图容器
    if (local_map_frames_.size() == 0) {
        current_frame_.pose = init_pose_;        // 局部地图第一帧作为predict_pose 先验用
        UpdateWithNewFrame(current_frame_);
        cloud_pose = current_frame_.pose;
        return true;
    }

    // 不是第一帧，就正常匹配
    registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr_, current_frame_.pose);
    cloud_pose = current_frame_.pose;

    // 更新相邻两帧的相对运动
    step_pose = last_pose.inverse() * current_frame_.pose;
    predict_pose = current_frame_.pose * step_pose;     // 更新预测位姿
    last_pose = current_frame_.pose;

    // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
    if (fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) + 
        fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) +
        fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3)) > key_frame_distance_) {
        UpdateWithNewFrame(current_frame_);
        last_key_frame_pose = current_frame_.pose;
    }

    return true;
}
```

## ICP_SVD

### Funtion

FILE:  icp_svd_registration.cpp

#### scanMatch()

```cpp
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
```

#### GetCorrespondence()

通过kdtree 寻找两片点云的匹配点

```cpp
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
```

#### GetTransform()

公式：

![svd_R](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11svd_R.png)

![svd_t](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11svd_t.png)

通过Eigen svd求解 R t， 

注意：求出的旋转矩阵必须满足是正交阵并且行列式为1，因此，需要对求出的旋转矩阵进行正交化.

1.旋转矩阵转四元数，对四元数进行归一化。

2.SO3流形投影。

3.SVD分解奇异值置1。

```cpp
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
```

###   SVD_ICP参数配置 

FILE：front_end/config.yaml

trick: 参数设置，icp_svd 主要修改 

max_corr_dist:  SVD_ICP 的精度比较依赖与临近点对的准确度，比如将临近点对的距离阈值设置尽可能小，比如0.5

max_iter:  一般来说，迭代次数越多越好，当然必然会徒增不必要的算力负担，因为ICP_SVD是一步求解，所以迭代次数可以尽不用太大。

```bash
ICP_SVD:
    max_corr_dist : 0.5
    trans_eps : 0.01
    euc_fitness_eps : 0.36
    max_iter : 10
```

### ICP补充

经陈助教的指点，在进行ICP_SVD求解时，求出的旋转矩阵，可以进行reflection(旋转反射的判断)，KITTI场景中，数据集可能没有点云反射的现象，但是工程中，对求出的R阵加入reflection的判断，可以提升激光里程计ICP的精度。当出现需反射问题时：det(VU.tranpose()) = -1 ，正确的R阵为： det(VU.tranpose()) = 1。因此，我们可以将R阵改为如下通式：

![image-20220424175530082](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11image-20220424175530082.png) 

参考论文：Least-Squares Rigid Motion Using SVD

参考博客：[雷达系列论文翻译（二）：Least-Squares Rigid Motion Using SVD](https://blog.csdn.net/weixin_39061796/article/details/119861178)



## NDT_CPU

参考源码： [autoware  ndt_cpu](https://github.com/Autoware-AI/core_perception/tree/master/ndt_cpu)

[NDT 公式推导及源码解析（1）](https://blog.csdn.net/u013794793/article/details/89306901)

[NDT 公式推导及源码解析（2）](https://blog.csdn.net/u013794793/article/details/89321792)

![ndt_cpu](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11ndt_cpu.png)

FILE:  include/models/ndt_cpu/ndt_cpu_registration.hpp

```cpp
#ifndef  LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_CPU_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_CPU_REGISTRATION_HPP_

#include "lidar_localization/models/registration/registration_interface.hpp"
#include "lidar_localization/models/registration/ndt_cpu/NormalDistributionsTransform.h"

namespace lidar_localization {

class NDTCPURegistration: public RegistrationInterface {     // 继承点云配准的基类
  public:
    NDTCPURegistration(const YAML::Node&  node);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(
      const CloudData::CLOUD_PTR& input_source, 
      const Eigen::Matrix4f& predict_pose, 
      CloudData::CLOUD_PTR& result_cloud_ptr,
      Eigen::Matrix4f& result_pose
    ) override;

   private:
    bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

  private:
    cpu::NormalDistributionsTransform<CloudData::POINT,   CloudData::POINT>  ndt_cpu_;      // 实例化cpu_ndt 对象

};

} // namespace lidar_localization

#endif
```

FILE:  src/models/registration/ndt_cpu/ndt_cpu_registration.cpp

```cpp
/*
 * @Description: NDT CPU lidar odometry
 * @Author: KaHo
 * @Date: 2021-8-22 
 */

#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include "glog/logging.h"

#include "lidar_localization/models/registration/ndt_cpu/ndt_cpu_registration.hpp"

namespace  lidar_localization{

NDTCPURegistration::NDTCPURegistration(const YAML::Node& node){
    float res = node["res"].as<float>();
    float step_size = node["step_size"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

bool NDTCPURegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter) {
    ndt_cpu_.setResolution(res);
    ndt_cpu_.setStepSize(step_size);
    ndt_cpu_.setTransformationEpsilon(trans_eps);
    ndt_cpu_.setMaximumIterations(max_iter);

    LOG(INFO) << "NDT params:" << std::endl
              << "res: " << res << ", "
              << "step_size: " << step_size << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter 
              << std::endl << std::endl;

    return true;
}

bool NDTCPURegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    ndt_cpu_.setInputTarget(input_target);

    return true;
}

bool NDTCPURegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    ndt_cpu_.setInputSource(input_source);
    ndt_cpu_.align(*result_cloud_ptr, predict_pose);
    result_pose = ndt_cpu_.getFinalTransformation();    // 匹配后的点云

    return true;
}

}
```

NDT_CPU 参数配置

```bash
NDT_CPU:
    res : 0.8                 # volex  resolution
    step_size : 0.1    # 梯度下降的步长，越大下降越快，但是容易over shoot陷入局部最优
    trans_eps : 0.01    # 最大容差，一旦两次转换矩阵小于 trans_eps  退出迭代
    max_iter : 30         #   最大迭代次数
```



## ICP_PCL

![icp_0](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11icp_0.png)

FILE:  icp_registration.cpp

```cpp
  private:
    pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>::Ptr icp_ptr_;
```

```cpp
bool ICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    icp_ptr_->setInputTarget(input_target);     

    return true;
}
```

```cpp
bool ICPRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    icp_ptr_->setInputSource(input_source);          //  输入待配准点云
    icp_ptr_->align(*result_cloud_ptr, predict_pose);      // 配准              
    result_pose = icp_ptr_->getFinalTransformation();    // 获取变换矩阵 

    return true;
}
```

##    NDT_PCL

![ndt_pcl 0](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11ndt_pcl%200.png)

FILE:  ndt_registration.cpp

```cpp
  private:
    pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;
```

```cpp
bool NDTRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    ndt_ptr_->setInputTarget(input_target);

    return true;
}
```

```cpp
bool NDTRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    ndt_ptr_->setInputSource(input_source);
    ndt_ptr_->align(*result_cloud_ptr, predict_pose);
    result_pose = ndt_ptr_->getFinalTransformation();

    return true;
}
```



## Running

### 参数配置选择

lidar_localization/config/front_end/config.yaml  选择点云匹配方式和指定数据存放路径

```bash
data_path: /home/x/catkin_ws/src/lidar_localization   # 数据存放路径

# 匹配
# TODO: implement your custom registration method and add it here
registration_method: ICP_SVD   # 选择点云匹配方法，目前支持：ICP, ICP_SVD, NDT, SICP

# 局部地图
key_frame_distance: 2.0 # 关键帧距离
local_frame_num: 20
local_map_filter: voxel_filter # 选择滑窗地图点云滤波方法，目前支持：voxel_filter

# rviz显示
display_filter: voxel_filter # rviz 实时显示点云时滤波方法，目前支持：voxel_filter

# 当前帧
frame_filter: voxel_filter # 选择当前帧点云滤波方法，目前支持：voxel_filter

## 滤波相关参数
voxel_filter:
    local_map:
        leaf_size: [0.6, 0.6, 0.6]
    frame:
        leaf_size: [1.3, 1.3, 1.3]
    display:
        leaf_size: [0.5, 0.5, 0.5]

# 各配置选项对应参数
## 匹配相关参数
ICP:
    max_corr_dist : 1.2
    trans_eps : 0.01
    euc_fitness_eps : 0.36
    max_iter : 30
ICP_SVD:
    max_corr_dist : 0.5
    trans_eps : 0.01
    euc_fitness_eps : 0.36
    max_iter : 10
NDT:
    res : 1.0
    step_size : 0.1
    trans_eps : 0.01
    max_iter : 30
SICP:
    p : 1.0
    mu : 10.0
    alpha : 1.2
    max_mu : 1e5
    max_icp : 100
    max_outer : 100
    max_inner : 1
    stop : 1e-5
```

### 运行建图,保存地图

```bash
roslaunch lidar_localization front_end.launch
```

```bash
rosbag  play kitti_lidar_only_2011_10_03_drive_0027_synced.bag
```

![1](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A111.png)

保存地图到 slam_data 文件夹下

```bash
rosservice call /save_map
```

### evo轨迹评估

lidar_localization/slam_data/trajectory 路径下有 ground_truth.txt   laser_odom.txt 两个文件

#### 下载evo

```bash
pip install evo --upgrade --no-binary evo
```

EVO评价数据有两种模式，对应的指令分别是 evo_rpe 和 evo_ape ，前者评价的是每段距离内的误差，后者评价的是绝对误差随路程的累计。

#### evo_rpe

评价每段距离内的误差可以使用如下指令

```bash
evo_rpe kitti ground_truth.txt laser_odom.txt -r trans_part --delta 100 --plot --plot_mode xyz
```

其中–delta 100表示的是每隔100米统计一次误差，这样统计的其实就是误差的百分比，和kitti的odometry榜单中的距离误差指标就可以直接对应了。

#### evo_ape

评价总累计误差可以用如下指令

```bash
evo_ape kitti ground_truth.txt laser_odom.txt -r full --plot --plot_mode xyz
```

#### 各种算法评估结果

##### ICP_SVD

###### evo_rpe

![rpe1](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11rpe1.png)

<img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11rpe2.png" alt="rpe2" style="zoom:50%;" />

<img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11rpe3.png" alt="rpe3" style="zoom:50%;" />

###### evo_ape

![ape1](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11ape1.png)

<img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11ape2.png" alt="ape2" style="zoom:50%;" />

<img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11ape3.png" alt="ape3" style="zoom:50%;" />

##### NDT_CPU

###### evo_rpe

<img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11ndt_cpu_rpe_1.png" alt="ndt_cpu_rpe_1"  />

<img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11ndt_cpu_rpe_2.png" alt="ndt_cpu_rpe_2" style="zoom:67%;" />

<img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11ndt_cpu_rpe_3.png" alt="ndt_cpu_rpe_3" style="zoom:67%;" />

###### evo_ape

![ndt_pcl_ape 1](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11ndt_pcl_ape%201.png)

<img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11ndt_pcl_ape%202.png" alt="ndt_pcl_ape 2" style="zoom:67%;" />

<img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11ndt_pcl_ape%203.png" alt="ndt_pcl_ape 3" style="zoom:67%;" />

##### ICP_PCL

###### evo_rpe

![icp_pcl rpe 1](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11icp_pcl%20rpe%201.png)

<img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11icp_pcl%20rpe%202.png" alt="icp_pcl rpe 2" style="zoom:67%;" />

<img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11icp_pcl%20rpe%203.png" alt="icp_pcl rpe 3" style="zoom:67%;" />

###### evo_ape

![icp_pcl ape 1](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11icp_pcl%20ape%201.png)

<img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11icp_pcl%20ape%202.png" alt="icp_pcl ape 2" style="zoom:67%;" />

<img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11icp_pcl%20ape%203.png" alt="icp_pcl ape 3" style="zoom:67%;" />

##### ICP_NDT

###### evo_rpe

![ndt_pcl_rpe 1](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11ndt_pcl_rpe%201.png)

<img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11ndt_pcl_rpe%202.png" alt="ndt_pcl_rpe 2" style="zoom:67%;" />

<img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11ndt_pcl_rpe%203.png" alt="ndt_pcl_rpe 3" style="zoom:67%;" />

###### evo_ape

![ndt_pcl_ape 1](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11ndt_pcl_ape%201.png)

<img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11ndt_pcl_ape%202.png" alt="ndt_pcl_ape 2" style="zoom:67%;" />

<img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11ndt_pcl_ape%203.png" alt="ndt_pcl_ape 3" style="zoom:67%;" />

​																									          edited by kaho  2021.8.17