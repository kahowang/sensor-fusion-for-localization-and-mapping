# 多传感器融合定位 (单天线gps+ndt匹配)初始化重定位

参考博客：[多传感器融合定位 第四章 点云地图构建及基于点云地图定位](https://blog.csdn.net/weixin_41281151/article/details/120116838)

代码下载：

最近工作比较繁忙，所以课程第九章预积分开始的内容，并没有跟上，在之后的日子里会慢慢跟上~

最近项目使用自己的数据集跑定位，目前使用的设备为 双天线RTK +  九轴IMU  +  16线Lidar

存在问题：

1.九轴的IMU磁场计受到较大的影响，导致姿态不准确，并且客户每次使用都需要校准，不太方便；

2.为了节约成本有时会使用单天线gps和六轴IMU，只能得到初始大概位置和基于上电时刻的相对姿态，不能得到导航系下的姿态； 

解决：

鉴于上述的实际问题，考虑采用 （gps 初始化position、PointCloud_Registration初始化的方法）进行重定位初始化

## 1.代码框架

整体代码框架为基于先验地图的定位，在 [多传感器融合定位 第七章 基于滤波的融合方法](https://blog.csdn.net/weixin_41281151/article/details/120904368)的代码框架基础上进行开发。

初始化位姿(POSE)包括两部分 ：  初始化位置(POSI)  初始化姿态(ORI)

### 1.1**初始化位置(POSI)**: 

使用GPS获得当前车体所在的经纬高，并转换为导航系下的导航系(XYZ)坐标，并获取当前所在位置的局部地图

### **1.2初始化姿态(ORI)**:

 当前点云地图 和 上一步获取的局部地图 进行点云配准，获取初始姿态角。 

### 1.3点云配准算法：

NDT、ICP、Loam 、 ScanContext 均可， 本次使用的配准算法为pcl_ndt

## 2.算法实现流程

当前的点云 m_current、局部地图的点云 m_local、ndt匹配的先验位姿 predict_pose、ndt匹配后的得分 fitness_score

``` shell
#step 1     获取当前的点云数据和gnss数据
#step 2    通过gnss返回的载体当前所在地图上的坐标,分割出局部地图的点云 m_local
#step 3    初始化ndt匹配的先验位姿 predict_pose 的 旋转部分 为单位阵、位置部分为step2得到的 posi
#step 3    将360度分成72份，每旋转5度为1份，predict_pose 每次绕Z轴旋转5度，输入到pcl_ndt中，进行 m_current  m_local 点云匹配,  并输出匹配后的得分 score
#step 4     选出得分最低的配准结果对应的旋转矩阵temp_init_rotation，作为定位初始姿态
```

trick： 进行72次(每5度)一次的旋转匹配中，发现正确的姿态和错误的姿态，在score上有较大的差异，如下表所示，为输入 predict_pose 旋转 5-360 ndt 匹配分数，正确的匹配角度为 60-65 之间，score分数对比其他错误姿态明显很小，为了加速初始化，可通过设置score阈值的方法进行加速初始化。 

| angle =  5.00037  score  =  8.01031<br/>angle =  10.0007  score  =  7.6049<br/>angle =  15.0011  score  =  6.99564<br/>angle =  20.0015  score  =  5.50515<br/>angle =  25.0018  score  =  5.58489<br/>angle =  30.0022  score  =  5.34696<br/>angle =  35.0026  score  =  4.43976<br/>angle =  40.0029  score  =  3.23621<br/>angle =  45.0033  score  =  2.3229<br/>angle =  50.0037  score  =  1.71253<br/>angle =  55.0041  score  =  0.800658<br/>angle =  60.0044  score  =  0.0609917<br/>angle =  65.0048  score  =  0.0612495<br/>angle =  70.0052  score  =  0.805328<br/>angle =  75.0055  score  =  1.18486<br/>angle =  80.0059  score  =  1.70853<br/>angle =  85.0063  score  =  2.08244<br/>angle =  90.0066  score  =  2.84847<br/>angle =  95.007  score  =  4.08729<br/>angle =  100.007  score  =  5.58512<br/>angle =  105.008  score  =  8.91464<br/>angle =  110.008  score  =  9.46592<br/>angle =  115.008  score  =  8.968<br/>angle =  120.009  score  =  10.8153 | angle =  125.009  score  =  12.5608<br/>angle =  130.01  score  =  11.598<br/>angle =  135.01  score  =  9.47127<br/>angle =  140.01  score  =  9.09879<br/>angle =  145.011  score  =  9.46195<br/>angle =  150.011  score  =  8.57565<br/><br/>angle =  155.011  score  =  8.03287<br/>angle =  160.012  score  =  7.2493<br/>angle =  165.012  score  =  7.13777<br/>angle =  170.013  score  =  5.51928<br/>angle =  175.013  score  =  4.88985<br/>angle =  180.013  score  =  3.738<br/>angle =  185.014  score  =  3.34093<br/>angle =  190.014  score  =  3.30719<br/>angle =  195.014  score  =  3.40997<br/>angle =  200.015  score  =  3.53268<br/>angle =  205.015  score  =  3.8749<br/>angle =  210.015  score  =  4.32289<br/>angle =  215.016  score  =  4.51662<br/>angle =  220.016  score  =  4.2599<br/>angle =  225.017  score  =  4.02596<br/>angle =  230.017  score  =  3.53537<br/>angle =  235.017  score  =  3.18032<br/>angle =  240.018  score  =  3.20526 | angle =  245.018  score  =  2.85211<br/>angle =  250.018  score  =  2.21396<br/>angle =  255.019  score  =  1.99666<br/>angle =  260.019  score  =  1.93678<br/>angle =  265.02  score  =  1.92775<br/>angle =  270.02  score  =  2.59312<br/>angle =  275.02  score  =  3.13421<br/>angle =  280.021  score  =  3.19898<br/>angle =  285.021  score  =  3.44148<br/>angle =  290.021  score  =  4.13718<br/>angle =  295.022  score  =  5.13398<br/>angle =  300.022  score  =  5.79304<br/>angle =  305.022  score  =  5.242<br/>angle =  310.023  score  =  5.00261<br/>angle =  315.023  score  =  3.70704<br/>angle =  320.024  score  =  3.63942<br/>angle =  325.024  score  =  3.96029<br/>angle =  330.024  score  =  4.7522<br/>angle =  335.025  score  =  5.44406<br/>angle =  340.025  score  =  5.60252<br/>angle =  345.025  score  =  5.92031<br/>angle =  350.026  score  =  7.90565<br/>angle =  355.026  score  =  7.74071<br/>angle =  360.027  score  =  8.39916<br/> |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |

## 3.核心代码

### 3.1 根据gps分割出的局部地图进行ndt配准，根据配准score 获取初始化姿态

FILE :  kitti_filtering.cpp

```cpp
bool KITTIFiltering::InitOri(Eigen::Matrix4f &init_pose,   CloudData &cloud_data ,  Eigen::Matrix3f &init_ori ){        //  use gnss and though ndt get  init_ori
      init_pose.block<3,3>(0,0)  =   Eigen::Matrix3f::Identity();       //   初始化旋转部分为单位阵
      Eigen::Matrix4f  predict_pose =  init_pose;      

      ResetLocalMap(predict_pose(0, 3), predict_pose(1, 3), predict_pose(2, 3));         //   根据gnss 的posi加载局部地图
       // remove invalid measurements:
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *cloud_data.cloud_ptr,
                              indices);
      // downsample:
      CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
      current_scan_filter_ptr_->Filter(cloud_data.cloud_ptr, filtered_cloud_ptr);

      Eigen::Matrix4f  temp_init_pose  =  Eigen::Matrix4f::Identity();     //  ndt 匹配得到的变换矩阵
       int iter_rate  =72;
       for(int i = 1; i <  iter_rate + 1;  i++){     //  分成72份
             double  angle =   2*M_PI /  72 * i;
             predict_pose  = init_pose; 
             Eigen::AngleAxisf rotation_vector(angle,  Eigen::Vector3f(0,0,1))  ;   // 绕z轴旋转  
             Eigen::Matrix3f    rotation_ =  rotation_vector.matrix();
            predict_pose.block<3,3>(0,0)  =  rotation_ *  predict_pose.block<3,3>(0,0);    

          // matching: 
            CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());    
            registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose,
                                        result_cloud_ptr, temp_init_pose);     
            
            float  score =   registration_ptr_->GetFitnessScore();         //  获取匹配分数
            std::cout  <<  "angle =  "<<  (angle * 57.3 ) << "  score  =  "   <<   score  <<std::endl; 
             if(score < 0.1){break;}   // 得分小于 0.1 跳出循环
       }
      predict_pose.block<3,3>(0,0)  =  temp_init_pose.block<3,3>(0,0);       // 获取配准质量高的姿态
     // matching: 
      CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());    
      registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose,
                                  result_cloud_ptr, temp_init_pose);     
      
      init_ori.block<3,3>(0,0)  =  temp_init_pose.block<3,3>(0,0);     //  更新init  pose的航向
}
```

### 3.2 获取第一帧gnss  和  point_cloud  数据，调用 init_ori 初始化

FILE:  kitti_filtering_flow.cpp

这里参考 KITTIFilteringFlow::InitCalibration()  这部分的写法，接受到第一帧数据后，进行阻塞匹配，匹配完成后，清空数据buff，舍弃匹配过程中所有数据，更新数据进行下一步的定位。

```cpp
bool  KITTIFilteringFlow::InitOrientation(){
     static bool InitOrientation_received = false;
     if(!InitOrientation_received){
          ReadData();      //  订阅传感器的话题，并将数据存储到buff 中
     }
     if ( (!InitOrientation_received) &&( HasData()) ) {
      //  use  pcl_ndt   registration  to  init  
      current_gnss_data_    =   gnss_data_buff_.front();        //   提取第一帧gnss数据，做位置初始化，提取local_map
      current_cloud_data_  =   cloud_data_buff_.front();      //  提取第一帧点云数据

      filtering_ptr_->InitOri(current_gnss_data_.pose,   current_cloud_data_, init_ori_);
      // std::cout << "init_ori_  "  <<  "\n"<< init_ori_  << std::endl;
      imu_raw_data_buff_.clear();     //    清空缓存数据
      cloud_data_buff_.clear();
      imu_synced_data_buff_.clear();
      pos_vel_data_buff_.clear();
      gnss_data_buff_.clear();
      InitOrientation_received  = true;
     std::cout << "------------------------------InitOrientation   SUCESS------------------------------"  << std::endl;
     }
    return  InitOrientation_received;
}
```

### 3.3  更新   current_gnss_data_.pose 为配准后的pose

FILE:  kitti_filtering_flow.cpp    KITTIFilteringFlow::InitLocalization(void)

```cpp
  current_gnss_data_.pose.block<3,3>(0,0)  =   init_ori_.block<3,3>(0,0);     //  修正姿态
```

### 3.4  调用 InitOrientation

FILE:  kitti_filtering_flow.cpp   KITTIFilteringFlow::Run()

```cpp
  if(!InitOrientation()){
    return  false;
  }
```

## 4. 实验结果

### 4.1 使用

```shell
roslaunch lidar_localization kitti_localization.launch
rosbag  play  kitti_lidar_only_2011_10_03_drive_0027_synced.bag
```

### 4.2 结果  

在不同的地方进行播放地图，大致可以完成初始化

#### 原点播放数据集

![2021-11-21 16-08-52 的屏幕截图](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E5%85%AD%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112021-11-21%2016-08-52%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)

#### 95s播放数据集合

![2021-11-21 16-13-59 的屏幕截图](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E5%85%AD%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112021-11-21%2016-13-59%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)

#### 200s播放数据集合

![2021-11-21 16-14-45 的屏幕截图](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E5%85%AD%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112021-11-21%2016-14-45%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)

#### 300s播放数据集合

![2021-11-21 16-15-28 的屏幕截图](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E5%85%AD%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112021-11-21%2016-15-28%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)

## 5.存在问题

 在播放100s的数据时发现，初始化结束后，定位跑飞了，原因在于播放100s数据集所在地方刚好在转角处(姿态改变方向的位置)，并且角度在230度附近，需要较长的初始化匹配时间，初始化成功后，当前载体的姿态已经换向和原来方向差异大 

![2021-11-21 16-22-56 的屏幕截图](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E5%85%AD%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112021-11-21%2016-22-56%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)

![2021-11-21 16-23-26 的屏幕截图](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E5%85%AD%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112021-11-21%2016-23-26%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)

后续优化方向：

1.优化遍历角度的方法，可采用二叉树的思路，加快遍历速度，进而提高初始化姿态的效率

2.尝试loam、scancontext等匹配方法，加快匹配效率。

​																																						     edited by kaho  11.21