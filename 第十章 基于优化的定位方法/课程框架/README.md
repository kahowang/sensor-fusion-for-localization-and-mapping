# Multi-Sensor Fusion for Localization & Mapping: Sliding Window -- 多传感器融合定位与建图: 基于图优化的定位方法

深蓝学院, 多传感器融合定位与建图, 第10章Graph Optimization for Localization through Sliding Window代码框架.

---

## Overview

本作业旨在加深对**基于图优化的定位, 滑动窗口, 方法**的理解.

补全基于滑动窗口的融合定位方法的实现, 并分别与:

* 不加融合
* EKF融合

的效果做对比.V

---

## Getting Started

### 环境检查: 确保Git Repo与使用的Docker Image均为最新

首先, 请确保选择了正确的branch **10-sliding-window**:

<img src="doc/images/branch-check.png" alt="Branch Check" width="100%">

执行以下命令，确保所使用的Git Repo与Docker Image均为最新:

```bash
# update git repo:
git pull
#
# update docker image:
#
# 1. first, login to Sensor Fusion registry -- default password is shenlansf20210122:
docker login --username=937570601@qq.com registry.cn-shanghai.aliyuncs.com
# 2. then download images:
docker pull registry.cn-shanghai.aliyuncs.com/shenlanxueyuan/sensor-fusion-workspace:bionic-cpu-vnc
```

### 及格要求: 补全代码，且功能正常

干就完了! 

**友情提示**

* 如果**理论知识**有较大欠缺, 推荐参考**深蓝学院VIO课程**中的相关内容

* 如果在**实践环节**感觉无从下手, 推荐参考:

    * Ceres官方文档
    * VINS Mono GitHub实现

启动Docker后, 打开浏览器, 进入Web Workspace. 启动Terminator, 将Shell的工作目录切换如下:

<img src="doc/images/terminator.png" alt="Terminator" width="100%">

在**上侧**的Shell中, 输入如下命令, **编译lidar_localization**. 如遇到错误, 且非首次编译, 请尝试执行**catkin clean**, 清理catkin cache.

```bash
# build:
catkin config --install && catkin build lidar_localization
# set up session:
source install/setup.bash
# launch:
roslaunch lidar_localization lio_localization.launch
```

在**下侧**的Shell中, 输入如下命令, **Play KITTI ROS Bag**. 如果机器的配置较低, 可以降低播放速率.

**注意**: 两个数据集均可用于完成课程, 对代码功能的运行没有任何影响, 区别在于第一个有Camera信息

```bash
# play ROS bag, full KITTI:
rosbag play kitti_2011_10_03_drive_0027_synced.bag
# play ROS bag, lidar-only KITTI:
rosbag play kitti_lidar_only_2011_10_03_drive_0027_synced.bag
```

在**运行结束后**, 可以执行如下的命令, **保存Estimated Odometry, 以用作evo评估**

```bash
# set up session:
source install/setup.bash
# save estimated odometry
rosservice call /save_odometry
```

成功后, 可以看到如下的RViz Visualization.

<img src="doc/images/demo.png" alt="LIO Localization Demo" width="100%">

**此Demo为参考答案的演示效果**. 若未实现:

1. 你将无法看到Demo的效果. 
2. 命令行中Ceres优化的Cost会始终为**零**

**请你尝试理解框架, 在其中补完相关逻辑, 实现基于图优化的定位**. **请务必清晰条理地, 在最终提交的报告中, 整理出你的实现逻辑**. 你的任务是自行实现精度尽可能高的解算方法. 期待你的精彩发挥!

请搜索TODO, 开始你的编码 :P. 

此处将完成作业相关的配置汇总如下:

* **IMU Pre-Integration** [here](src/lidar_localization/src/models/pre_integrator/imu_pre_integrator.cpp#L178)

* **Ceres Factors**
    * **Map Matching / GNSS Position Prior** [here]()
    * **IMU Pre-Integration** [here]()
    * **Lidar Odometry** [here]()
    * **Sliding Window Marginalization** [here]()

* **Module Hyper Params.**
    * **Sliding Window Config** [here](src/lidar_localization/config/matching/sliding_window.yaml)

### 良好要求: 实现功能的基础上，性能在部分路段比EKF有改善。

**备注**

1. 对比是全方位的, 既包括轨迹精度的对比, 也包括地图质量的对比(因为IMU会增加估计的平滑性)；
2. 由于数据集的老问题, 部分指标可能与预期不一致, 且地图质量无法量化, 因此给出自己的分析即可.

干就完了! 期待你的精彩发挥.

### 优秀要求: 由于基于滑窗的方法中，窗口长度对最终的性能有很大影响，请在良好的基础上，提供不同窗口长度下的融合结果，并对效果及原因做对比分析

如果理论知识有较大欠缺, 推荐参考深蓝学院VIO课程中的相关内容