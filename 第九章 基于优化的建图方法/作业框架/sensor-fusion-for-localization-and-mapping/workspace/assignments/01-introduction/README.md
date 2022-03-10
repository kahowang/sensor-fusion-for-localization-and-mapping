# Multi-Sensor Fusion for Localization & Mapping -- 多传感器融合定位与建图: Introduction

深蓝学院多传感器融合定位与建图第1节Environment Setup作业框架.

---

## Overview

本作业旨在引导您:

* 验证环境配置的正确性
* 熟悉课程教学框架

---

## Getting Started

启动Docker后, 打开浏览器, 前往localhost:40080, 进入Web Workspace. **若需要提高清晰度, 可以更改URL中的quality参数**. 启动Terminator, 将两个Shell的工作目录切换如下:

<img src="doc/terminator.png" alt="Terminator" width="100%">

在**上侧**的Shell中, 输入如下命令, **编译catkin_workspace**

```bash
# build
catkin config --install && catkin build
```

然后**启动解决方案**

```bash
# set up session:
source install/setup.bash
# launch:
roslaunch lidar_localization hello_kitti.launch
```

在**下侧**的Shell中, 输入如下命令, **Play KITTI ROS Bag**

```bash
# play ROS bag
rosbag play kitti_2011_10_03_drive_0027_synced.bag
```

成功后, 可以看到如下的RViz界面:

<img src="doc/demo.png" alt="KITTI Demo" width="100%">