# Multi-Sensor Fusion for Localization & Mapping -- 多传感器融合定位与建图

深蓝学院[多传感器融合定位与建图/Multi-Sensor Fusion for Localization & Mapping](https://www.shenlanxueyuan.com/course/324)学习环境.

Maintained by **Ge Yao**, alexgecontrol@qq.com

---

## Overview

本Repo为基于**ROS melodic** @ **Ubuntu 18.04**的[多传感器融合定位与建图/Multi-Sensor Fusion for Localization & Mapping](https://www.shenlanxueyuan.com/course/324)学习环境. 

定位与建图是一个复杂的工程. **每一个解决方案, 都有较为复杂的环境依赖**. 然而:

* 部分依赖项由于网络原因(Great Fire Wall & Server IP Block)难以直接获得

* 由于课程依赖项的版本, 与本地现有依赖项的版本, 可能有所不同, 直接冒然安装, 可能会导致本地开发环境被破坏

故: 本课程的学习环境将以**Docker Image**的形式提供, 以实现与Native PC环境的隔离. 

* 本项目Native PC的操作系统选择**Ubuntu**. Windows与Mac不推荐使用, 若有意尝试, 请自行搜索相关解决方案.


**若您坚持在Native PC上进行开发**, 此处**将默认您有足够的定位&建图开发环境运维经验, 能够自定义开发环境, 并且自主解决由于环境依赖冲突导致的各种问题**. 请您

* 根据Dockerfile[here](docker/cpu.Dockerfile)自行准备开发环境
* 熟悉其中的各个依赖库, 有能力在出现依赖问题时, 自行查询相关资料, 修复依赖冲突

---

### 安装Ubuntu

首先请确保您可以访问**Ubuntu**开发环境. 如果没有**Ubuntu**环境, 请按照[点击链接进入](ubuntu-setup/README.md)指南, 在本地PC上安装配置**Ubuntu**环境.

---

### 获取Docker开发环境

本课程推荐使用配套的Docker环境[点击链接进入](docker/README.md)完成课程学习. Docker提供了一个轻量级的标准化开发环境, 能够避免环境配置差异导致的诸多问题.

---

### Workspace

当Native PC与Course Docker Environment均准备就绪时, 即可开始使用本开发环境:

* **第一次使用时, 请首先下载课程配套的KITTI测试数据**[here](workspace/data/kitti/README.md).

* 之后, 即可通过**本地VSCode开发, Docker内部编译测试**的模式, 完成课程作业.

#### 获取课程数据

在第一次使用时, 需要将**课程配套的修复后KITTI数据**下载至本地文件系统. 具体操作方法参考[点击链接进入](workspace/data/kitti/README.md)

#### 开发, 编译与测试

启动Docker环境后, **Docker**中的/workspace目录, 会被映射到**当前Repo**中的workspace目录.

使用该Workspace进行开发, 编译与测试的方法如下:

* 在**当前Repo的workspace**下, 启动[VSCode](https://code.visualstudio.com/), 编辑源代码:

<img src="doc/development-environment.png" alt="Development Environment, Native VS Code in Mounting Point" width="100%">

* 在**Docker /workspace**下, 进行编译. 具体的编译方法, 请参考[作业1 环境搭建](workspace/assignments/01-introduction/README.md)


---

Keep Learning & Keep Coding

Ge Yao, alexgecontrol@qq.com