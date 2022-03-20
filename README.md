# 多传感器融合定位-章节索引

前言：

​	本博客为[深蓝学院](https://www.shenlanxueyuan.com/)多传感器融合定位的课程作业笔记，为了方便个人检索，故将笔记记录到网上，同时也希望能给大家一些启发。在讲师任乾老师和其他学员的帮助下，总算完成十章的作业。

​		回顾整个历程，先后修了三次的课程，也十分感慨。刚录取研究生(2020年)的时候，导师给了我一个课题，做3D巡检小车的定位建图，在本科一直参与robotics的比赛，接触了不少的2D agv项目，因此也欣然接受。开始了资料的检索，开始了解到点云，先修了深蓝学院黎嘉信老师的三维点云处理，对点云有了初级的了解。

​		第一阶段 2020.10：后来接触到了任乾老师的第一期多传感器融合定位课程(2020.10),开始发现自己的知识十分薄弱，对非线性优化、惯导等概念一无所知，甚至不知道什么是外参、内参，多线激光雷达也从没接触过，c++也十分的差。第一期课程的难度对于我这样的菜鸟来说确实很难，我一边上课一遍不断补习知识，去翻阅高博的slam14讲、机器人状态估计。最后因为临近期末考试和个人基础知识太薄弱，上到第四章(惯性导航分析)就没跟上。之后我重新把高翔博士的14讲重看了一遍，也在实验室自己搭建造了一辆[3D的slam移动小平台](https://blog.csdn.net/weixin_41281151/article/details/113558183),通过自己的选型，设计框架，开始慢慢了解在自动驾驶领域传感器的种类和属性，也开始对传感器有了更深的理解。

​		第二阶段 2021.8 : 在外出实习期间，一直没有放弃对slam的学习，进行了第四期课程的学习，这一次的学习更加细致，进行了1-8章较为详细的作业，对比之前，这一次开始看懂代码，理解其中算法的含义，理解ceres g2o 等非线性优化的使用，很好地了解了kalman的融合，在此之前kalman一直停留在本科期间机器人比赛中自瞄跟踪算法，但也只是调库，这一次有了更深的理解，也不会畏惧公式，开始耐心的推导。因为临近开题原因，故这一次学习到第八章kalman容和定位。

​		第三阶段 2022.3：在第二阶段的课程学习之后，期间认真阅读了课程的框架代码，也开始自己搭建平台，制作数据集，将课程的框架部署到自己的搭建的平台上。如：[3D-SLAM自搭平台 主动阿克曼 + RS16 + LPMS_IMU LEGO_LOAM 建图](https://blog.csdn.net/weixin_41281151/article/details/122492223)    [B站视频](https://m.bilibili.com/space/95253336)。开始了第三次的多传感器容和定位的学习，这一次，补充了第九和第十章，预积分和图优化的部分，优化和预积分的方法是未来的一个趋势，可以说是课程的核心内容。通过不断的提问，反复看视频，交叉观看VIO的课程，开始有了自己的理解。

​		一路以来的学习，有赖于任乾老师和各位助教学员的解答，非常感谢各位。

​		列宁曾说过“人的认识不是直线，而是无限地近似于一串圆圈、近似于螺旋式的曲线”，事物的自身发展，经过肯定、否定和新的肯定。同样人的认知也需要经过“否定之否定”的过程，在不断地否定中自我成长。

## 课程笔记检索

1-10章的课程检索  [github 链接](https://github.com/kahowang/sensor-fusion-for-localization-and-mapping) (如果对大家有帮助，烦请大家star我一下~)

[多传感器融合定位 第一章 概述](https://blog.csdn.net/weixin_41281151/article/details/119777842)

[多传感器融合定位 第二章 3D激光里程计](https://blog.csdn.net/weixin_41281151/article/details/119883874)

[多传感器融合定位 三章 3D激光里程计2](https://blog.csdn.net/weixin_41281151/article/details/122504556)

[多传感器融合定位 第四章 点云地图构建及基于点云地图定位](https://blog.csdn.net/weixin_41281151/article/details/120116838)

[多传感器融合定位 第五章 惯性导航原理及误差分析-不需要转台的IMU内参标定](https://blog.csdn.net/weixin_41281151/article/details/120413308)

[多传感器融合定位 第六章 惯性导航结算及误差模型](https://blog.csdn.net/weixin_41281151/article/details/120686591)

[多传感器融合定位 第七章 基于滤波的融合方法](https://blog.csdn.net/weixin_41281151/article/details/120904368)

[多传感器融合定位 第八章 基于滤波的融合方法进阶](https://blog.csdn.net/weixin_41281151/article/details/120911364)

[多传感器融合定位 第九章 基于优化的建图方法](https://blog.csdn.net/weixin_41281151/article/details/123406337)

[多传感器融合定位 第十章 基于优化的定位方法](https://blog.csdn.net/weixin_41281151/article/details/123611615)

[自动驾驶惯性传感器中的基本原理笔记](https://blog.csdn.net/weixin_41281151/article/details/123308757)

[多传感器融合定位-常用辅助调试工具总结](https://blog.csdn.net/weixin_41281151/article/details/121732073)



## 自搭实验平台

### jackal  + 镭神32 

[搭建实验室3d slam 移动小车 1-前期准备](https://blog.csdn.net/weixin_41281151/article/details/112920317)

[搭建实验室3d slam 移动小车 2.1-镭神32线激光雷达使用调试](https://blog.csdn.net/weixin_41281151/article/details/112928637)

[搭建实验室3d slam 移动小车 2.2镭神32线激光雷达修改主从机IP](https://blog.csdn.net/weixin_41281151/article/details/113063851)

[搭建实验室3d slam 移动小车 2.3镭神32线激光雷达ROS-RVIZ上方向确定](https://blog.csdn.net/weixin_41281151/article/details/113556506)

[搭建实验室3d slam 移动小车 3.1jackal移动小车平台调试](https://blog.csdn.net/weixin_41281151/article/details/113064766)

[搭建实验室3d slam 移动小车 3.2jackal移动平台axis-ptz魚眼摄像头调试](https://blog.csdn.net/weixin_41281151/article/details/113094378)

[搭建实验室3d slam 移动小车 3.3jackal移动平台 组合导航POMS-GI201C、镭神32线激光雷达 卫星授时](https://blog.csdn.net/weixin_41281151/article/details/115519477)

[搭建实验室3d slam 移动小车 4.1jackal小车+镭神32线激光雷达lego-loam建图](https://blog.csdn.net/weixin_41281151/article/details/113558183)

[使用Mapviz、中科图新 进行机器人GPS轨迹卫星地图绘制](https://blog.csdn.net/weixin_41281151/article/details/114046438)

[使用Mapviz,进行机器人GPS轨迹卫星地图绘制(2)-调用天地图API，快速加载刷新地图](https://blog.csdn.net/weixin_41281151/article/details/120630786)



### 主动阿克曼 + RS16 + LPMS_IMU

[3D-SLAM自搭平台 主动阿克曼 + RS16 + LPMS_IMU LEGO_LOAM 建图](https://blog.csdn.net/weixin_41281151/article/details/122492223)



## 第一阶段深蓝多传感课程检索

第一期课程学习，也进行了一些笔记记录，但不全，望见谅

[多传感器融合定位 课程概述](https://blog.csdn.net/weixin_41281151/article/details/109004126)

[多传感器融合定位（1-3D激光里程计）1-前端里程计ICP](https://blog.csdn.net/weixin_41281151/article/details/109008854)

[多传感器融合定位（1-3D激光里程计）2-前端里程计NDT](https://blog.csdn.net/weixin_41281151/article/details/109018860)

[多传感器融合定位（1-3D激光里程计）3-前端里程计LOAM系列](https://blog.csdn.net/weixin_41281151/article/details/109030376)

[多传感器融合定位（1-3D激光里程计）4-实现调用pcl-icp-1](https://blog.csdn.net/weixin_41281151/article/details/109116984)

[多传感器融合定位（1-3D激光里程计）5-实现调用pcl-icp-2 通过 config.yaml 实现接口的多态性](https://blog.csdn.net/weixin_41281151/article/details/109129415)

[多传感器融合定位（1-3D激光里程计）6-实现调用pcl-icp-3 evo里程计精度评价](https://blog.csdn.net/weixin_41281151/article/details/109133442)

[多传感器融合定位（2-点云地图构建及基于地图定位）1-回环检测及代码实现](https://blog.csdn.net/weixin_41281151/article/details/109167498)

[多传感器融合定位（2-点云地图构建及基于地图定位）2-后端优化与点云地图构建](https://blog.csdn.net/weixin_41281151/article/details/109188866)

[多传感器融合定位（3-点云地图构建及基于地图定位）3-实现ScanContext 回环检测](https://blog.csdn.net/weixin_41281151/article/details/109290982)

[多传感器融合定位（4-点云地图构建及基于地图定位）4-通过GNSS 实现地图定位](https://blog.csdn.net/weixin_41281151/article/details/109280413)

[多传感器融合定位（3-惯性导航原理及误差分析）2-IMU误差分析及处理：随机误差理论分析&allan方差分析及实现](https://blog.csdn.net/weixin_41281151/article/details/109319775)

[多传感器融合定位（3-惯性导航原理及误差分析）3-内参模型与分立级、系统级、迭代优化等标定方法：器件内参标定](https://blog.csdn.net/weixin_41281151/article/details/109362003)

[多传感器融合定位（3-惯性导航原理及误差分析）4-IMU温补：常见温补模型与基于多项式你和的温补方法：惯性器件误差分析](https://blog.csdn.net/weixin_41281151/article/details/109364735)

[多传感器融合定位（1-3D激光里程计）6-实现手写icp](https://blog.csdn.net/weixin_41281151/article/details/109439049)

[多传感器融合定位（3-惯性导航原理及误差分析）7-实现 分立级标定 accel加速度计内参公式验证](https://blog.csdn.net/weixin_41281151/article/details/109512065)

[多传感器融合定位（3-惯性导航原理及误差分析）6-Allan方差 进行随机误差分析](https://blog.csdn.net/weixin_41281151/article/details/109559980)

[多传感器融合定位（4-基于滤波的融合方法）kitti数据集 IMU频率改为100HZ](https://blog.csdn.net/weixin_41281151/article/details/109873579)

[Docker快速上手及常用指令集](https://blog.csdn.net/weixin_41281151/article/details/109966209)

[多传感器融合定位（3-惯性导航原理及误差分析）8-惯性导航解算验证](https://blog.csdn.net/weixin_41281151/article/details/110313225)

[多传感器融合定位（4-基于滤波的2融合方法）2-使用仿真数据进行imu-gnss eskf和时变系统下的可观测性分析](https://blog.csdn.net/weixin_41281151/article/details/110677974)

[ROS自定义消息类型，编译无法生成 msg/srv 文件产生的头文件](https://blog.csdn.net/weixin_41281151/article/details/118754614)

​																																		   edited  by kaho 2022.3.20  祝愿疫情早日消退，花开满地