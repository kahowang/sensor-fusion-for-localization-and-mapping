# 多传感器融合定位 尝试使用scan_context方法进行匹配重定位(失败)

参考博客：

[Scan Context 介绍及理解](https://zhuanlan.zhihu.com/p/393353116)

[Lidar定位：Scan Context](https://zhuanlan.zhihu.com/p/359523177)

[激光闭环检测Scancontext阅读笔记](https://www.cnblogs.com/long5683/p/13380572.html)

前言：[使用 pcl_ndt 遍历匹配 relocalization ](https://blog.csdn.net/weixin_41281151/article/details/121455843)存在的问题：

1.使用角度遍历初始化的方法，遍历360度，耗费大量的时间，大约为(7~8s)时间，若匹配初始化的地方为角点，容易发生姿态换向，导致初始化姿态失败。

2.ndt_pcl 遍历对场景要求比较高，在结构性复杂的场景会有比较好的变现，结构性比较弱的场景，表现不好。

鉴于上述原因，选择使用更加快速的匹配方法辅助，使用scancontext 描述子的方法进行加速匹配。scancontext方法主要用于回环检测用，通过把当前帧点云在历史帧点云中进行搜寻，得到匹配的位姿。本次的灵感主要在于sc方法的描述子，具体算法流程为，通过gps分割出local_map，构建sc_local_map,然后把将当前点云旋转360度，每旋转1度计算一次sc_cur_scan,将sc_local_map 和 sc_cur_scan 进行比较，选出最优的旋转。但是实验结果并不符合预期，故此方法不可行，但是在调试过程中使用了较多的方法和思路可以值得借鉴~

## 具体代码

FILE: kitti_filtering.cpp    KITTIFiltering::InitOri

### 1. 旋转 cur_scan，并获取对应的sc，存在sc_buffer中

```cpp
// kitti_filtering.cpp    KITTIFiltering::InitOri     
Eigen::Matrix4f  translation  =  Eigen::Matrix4f::Identity();     
       int iter_rate  = 360;
       for(int i = 1; i <  iter_rate + 1;  i++){     //  分成360份
             double  angle =   2*M_PI /  360 * i;
             Eigen::AngleAxisf rotation_vector(angle,  Eigen::Vector3f(0,0,1))  ;   // 绕z轴旋转  360  180  120
             Eigen::Matrix3f    rotation_ =  rotation_vector.matrix();
             translation.block<3,3>(0,0) = rotation_.block<3,3>(0,0);

            pcl::transformPointCloud(*filtered_cloud_ptr,   *transformed_scan_.cloud_ptr, translation);         
            scan_context_manager_ptr_->SaveInitOri(transformed_scan_);              //  计算遍历后的sc描述子,并储存
    /*----------------------------------------------------------------------------------------------------------------------------------------------------------*/
// scan_context_manager.cpp  SaveInitOri
  void ScanContextManager::SaveInitOri(const CloudData &scan){
    // extract scan context and corresponding ring key:
    ScanContext scan_context = GetScanContext(scan);
    RingKey ring_key = GetRingKey(scan_context);

    // update buffer:
    state_.scan_context_.push_back(scan_context);					//  存储sc
    state_.ring_key_.push_back(ring_key);									   //  存储ring_key
    state_.index_.data_.ring_key_.push_back(ring_key);				

    if(state_.ring_key_.size()  == 360){											     //  构建ring_key  kd_tree
        state_.index_.kd_tree_.reset(); 
        state_.index_.kd_tree_ = std::make_shared<RingKeyIndex>(
        NUM_RINGS_,  /* dim */
        state_.index_.data_.ring_key_,
        10           /* max leaf size */
    );
    LOG(INFO) << "\tIndex Size: " << state_.index_.kd_tree_->kdtree_get_point_count() 
              << std::endl;
    }
  }
```

### 2. 根据GPS获取local_map,并获取local_map_sc ，和cur_scan_sc  进行匹配

```cpp
// kitti_filtering.cpp    KITTIFiltering::InitOri     
 Eigen::Matrix4f temp_pose = Eigen::Matrix4f::Identity();
      if(!scan_context_manager_ptr_->DetectLoopClosure(local_map_,  temp_pose) ){
          std::cout << "-------------------------------DetectLoopClosure   false  ------------------------------------"    <<  std::endl;
          return  false;
      }
/*---------------------------------------------------------------------------------------------------------------------------------------------------*/
//  scan_context_manager.cpp   DetectLoopClosure
/**
 * @brief  get loop closure proposal using the given key scan
 * @param  scan, query key scan
 * @param  pose, matched pose
 * @return true for success match otherwise false
 */
bool ScanContextManager::DetectLoopClosure(
    const CloudData &scan,
    Eigen::Matrix4f &pose
) {
    // extract scan context and corresponding ring key:
    ScanContext query_scan_context = GetScanContext(scan);
    RingKey query_ring_key = GetRingKey(query_scan_context);

    //scan_context  debug
    std::cout  << "sc_matrix_local_map  =   "    <<std::endl;
    Eigen::MatrixXf   sc_matrix(query_scan_context.rows(), query_scan_context.cols());
    for(int did = 0; did < query_scan_context.rows();  ++ did){
        for(int sid = 0; sid < query_scan_context.cols(); ++sid){
            sc_matrix(did,sid)  =   query_scan_context(did,sid);
        }
    }
    CreateFile(sc_local_map, sc_local_map_path);   //  创建文件夹
    WriteText(sc_local_map, sc_matrix);             // 把 sc_matrxi_local_map  写进文件夹中
    //scan_context  debug
    
    // get proposal:
    std::pair<int, float> proposal = GetLoopClosureMatch(
        query_scan_context, query_ring_key
    );

    const int key_frame_id = proposal.first;
    const float yaw_change_in_rad = proposal.second;

    std::cout <<  "key_frame_id  = " << key_frame_id  <<std::endl;
    std::cout <<  "yaw_change_in_rad  = " << yaw_change_in_rad  <<std::endl;

    // check proposal validity:
    if (ScanContextManager::NONE == key_frame_id) {
        std::cout <<  "--------------------False  NONE  ==  key_frame_id ---------------------- " <<  std::endl;
        return false;
    }

    // set matched pose:
    pose = state_.index_.data_.key_frame_.at(key_frame_id).pose;
    // apply orientation change estimation:
    Eigen::AngleAxisf orientation_change(yaw_change_in_rad, Eigen::Vector3f::UnitZ());
    pose.block<3, 3>(0, 0) = pose.block<3, 3>(0, 0) * orientation_change.toRotationMatrix();

    std::cout << "--------------------DetectLoopClosure   true -----------------------"  << std::endl;
    // finally:
    return true;
}
```

### 3.实验记录

#### 3.1  sc_local_map  VS  sc_curent_scan

如下图所示，为20*60矩阵，为scan_context的描述子(黄色为1：存在点云，紫色为0：没有点云)。sc的栅格分辨率为(4m x 6度),sc_local_map点云覆盖范围明显要大很多，而且sc_lcoal_map 和  sc_curent 描述子差异巨大,无法进行匹配。所以需要对点云进行可视化继续分析。

![2021-12-04 21-57-46 的屏幕截图](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E5%85%AD%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112021-12-04%2021-57-46%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)

#### 3.2  local_map  curent_scan  open3d 点云可视化分析

如下所示灰色为lcoal_map  红色为current_scan的可视化

初步判断：产生上述sc描述子巨大差异的原因是，雷达当前点云视角受限，看不到地图中的一些区域，导致匹配不上。所以使用先验地图的sc和current_scan的sc进行匹配获取初始化定位的方法并不可行。

解决方法：建图的时候记录各个关键帧的点云，然后匹配的时候只跟初始位置附近的关键帧匹配

![2021-12-05 14-11-10 的屏幕截图](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E5%85%AD%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112021-12-05%2014-11-10%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png) 																																																		

### 4.调试方法汇总

#### 4.1 mamtplotlib可视化scan_context矩阵

存储scan_context 矩阵都txt文件中

```cpp
#include <list>
#include <sstream>
#include <fstream>
#include <iomanip>
std::ofstream  sc_local_map;
std::ofstream  sc_cur_scan;
char sc_local_map_path[] = "/home/kaho/catkin2_ws/visual_sc_matrix/sc_matrix_local_map.txt";
char sc_cur_scan_path[] = "/home/kaho/catkin2_ws/visual_sc_matrix/sc_matrix_cur_scan.txt";

bool CreateFile(std::ofstream& ofs, std::string file_path) {
    ofs.open(file_path, std::ios::out);                          //  使用std::ios::out 可实现覆盖
    if(!ofs)
    {
        std::cout << "open csv file error " << std::endl;
        return  false;
    }
    return true;
}

/* write2txt  */
void WriteText(std::ofstream& ofs, Eigen::MatrixXf   sc_matrix){
    for(int did = 0; did < sc_matrix.rows();  ++did){
        for(int sid=0; sid < sc_matrix.cols(); ++sid){
                ofs << std::fixed  << sc_matrix(did, sid)  << "\t";
        }ofs  << "\n";
    }
}
```

调用方法：

```cpp
    Eigen::MatrixXf   sc_matrix(query_scan_context.rows(), query_scan_context.cols());				// 将sc存到sc_matrix 中
    for(int did = 0; did < query_scan_context.rows();  ++ did){
        for(int sid = 0; sid < query_scan_context.cols(); ++sid){
            sc_matrix(did,sid)  =   query_scan_context(did,sid);
        }
    }   
CreateFile(sc_local_map, sc_local_map_path);   //  创建文件夹
    WriteText(sc_local_map, sc_matrix);             // 把 sc_matrxi_local_map  写进文件夹中
```

python matplotlib 读取sc_matrix  并可视化矩阵

```python
import matplotlib.pyplot as plt
import numpy as np

sc_matrix_cur_scan = np.loadtxt("sc_matrix_cur_scan.txt")
sc_matrix_lcoal_map = np.loadtxt("sc_matrix_local_map.txt")

def samplemat(mat):
    """Make a matrix with all zeros and increasing elements on the diagonal"""
    sector = mat.shape[0]   # x
    ring = mat.shape[1]     # y
    aa = np.zeros((sector,ring))
    for i in range(sector):
        for j in range(ring):
            if(mat[i,j]):
                aa[i,j]  = 1 #mat[i,j]
            else:
                aa[i, j] = mat[i,j]
    return aa

fig = plt.figure(figsize=(10,10))
ax1 =  fig.add_subplot(2,1,1)          # 2*1 分布，第1个

# Display matrix
#ax1.matshow(samplemat(sc_matrix_lcoal_map))   #,cmap = plt.cm.gray
ax1.matshow(sc_matrix_lcoal_map)   #,cmap = plt.cm.gray
plt.title("sc_local_map");

ax2 =  fig.add_subplot(2,1,2)          # 2*1 分布，第2个
#ax2.matshow(samplemat(sc_matrix_cur_scan))   #,cmap = plt.cm.gray
 ax2.matshow(sc_matrix_cur_scan)   #,cmap = plt.cm.gray
plt.title("sc_current_scan ");

plt.show()
```

![2021-12-05 16-27-36 的屏幕截图](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E5%85%AD%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112021-12-05%2016-27-36%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)																																																			

#### 4.2 open3d可视化点云

存储点云为pcd

```cpp
pcl::io::savePCDFileASCII("/home/kaho/catkin2_ws/visual_sc_matrix/pcd/cur_scan.pcd",*transformed_scan_.cloud_ptr );
//save pcd 
 pcl::io::savePCDFileASCII("/home/kaho/catkin2_ws/visual_sc_matrix/pcd/local_map.pcd",*local_map_ptr_ );
```

python open3d 可视化

```python
import open3d as o3d
import numpy as np
local_map_pcd = o3d.io.read_point_cloud("pcd/local_map.pcd")
cur_scan_pcd = o3d.io.read_point_cloud("pcd/cur_scan.pcd")

local_map_points = np.asarray(local_map_pcd.points)
cur_scan_points = np.asarray(cur_scan_pcd.points)


# print(local_map_pcd)#输出点云点的个数
# print(cur_scan_pcd)#输出点云点的个数
#
print(local_map_points)#输出点的三维坐标
# print(cur_scan_points)#输出点的三维坐标
#
# print('给所有的点上一个统一的颜色，颜色是在RGB空间得[0，1]范围内得值')

local_map_pcd.paint_uniform_color([0.9, 0.9, 0.9])        # blue
cur_scan_pcd.paint_uniform_color([1, 0, 0])        # red


o3d.visualization.draw_geometries([local_map_pcd,cur_scan_pcd])
```

![2021-12-05 14-10-40 的屏幕截图](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E5%85%AD%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112021-12-05%2014-10-40%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)																																																		

​																																																							edited  by kaho  12.5

