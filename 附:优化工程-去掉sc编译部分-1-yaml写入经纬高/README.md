# 多传感器融合定位 优化工程-1-去掉sc编译部分-yaml写入经纬高

参考博客：[多传感器融合定位 第四章 点云地图构建及基于点云地图定位](https://blog.csdn.net/weixin_41281151/article/details/120116838)

前言：最近在使用任乾老师的工程，因为ScanContext部分版本比较混乱，导致产生编译上比较累赘，而且工程上甚少用到，所以本次工程优化，打算去掉这一部分。 并把原点地图经纬高初始部分通过yaml文件写入。

## 1. lidar_localization 工程 整体工程优化

### 1.1 编译优化，去掉ScanContext 编译

因为原作者GeYao大神增添了scanconext初始化功能，使用到了proto buff的环形队列传输数据，依赖特定版本的protobuf版本，在工程使用上，scancontext效果并不是很好，为了适配自身的数据集，故将这部分删除，编译优化。

```cpp
FILE: kitti_filtering.cpp
line  324  SetInitScan()
line 31  bool KITTIFiltering::Init()
line 202   InitScanContextManager(config_node);
line 271   KITTIFiltering::InitScanContextManager
```

```cpp
FILE: kitti_filtering.hpp
line 23    #include "lidar_localization/models/scan_context_manager/scan_context_manager.hpp"
line 96     std::shared_ptr<ScanContextManager> scan_context_manager_ptr_;
line 36     bool Init();
line 87     bool SetInitScan(const CloudData &init_scan);
line 75     bool InitScanContextManager(const YAML::Node &config_node);
line 110   std::shared_ptr<ScanContextManager> scan_context_manager_ptr_;
```

```shell
delete  
lidar_localization/src/models/scan_context_manager
lidar_localizationconfig/scan_context_manager
lidar_localization/include/lidar_localization/models/scan_context_manager
```

### 1.2   优化初始化地图原点gnss经纬高的部分，改为yaml输入，避免不必要的编译

仿照其他yaml_config的写法

FILE: kitti_filtering.hpp  添加对应头文件

```cpp
#include "lidar_localization/global_defination/global_defination.h"
#include <yaml-cpp/yaml.h>
```

FILE:  kitti_filtering.cpp   在InitOriginPosition() 函数中增添从yaml 中读取原点地图的经纬高信息

```cpp
void GNSSData::InitOriginPosition() {
    //init  origin latitude  longitude  altitude  form yaml
    std::string  config_file_path = 
        WORK_SPACE_PATH + "/config/filtering/kitti_filtering.yaml";
    
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    origin_latitude  =  config_node["origin_latitude"].as<double>();
    origin_longitude = config_node["origin_longitude"].as<double>();
    origin_altitude  = config_node["origin_altitude"].as<double>();
    
    //geo_converter.Reset(latitude, longitude, altitude);
    geo_converter.Reset(origin_latitude, origin_longitude, origin_altitude);         //   设置原点
    std::cout << "----------------------init  OriginPosition -------------------------------"   << std::endl;   

    origin_position_inited = true;
}
```

FILE:  kitti_filtering.yaml 中添加经纬高信息

```yaml
origin_latitude: 48.982658
origin_longitude: 8.390455
origin_altitude: 116.396412
```

​																																																						sedited  by kaho  11.23