/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-06 20:42:23
 */
#include "lidar_localization/sensor_data/gnss_data.hpp"

#include "glog/logging.h"
#include <ostream>
#include <iostream>

//静态成员变量必须在类外初始化
double lidar_localization::GNSSData::origin_latitude = 0.0;
double lidar_localization::GNSSData::origin_longitude = 0.0;
double lidar_localization::GNSSData::origin_altitude = 0.0;

bool lidar_localization::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian lidar_localization::GNSSData::geo_converter;

namespace lidar_localization {
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

void GNSSData::UpdateXYZ() {
    if (!origin_position_inited) {
        LOG(WARNING) << "WARNING: GeoConverter is NOT initialized.";
    }

    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
}

void GNSSData::Reverse(
    const double &local_E, const double &local_N, const double &local_U,
    double &lat, double &lon, double &alt
) {
    if (!origin_position_inited) {
        LOG(WARNING) << "WARNING: GeoConverter is NOT initialized.";
    }

    geo_converter.Reverse(local_E, local_N, local_U, lat, lon, alt);
}

bool GNSSData::SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time) {
    // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
    // 即找到与同步时间相邻的左右两个数据
    // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
    while (UnsyncedData.size() >= 2) {
        if (UnsyncedData.front().time > sync_time)             //  1）如果第一个数据时间比雷达时间还要靠后，即插入时刻的前面没有数据，那么就无从插入，直接退出
            return false;
        if (UnsyncedData.at(1).time < sync_time) {              //  2）如果第一个数据比插入时刻早，第二个数据也比插入时刻早，那么第一个时刻的数据是没意义的，应该接着往下找，并删除第一个数据
            UnsyncedData.pop_front();
            continue;
        }
        if (sync_time - UnsyncedData.front().time > 0.2) {   // 3）如果雷达采集时刻已经处在前两个数据的中间了，但是第一个数据时刻与雷达采集时刻时间差过大，那么中间肯定丢数据了，退出
            UnsyncedData.pop_front();
            return false;
        }
        if (UnsyncedData.at(1).time - sync_time > 0.2) {       // 4）同样，如果第二个数据时刻与雷达采集时刻时间差过大，那么也是丢数据了，也退出
            UnsyncedData.pop_front();
            return false;
        }
        break;
    }
    if (UnsyncedData.size() < 2)
        return false;

    GNSSData front_data = UnsyncedData.at(0);           //  前一个数据
    GNSSData back_data = UnsyncedData.at(1);           //  后一个数据
    GNSSData synced_data;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.status = back_data.status;
    synced_data.longitude = front_data.longitude * front_scale + back_data.longitude * back_scale;
    synced_data.latitude = front_data.latitude * front_scale + back_data.latitude * back_scale;
    synced_data.altitude = front_data.altitude * front_scale + back_data.altitude * back_scale;
    synced_data.local_E = front_data.local_E * front_scale + back_data.local_E * back_scale;
    synced_data.local_N = front_data.local_N * front_scale + back_data.local_N * back_scale;
    synced_data.local_U = front_data.local_U * front_scale + back_data.local_U * back_scale;

    SyncedData.push_back(synced_data);
    
    return true;
}

}