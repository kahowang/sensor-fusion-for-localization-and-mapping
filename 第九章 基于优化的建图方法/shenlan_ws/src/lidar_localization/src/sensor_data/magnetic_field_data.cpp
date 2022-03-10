/*
 * @Description: magnetic field data
 * @Author: Ren Qian
 * @Date: 2020-11-25 23:07:14
 */
#include "lidar_localization/sensor_data/magnetic_field_data.hpp"

#include "glog/logging.h"

namespace lidar_localization {

bool MagneticFieldData::SyncData(
    std::deque<MagneticFieldData>& UnsyncedData, 
    std::deque<MagneticFieldData>& SyncedData, 
    double sync_time
) {
    // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
    // 即找到与同步时间相邻的左右两个数据
    // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
    while (UnsyncedData.size() >= 2) {
        if (UnsyncedData.front().time > sync_time)
            return false;
        if (UnsyncedData.at(1).time < sync_time) {
            UnsyncedData.pop_front();
            continue;
        }
        //
        // here assumes the measurement is generated at 400Hz
        //
        if (sync_time - UnsyncedData.front().time > 0.005) {
            UnsyncedData.pop_front();
            return false;
        }
        if (UnsyncedData.at(1).time - sync_time > 0.005) {
            UnsyncedData.pop_front();
            return false;
        }
        break;
    }
    if (UnsyncedData.size() < 2)
        return false;

    MagneticFieldData front_data = UnsyncedData.at(0);
    MagneticFieldData back_data = UnsyncedData.at(1);
    MagneticFieldData synced_data;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.magnetic_field.x = front_data.magnetic_field.x * front_scale + back_data.magnetic_field.x * back_scale;
    synced_data.magnetic_field.y = front_data.magnetic_field.y * front_scale + back_data.magnetic_field.y * back_scale;
    synced_data.magnetic_field.z = front_data.magnetic_field.z * front_scale + back_data.magnetic_field.z * back_scale;

    SyncedData.push_back(synced_data);

    return true;
}

void MagneticFieldData::TransformCoordinate(
    Eigen::Matrix4f transform_matrix
) {
    Eigen::Matrix4d matrix = transform_matrix.cast<double>();

    // parse orientation:
    Eigen::Matrix3d R = matrix.block<3,3>(0,0);

    // get magnetic field in IMU frame:
    Eigen::Vector3d t(magnetic_field.x, magnetic_field.y, magnetic_field.z);

    // transform to target frame:
    t = R.transpose() * t;

    // finally:
    magnetic_field.x = t(0);
    magnetic_field.y = t(1);
    magnetic_field.z = t(2);
}

void MagneticFieldData::NED2ENU(void) {
    MagneticField magnetic_field_enu;

    magnetic_field_enu.x = +magnetic_field.y;
    magnetic_field_enu.y = +magnetic_field.x;
    magnetic_field_enu.z = -magnetic_field.z;

    magnetic_field.x = magnetic_field_enu.x;
    magnetic_field.y = magnetic_field_enu.y;
    magnetic_field.z = magnetic_field_enu.z;
}

} // namespace lidar_localization