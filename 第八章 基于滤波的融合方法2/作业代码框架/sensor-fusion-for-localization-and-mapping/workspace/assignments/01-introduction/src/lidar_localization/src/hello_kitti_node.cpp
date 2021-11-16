/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:56:27
 */

#include <cstdlib>
#include <ros/ros.h>
#include <pcl/common/transforms.h>

#include "lidar_localization/global_defination/global_defination.h"
#include "glog/logging.h"

//
// TF tree:
//
#include "lidar_localization/subscriber/tf_listener.hpp"
#include "lidar_localization/publisher/tf_broadcaster.hpp"

//
// subscribers:
//
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/imu_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
//
// publishers:
//
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"

using namespace lidar_localization;

void GetTransformIMUToMap(
    GNSSData &gnss_data, IMUData &imu_data,
    Eigen::Matrix4f &imu_to_map
) {
    //
    // init
    // 
    gnss_data.UpdateXYZ();
    //
    // a. set position:
    // 
    imu_to_map(0,3) = gnss_data.local_E;
    imu_to_map(1,3) = gnss_data.local_N;
    imu_to_map(2,3) = gnss_data.local_U;
    //
    // b. set orientation:
    //
    imu_to_map.block<3,3>(0,0) = imu_data.GetOrientationMatrix();
}


int main(
    int argc, 
    char *argv[]
) {
    google::InitGoogleLogging(argv[0]);
    
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "hello_kitti_node");
    ros::NodeHandle nh;

    // 
    // get TF: 
    //
    std::shared_ptr<TFListener> lidar_to_imu_tf_sub_ptr = std::make_shared<TFListener>(nh, "/imu_link", "/velo_link");
    std::shared_ptr<TFBroadCaster> lidar_to_map_tf_pub_ptr = std::make_shared<TFBroadCaster>("/map", "/velo_link");

    //
    // subscribe to topics:
    //
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    std::shared_ptr<IMUSubscriber> imu_sub_ptr = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);

    //
    // register publishers:
    //
    std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "/map");
    std::shared_ptr<OdometryPublisher> odom_pub_ptr = std::make_shared<OdometryPublisher>(nh, "lidar_odom", "/map", "velo_link", 100);

    std::deque<CloudData> cloud_data_buff;
    std::deque<IMUData> imu_data_buff;
    std::deque<GNSSData> gnss_data_buff;

    Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f imu_to_map = Eigen::Matrix4f::Identity();

    bool transform_received = false;
    bool gnss_origin_position_inited = false;

    ros::Rate rate(100);

    while (ros::ok()) {
        ros::spinOnce();

        cloud_sub_ptr->ParseData(cloud_data_buff);
        imu_sub_ptr->ParseData(imu_data_buff);
        gnss_sub_ptr->ParseData(gnss_data_buff);

        if (!transform_received) {
            if (lidar_to_imu_tf_sub_ptr->LookupData(lidar_to_imu)) {
                transform_received = true;
            }
        } else {
            while (
                !cloud_data_buff.empty() && 
                !imu_data_buff.empty() && 
                !gnss_data_buff.empty()
            ) {
                CloudData cloud_data = cloud_data_buff.front();
                IMUData imu_data = imu_data_buff.front();
                GNSSData gnss_data = gnss_data_buff.front();

                double d_time = cloud_data.time - imu_data.time;

                if (d_time < -0.05) {
                    cloud_data_buff.pop_front();
                } else if (d_time > 0.05) {
                    imu_data_buff.pop_front();
                    gnss_data_buff.pop_front();
                } else {
                    cloud_data_buff.pop_front();
                    imu_data_buff.pop_front();
                    gnss_data_buff.pop_front();

                    if (!gnss_origin_position_inited) {
                        gnss_data.InitOriginPosition();
                        gnss_origin_position_inited = true;
                    }
                    
                    GetTransformIMUToMap(
                        gnss_data, imu_data,
                        imu_to_map
                    );

                    // lidar pose in map frame:
                    Eigen::Matrix4f lidar_odometry = imu_to_map * lidar_to_imu;
                    // lidar measurement in map frame:
                    pcl::transformPointCloud(
                        *cloud_data.cloud_ptr, *cloud_data.cloud_ptr, 
                        lidar_odometry
                    );

                    cloud_pub_ptr->Publish(cloud_data.cloud_ptr);
                    odom_pub_ptr->Publish(lidar_odometry);

                    //
                    // publish TF: lidar -> map
                    //
                    lidar_to_map_tf_pub_ptr->SendTransform(lidar_odometry, cloud_data.time);
                }
            }
        }

        rate.sleep();
    }

    return EXIT_SUCCESS;
}