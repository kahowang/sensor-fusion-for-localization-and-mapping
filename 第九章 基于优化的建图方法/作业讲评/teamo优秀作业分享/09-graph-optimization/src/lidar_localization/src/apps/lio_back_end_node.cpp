/*
 * @Description: LIO mapping backend ROS node
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
#include <ros/ros.h>
#include "glog/logging.h"

#include <lidar_localization/optimizeMap.h>
#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/mapping/back_end/lio_back_end_flow.hpp"

using namespace lidar_localization;

bool _need_optimize_map = false;

bool OptimizeMapCb(
    optimizeMap::Request &request, 
    optimizeMap::Response &response
) {
    _need_optimize_map = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "lio_back_end_node");
    ros::NodeHandle nh;

    std::string cloud_topic, odom_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");
    nh.param<std::string>("odom_topic", odom_topic, "/laser_odom");

    //
    // subscribe to:
    // 
    // a. undistorted Velodyne measurement:
    // b. lidar pose in map frame:
    // c. lidar odometry estimation:
    // d. loop close pose:
    // publish:
    // a. lidar odometry in map frame:
    // b. key frame pose and corresponding GNSS/IMU pose
    // c. optimized key frame sequence as trajectory
    std::shared_ptr<LIOBackEndFlow> lio_back_end_flow_ptr = std::make_shared<LIOBackEndFlow>(
        nh, cloud_topic, odom_topic
    );
    ros::ServiceServer service = nh.advertiseService(
        "optimize_map", OptimizeMapCb
    );

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        lio_back_end_flow_ptr->Run();

        if (_need_optimize_map) {
            lio_back_end_flow_ptr->ForceOptimize();
            lio_back_end_flow_ptr->SaveOptimizedOdometry();
            
            _need_optimize_map = false;
        }

        rate.sleep();
    }

    return EXIT_SUCCESS;
}