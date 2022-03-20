/*
 * @Description: backend node for lio localization
 * @Author: Ge Yao
 * @Date: 2021-01-02 10:47:23
 */
#include <ros/ros.h>

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/matching/back_end/sliding_window_flow.hpp"
#include "lidar_localization/saveOdometry.h"

#include "glog/logging.h"

using namespace lidar_localization;

bool save_odometry = false;

bool SaveOdometryCb(saveOdometry::Request &request, saveOdometry::Response &response) {
    save_odometry = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "sliding_window_node");
    ros::NodeHandle nh;

    //
    // subscribe to:
    // 
    // a. lidar odometry
    // b. map matching odometry

    //
    // publish:
    // 
    // a. optimized key frame sequence as trajectory
    std::shared_ptr<SlidingWindowFlow> sliding_window_flow_ptr = std::make_shared<SlidingWindowFlow>(nh);

    // register service for optimized trajectory save:
    ros::ServiceServer service = nh.advertiseService("save_odometry", SaveOdometryCb);

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        sliding_window_flow_ptr->Run();

        if (save_odometry) {
            save_odometry = false;
            sliding_window_flow_ptr->SaveOptimizedTrajectory();
        }

        rate.sleep();
    }

    return EXIT_SUCCESS;
}