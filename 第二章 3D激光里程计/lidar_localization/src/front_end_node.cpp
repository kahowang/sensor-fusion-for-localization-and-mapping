/*
 * @Description: front-end node, lidar odometry basic
 * @Author: Ge Yao
 * @Date: 2021-04-03
 */

#include <memory>

#include <ros/ros.h>

#include "lidar_localization/global_defination/global_defination.h"
#include "glog/logging.h"

#include "lidar_localization/front_end/front_end_flow.hpp"

#include <lidar_localization/saveMap.h>

using namespace lidar_localization;

bool save_map = false;

bool save_map_callback(
    saveMap::Request &request, 
    saveMap::Response &response
) {
    save_map = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;

    // register service save_map:
    ros::ServiceServer service = nh.advertiseService("save_map", save_map_callback);

    // register front end processing workflow:
    std::shared_ptr<FrontEndFlow> front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh);

    // process rate: 10Hz
    ros::Rate rate(10);

    while (ros::ok()) {
        ros::spinOnce();

        front_end_flow_ptr->Run();

        if (save_map) {
            front_end_flow_ptr->SaveMap();
            front_end_flow_ptr->PublishGlobalMap();

            save_map = false;
        }

        rate.sleep();
    }

    return EXIT_SUCCESS;
}