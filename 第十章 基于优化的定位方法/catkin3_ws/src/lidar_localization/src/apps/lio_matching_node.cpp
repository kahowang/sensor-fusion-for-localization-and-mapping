/*
 * @Description: frontend node for lio localization
 * @Author: Ge Yao
 * @Date: 2021-01-02 10:47:23
 */
#include <ros/ros.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/matching/front_end/matching_flow.hpp"

using namespace lidar_localization;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "lio_matching_node");
    ros::NodeHandle nh;

    // subscribe to:
    //     a. undistorted lidar measurements
    //     b. GNSS position

    // publish:
    //     a. relative pose estimation
    //     b. map matching estimation
    // this provides input to sliding window backend
    std::shared_ptr<MatchingFlow> matching_flow_ptr = std::make_shared<MatchingFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        matching_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}