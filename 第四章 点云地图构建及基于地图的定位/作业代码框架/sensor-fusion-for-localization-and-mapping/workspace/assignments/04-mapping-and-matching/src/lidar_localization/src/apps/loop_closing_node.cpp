/*
 * @Description: 闭环检测的node文件
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:56:27
 */
#include <ros/ros.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/mapping/loop_closing/loop_closing_flow.hpp"
#include <lidar_localization/saveScanContext.h>

using namespace lidar_localization;

bool save_scan_context = false;

bool SaveScanContextCb(saveScanContext::Request &request, saveScanContext::Response &response) {
    save_scan_context = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "loop_closing_node");
    ros::NodeHandle nh;

    // subscribe to:
    // a. key frame pose and corresponding GNSS/IMU pose from backend node
    // publish:
    // a. loop closure detection result for backend node:
    std::shared_ptr<LoopClosingFlow> loop_closing_flow_ptr = std::make_shared<LoopClosingFlow>(nh);

    // register service for scan context save:
    ros::ServiceServer service = nh.advertiseService("save_scan_context", SaveScanContextCb);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        loop_closing_flow_ptr->Run();

        if (save_scan_context) {
            save_scan_context = false;
            loop_closing_flow_ptr->Save();
        }

        rate.sleep();
    }

    return 0;
}