/*
 * @Description: GNSS-INS-Sim measurement preprocessing node
 * @Author: Ge Yao
 * @Date: 2020-11-21 15:39:24
 */
#include <ros/ros.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/data_pretreat/gnss_ins_sim_preprocess_flow.hpp"

using namespace lidar_localization;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "gnss_ins_sim_preprocess_node");
    ros::NodeHandle nh;

    // subscribe to
    // a. raw IMU measurement
    // b. raw GNSS measurement
    // c. raw odometer measurement
    // d. raw magnetometer measurement
    // e. reference trajectory
    // publish
    // a. synced IMU for ESKF/IEKF prediction
    // b. synced GNSS-odo-mag for ESKF/IEKF correction
    // b. synced reference trajectory for evo evaluation
    std::shared_ptr<GNSSINSSimPreprocessFlow> gnss_ins_sim_preprocess_flow_ptr = std::make_shared<
        GNSSINSSimPreprocessFlow
    >(
        nh
    );

    // pre-process IMU, GNSS & odo measurements at 100Hz:
    ros::Rate rate(400);
    while (ros::ok()) {
        ros::spinOnce();

        gnss_ins_sim_preprocess_flow_ptr->Run();

        rate.sleep();
    }

    return EXIT_SUCCESS;
}