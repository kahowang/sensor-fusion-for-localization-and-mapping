#include <iostream>
#include <vector>
#include <string>
#include <list>
#include <sstream>
#include <fstream>
#include <iomanip>

/*ROS INCLUDE*/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <Eigen/Core>


using namespace std;

/*定义姿态结构体*/
struct pose
{
    double  timestamp;
    Eigen::Vector3d  pos ;
    Eigen::Quaterniond  q;
};


pose  pose_gt;                      //   GroundTruth  pose
pose  pose_ins;                    //    Estimate   pose

std::ofstream  gt;
std::ofstream  ins;

double  stamp_gt = 0;
double  stamp_ins = 0;

double  stamp_gt_init = 0;
double  stamp_ins_init = 0;

int  flag_gt = 1;
int  flag_ins = 1;

bool CreateFile(std::ofstream& ofs, std::string file_path) {
    ofs.open(file_path, std::ios::out);                          //  使用std::ios::out 可实现覆盖
    if(!ofs)
    {
        std::cout << "open csv file error " << std::endl;
        return  false;
    }
    return true;
}
/* write2txt   format  TUM*/
void WriteText(std::ofstream& ofs, pose data){
    ofs << std::fixed  << data.timestamp  << " " << data.pos.x() << " " <<  data.pos.y()  << " " <<   data.pos.z()  << " "
                                                                              <<  data.q.x()  << " "  <<  data.q.y()   <<" " <<  data.q.z()   << " "  <<  data.q.w()  <<  std::endl;
}

void  insCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(flag_ins){
        stamp_ins_init  =  msg->header.stamp.toSec();   
        flag_ins = 0;
    }
    pose_ins.timestamp =  msg->header.stamp.toSec()  -   stamp_ins_init;  //把时间戳转化成浮点型格式
    /*update  position*/
     pose_ins.pos.x()  =  msg->pose.pose.position.x ;
     pose_ins.pos.y()  =  msg->pose.pose.position.y ;
     pose_ins.pos.z()  =  msg->pose.pose.position.z ;
     /*update  orientation*/
     pose_ins.q.w()  =  msg->pose.pose.orientation.w;
     pose_ins.q.x()  =  msg->pose.pose.orientation.x;
     pose_ins.q.y()  =  msg->pose.pose.orientation.y;
     pose_ins.q.z()  =  msg->pose.pose.orientation.z;
     /*write to txt, fomat TUM*/
     WriteText(ins,pose_ins);
}

void  gtCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(flag_gt){
        stamp_gt_init = msg->header.stamp.toSec();   
        flag_gt = 0;
    }
        pose_gt.timestamp = msg->header.stamp.toSec() -  stamp_gt_init;
        /*update  position*/
        pose_gt.pos.x()  =  msg->pose.pose.position.x ;
        pose_gt.pos.y()  =  msg->pose.pose.position.y ;
        pose_gt.pos.z()  =  msg->pose.pose.position.z ;
        /*update  orientation*/
        pose_gt.q.w()  =  msg->pose.pose.orientation.w;
        pose_gt.q.x()  =  msg->pose.pose.orientation.x;
        pose_gt.q.y()  =  msg->pose.pose.orientation.y;
        pose_gt.q.z()  =  msg->pose.pose.orientation.z;
        /*write to txt, fomat TUM*/
        WriteText(gt,pose_gt);
}

int main(int argc, char  **argv)
{
    char path_gt[] = "/home/kaho/catkin_ws/src/data/gnss_ins_sim/recorder_gnss_ins_sim/gt.txt";
    char path_ins[] = "/home/kaho/catkin_ws/src/data/gnss_ins_sim/recorder_gnss_ins_sim/ins.txt";

    CreateFile(gt,path_gt );
    CreateFile(ins,path_ins );
    // init  node
    ros::init(argc, argv, "evaluate_node");
    //  create nodehandle
    ros::NodeHandle nh;
    //  create  subscriber
    ros::Subscriber sub_ins  =  nh.subscribe("/pose/estimation", 10, insCallback);
    ros::Subscriber sub_gt    =  nh.subscribe("/pose/ground_truth",10, gtCallback);

    ros::Rate loop_rate(100);      //  frequence 100hz
    while (ros::ok())
    {
        ros::spinOnce();                        //  goto  callback function
        loop_rate.sleep();
    }
    gt.close();
    ins.close();
    return 0;
}

