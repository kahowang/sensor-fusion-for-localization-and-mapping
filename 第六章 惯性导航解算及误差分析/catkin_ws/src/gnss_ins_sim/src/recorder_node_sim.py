#!/usr/bin/python

import os

import rospkg
import rospy
import rosbag

import math
import numpy as np
import pandas as pd

from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

# from gnss_ins_sim.geoparams import  geoparams
from std_msgs import msg

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

def get_gnss_ins_sim(motion_def_file, fs_imu, fs_gps):
    # set  origin  x y z
    origin_x =  2849886.61825
    origin_y =  -4656214.27294
    origin_z =  -3287190.60046
    '''
    Generate simulated GNSS/IMU data using specified trajectory.
    '''
    # set IMU model:
    D2R = math.pi/180.0
    # imu_err = 'low-accuracy'
    imu_err = {
        # 1. gyro:
        # a. random noise:
        # gyro angle random walk, deg/rt-hr
        'gyro_arw': np.array([0., 0., 0.]),
        # gyro bias instability, deg/hr
        'gyro_b_stability': np.array([0.0, 0.0, 0.0]),
        # gyro bias isntability correlation time, sec
        'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
        # b. deterministic error:
        'gyro_b': np.array([0.0, 0.0, 0.0]),
        'gyro_k': np.array([1.0, 1.0, 1.0]),
        'gyro_s': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        # 2. accel:
        # a. random noise:
        # accel velocity random walk, m/s/rt-hr
        'accel_vrw': np.array([0., 0., 0.]),
        # accel bias instability, m/s2
        'accel_b_stability': np.array([0., 0., 0.]),
        # accel bias isntability correlation time, sec
        'accel_b_corr': np.array([100.0, 100.0, 100.0]),
        # b. deterministic error:
        'accel_b': np.array([0.0, 0.0, 0.0]),
        'accel_k': np.array([1.0, 1.0, 1.0]),
        'accel_s': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        # 3. mag:
        'mag_si': np.eye(3) + np.random.randn(3, 3)*0.0, 
        'mag_hi': np.array([10.0, 10.0, 10.0])*0.0,
        'mag_std': np.array([0.1, 0.1, 0.1])
    }
    # generate GPS and magnetometer data:
    imu = imu_model.IMU(accuracy=imu_err, axis=9, gps=True)

    # init simulation:
    sim = ins_sim.Sim(
        # here sync GPS with other measurements as marker:
        [fs_imu, fs_imu, fs_imu],
        motion_def_file,
        ref_frame=1,
        imu=imu,
        mode=None,
        env=None,
        algorithm=None
    )
    
    # run:
    sim.run(1)

    # get simulated data:
    rospy.logwarn(
        'Simulated data size: Gyro-{}, Accel-{}, pos-{}'.format(
            len(sim.dmgr.get_data_all('gyro').data[0]),
            len(sim.dmgr.get_data_all('accel').data[0]),
            len(sim.dmgr.get_data_all('ref_pos').data)
        )
    )

    # calibration stages:
    step_size = 1.0 / fs_imu

    for i, (gyro, accel, ref_q, ref_pos, ref_vel) in enumerate(
        zip(
            # a. gyro:
            sim.dmgr.get_data_all('gyro').data[0], 
            # b. accel:
            sim.dmgr.get_data_all('accel').data[0],
            # c. gt_pose:
            sim.dmgr.get_data_all('ref_att_quat').data,                # groundtruth
            sim.dmgr.get_data_all('ref_pos').data,
            # d. true_vel :
            sim.dmgr.get_data_all('ref_vel').data
        )
    ):  

        yield {
            'stamp': i * step_size,
             'data': {
                    # a. gyro:
                    'gyro_x': gyro[0],
                    'gyro_y': gyro[1],
                    'gyro_z': gyro[2],
                    # b. accel:
                    'accel_x': accel[0],
                    'accel_y': accel[1],
                    'accel_z': accel[2],
                    # c. true orientation:
                    'gt_quat_w': ref_q[0],
                    'gt_quat_x':  ref_q[1],
                    'gt_quat_y':  ref_q[2],
                    'gt_quat_z':  ref_q[3],
                    # d. true position:
                    'gt_pos_x': ref_pos[0]  + origin_x,
                    'gt_pos_y': ref_pos[1]  + origin_y,
                    'gt_pos_z': ref_pos[2]  + origin_z,
                    # d. true velocity:
                    'gt_vel_x': ref_vel[0],
                    'gt_vel_y': ref_vel[1],
                    'gt_vel_z': ref_vel[2]
             }
         }
    sim.results()
    sim.plot(['ref_pos', 'ref_vel'], opt={'ref_pos': '3d'})
    

def gnss_ins_sim_recorder():
    """
    Record simulated GNSS/IMU data as ROS bag
    """
    # ensure gnss_ins_sim_node is unique:
    rospy.init_node('gnss_ins_sim_recorder_node')

    # parse params:
    motion_def_name = rospy.get_param('/gnss_ins_sim_recorder_node/motion_file')
    sample_freq_imu = rospy.get_param('/gnss_ins_sim_recorder_node/sample_frequency/imu')
    sample_freq_gps = rospy.get_param('/gnss_ins_sim_recorder_node/sample_frequency/gps')
    topic_name_imu = rospy.get_param('/gnss_ins_sim_recorder_node/topic_name_imu')
    topic_name_gt = rospy.get_param('/gnss_ins_sim_recorder_node/topic_name_gt')

    ## save scv
    output_path = rospy.get_param('/gnss_ins_sim_recorder_node/output_path')
    output_name = rospy.get_param('/gnss_ins_sim_recorder_node/output_name')
    ## save rosbag
    rosbag_output_path = rospy.get_param('/gnss_ins_sim_recorder_node/output_path')
    rosbag_output_name = rospy.get_param('/gnss_ins_sim_recorder_node/output_name')

    # generate simulated data:
    motion_def_path = os.path.join(
        rospkg.RosPack().get_path('gnss_ins_sim'), 'config', 'motion_def', motion_def_name
    )
    imu_simulator = get_gnss_ins_sim(
        # motion def file:
        motion_def_path,
        # gyro-accel/gyro-accel-mag sample rate:
        sample_freq_imu,
        # GPS sample rate:
        sample_freq_gps
    )

    # write as csv:
    # data = pd.DataFrame(
    #     list(imu_simulator)
    # )
    # data.to_csv(
    #     os.path.join(output_path, output_name)
    # )
    
    #write  rosbag
    with rosbag.Bag(
        os.path.join(rosbag_output_path, rosbag_output_name), 'w'
    ) as bag:
        # get timestamp base:
        timestamp_start = rospy.Time.now()

        for measurement in imu_simulator:
            # init:
            msg_imu = Imu()
            # a. set header:
            msg_imu.header.frame_id = 'inertial'
            msg_imu.header.stamp = timestamp_start + rospy.Duration.from_sec(measurement['stamp'])
            # b. set orientation estimation:
            msg_imu.orientation.x = 0.0
            msg_imu.orientation.y = 0.0
            msg_imu.orientation.z = 0.0
            msg_imu.orientation.w = 1.0
            # c. gyro:
            msg_imu.angular_velocity.x = measurement['data']['gyro_x']
            msg_imu.angular_velocity.y = measurement['data']['gyro_y']
            msg_imu.angular_velocity.z = measurement['data']['gyro_z']
            msg_imu.linear_acceleration.x = measurement['data']['accel_x']
            msg_imu.linear_acceleration.y = measurement['data']['accel_y']
            msg_imu.linear_acceleration.z = measurement['data']['accel_z']

            # write:
            bag.write(topic_name_imu, msg_imu, msg_imu.header.stamp)

            # write:
            bag.write(topic_name_imu, msg_imu, msg_imu.header.stamp)

            # init :  groundtruth
            msg_odom = Odometry()
            # a.set header:
            msg_odom.header.frame_id = 'inertial'
            msg_odom.header.stamp =    msg_imu.header.stamp
            # b.set gt_pose
            msg_odom.pose.pose.position.x =  measurement['data']['gt_pos_x']   
            msg_odom.pose.pose.position.y =  measurement['data']['gt_pos_y']  
            msg_odom.pose.pose.position.z =  measurement['data']['gt_pos_z']   

            msg_odom.pose.pose.orientation.w = measurement['data']['gt_quat_w']
            msg_odom.pose.pose.orientation.x = measurement['data']['gt_quat_x']
            msg_odom.pose.pose.orientation.y = measurement['data']['gt_quat_y']
            msg_odom.pose.pose.orientation.z = measurement['data']['gt_quat_z']
            #c.set gt_vel
            msg_odom.twist.twist.linear.x = measurement['data']['gt_vel_x']
            msg_odom.twist.twist.linear.y = measurement['data']['gt_vel_y']
            msg_odom.twist.twist.linear.z = measurement['data']['gt_vel_z']

            # write 
            bag.write(topic_name_gt, msg_odom, msg_odom.header.stamp)

if __name__ == '__main__':
    try:
        gnss_ins_sim_recorder()
    except rospy.ROSInterruptException:
        pass