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

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

def get_gnss_ins_sim(motion_def_file, fs_imu, fs_gps):
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
        # 'gyro_arw': np.array([0.75, 0.75, 0.75]),
        'gyro_arw': np.array([0.00, 0.00, 0.00]),
        # gyro bias instability, deg/hr
        # 'gyro_b_stability': np.array([10.0, 10.0, 10.0]),
        'gyro_b_stability': np.array([0.0, 0.0, 0.0]),
        # gyro bias isntability correlation time, sec
        # 'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
        # b. deterministic error:
        'gyro_b': np.array([36.00, 36.00, 36.00]),
        'gyro_k': np.array([0.98, 0.98, 0.98]),
        'gyro_s': np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01]),
        # 2. accel:
        # a. random noise:
        # accel velocity random walk, m/s/rt-hr
        'accel_vrw': np.array([0.05, 0.05, 0.05]),
        # accel bias instability, m/s2
        'accel_b_stability': np.array([2.0e-4, 2.0e-4, 2.0e-4]),
        # accel bias isntability correlation time, sec
        'accel_b_corr': np.array([100.0, 100.0, 100.0]),
        # b. deterministic error:
        'accel_b': np.array([0.01, 0.01, 0.01]),
        'accel_k': np.array([0.98, 0.98, 0.98]),
        'accel_s': np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01]),
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
        'Simulated data size: Gyro-{}, Accel-{}, GPS Availability-{}'.format(
            len(sim.dmgr.get_data_all('gyro').data[0]),
            len(sim.dmgr.get_data_all('accel').data[0]),
            len(sim.dmgr.get_data_all('gps_visibility').data)
        )
    )

    # calibration stages:
    STEP_SIZE = 1.0 / fs_imu

    STAGES = [
        'rotate_z_pos', 'rotate_z_neg', 'rotate_y_pos', 'rotate_y_neg', 'rotate_x_pos', 'rotate_x_neg',
        'static_z_pos', 'static_z_neg', 'static_y_pos', 'static_y_neg', 'static_x_pos', 'static_x_neg'
    ]
    stage_id = 0

    last_gps_visibility = True 
    curr_gps_visibility = True
    for i, (gyro, accel, ref_gyro, ref_accel, gps_visibility) in enumerate(
        zip(
            # a. gyro:
            sim.dmgr.get_data_all('gyro').data[0], 
            # b. accel:
            sim.dmgr.get_data_all('accel').data[0],
            # c. ref gyro:
            sim.dmgr.get_data_all('ref_gyro').data, 
            # d. ref accel:
            sim.dmgr.get_data_all('ref_accel').data,
            # e. gps visibility as marker:
            sim.dmgr.get_data_all('gps_visibility').data
        )
    ):  
        last_gps_visibility = curr_gps_visibility
        curr_gps_visibility = gps_visibility

        if (not last_gps_visibility) and curr_gps_visibility:
            stage_id += 1
        
        if not curr_gps_visibility:
            continue

        yield {
            'stamp': i * STEP_SIZE,
            # a. gyro:
            'gyro_x': gyro[0],
            'gyro_y': gyro[1],
            'gyro_z': gyro[2],
            # b. accel:
            'accel_x': accel[0],
            'accel_y': accel[1],
            'accel_z': accel[2],
            # c. gyro:
            'ref_gyro_x': ref_gyro[0],
            'ref_gyro_y': ref_gyro[1],
            'ref_gyro_z': ref_gyro[2],
            # d. accel:
            'ref_accel_x': ref_accel[0],
            'ref_accel_y': ref_accel[1],
            'ref_accel_z': ref_accel[2],
            # e. stage:
            'stage': STAGES[stage_id]
        }

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
    topic_name_imu = rospy.get_param('/gnss_ins_sim_recorder_node/topic_name')
    output_path = rospy.get_param('/gnss_ins_sim_recorder_node/output_path')
    output_name = rospy.get_param('/gnss_ins_sim_recorder_node/output_name')

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
    data = pd.DataFrame(
        list(imu_simulator)
    )
    data.to_csv(
        os.path.join(output_path, output_name)
    )

if __name__ == '__main__':
    try:
        gnss_ins_sim_recorder()
    except rospy.ROSInterruptException:
        pass