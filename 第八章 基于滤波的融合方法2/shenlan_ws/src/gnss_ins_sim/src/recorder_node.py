#!/usr/bin/python

import os

import rospkg
import rospy
import rosbag

import math
import numpy as np
import pandas as pd

from gnss_ins_sim.geoparams import geoparams
from gnss_ins_sim.geoparams import geomag
from gnss_ins_sim.attitude import attitude

from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


D2R = math.pi / 180.0
R2D = 180.0 / math.pi


config = {
    'imu': {
        'no_error': {
            # 1. gyro:
            # a. random noise:
            # gyro angle random walk, deg/rt-hr
            'gyro_arw': np.array([0.00, 0.00, 0.00]) * D2R/60,
            # gyro bias instability, deg/hr
            'gyro_b_stability': np.array([0.00, 0.0, 0.0]) * D2R/3600.0,
            # gyro bias isntability correlation time, sec
            #'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
            # b. deterministic error:
            'gyro_b': np.array([0.00, 0.00, 0.00]),
            'gyro_k': np.array([1.00, 1.00, 1.00]),
            'gyro_s': np.array([0.00, 0.00, 0.00, 0.00, 0.00, 0.00]),
            # 2. accel:
            # a. random noise:
            # accel velocity random walk, m/s/rt-hr
            'accel_vrw': np.array([0.00, 0.00, 0.00]) / 60,
            # accel bias instability, m/s2
            'accel_b_stability': np.array([0.00, 0.00, 0.00]),
            # accel bias isntability correlation time, sec
            #'accel_b_corr': np.array([100.0, 100.0, 100.0]),
            # b. deterministic error:
            'accel_b': np.array([0.00, 0.00, 0.00]),
            'accel_k': np.array([1.00, 1.00, 1.00]),
            'accel_s': np.array([0.00, 0.00, 0.00, 0.00, 0.00, 0.00]),
        },
        'high_accuracy': {
            # 1. gyro:
            # a. random noise:
            # gyro angle random walk, deg/rt-hr
            'gyro_arw': np.array([2.0e-3, 2.0e-3, 2.0e-3]) * D2R/60,
            # gyro bias instability, deg/hr
            'gyro_b_stability': np.array([0.1, 0.1, 0.1]) * D2R/3600.0,
            # gyro bias isntability correlation time, sec
            'gyro_b_corr': np.array([100.0, 100.0, 100.0]),
            # b. deterministic error:
            'gyro_b': np.array([0.0, 0.0, 0.0]) * D2R,
            'gyro_k': np.array([1.00, 1.00, 1.00]),
            'gyro_s': np.array([0.00, 0.00, 0.00, 0.00, 0.00, 0.00]),
            # 2. accel:
            # a. random noise:
            # accel velocity random walk, m/s/rt-hr
            'accel_vrw': np.array([2.5e-5, 2.5e-5, 2.5e-5]) / 60,
            # accel bias instability, m/s2
            'accel_b_stability': np.array([3.6e-6, 3.6e-6, 3.6e-6]),
            # accel bias isntability correlation time, sec
            'accel_b_corr': np.array([100.0, 100.0, 100.0]),
            # b. deterministic error:
            'accel_b': np.array([0.0e-3, 0.0e-3, 0.0e-3]),
            'accel_k': np.array([1.00, 1.00, 1.00]),
            'accel_s': np.array([0.00, 0.00, 0.00, 0.00, 0.00, 0.00]),
        }
    },
    'mag': {
        'no_error': {
            'mag_si': np.eye(3) + np.random.randn(3, 3)*0.0,
            'mag_hi': np.array([10.0, 10.0, 10.0])*0.0,
            'mag_std': np.array([0.00, 0.00, 0.00])
        },
        'high_accuracy': {
            'mag_si': np.eye(3) + np.random.randn(3, 3)*0.0,
            'mag_hi': np.array([10.0, 10.0, 10.0])*0.0,
            'mag_std': np.array([0.001, 0.001, 0.001])
        },
        'mid_accuracy': {
            'mag_si': np.eye(3) + np.random.randn(3, 3)*0.0,
            'mag_hi': np.array([10.0, 10.0, 10.0])*0.0,
            'mag_std': np.array([0.01, 0.01, 0.01])
        },
        'low_accuracy': {
            'mag_si': np.eye(3) + np.random.randn(3, 3)*0.0, 
            'mag_hi': np.array([10.0, 10.0, 10.0])*0.0,
            'mag_std': np.array([0.1, 0.1, 0.1])
        }
    },
    'gps': {
        'no_error': {
            'stdp': np.array([0.0, 0.0, 0.0]),
            'stdv': np.array([0.0, 0.0, 0.0])
        },
        'high_accuracy': {
            'stdp': np.array([0.10, 0.10, 0.10]),
            'stdv': np.array([0.01, 0.01, 0.01])
        },
        'mid_accuracy': {
            'stdp': np.array([0.50, 0.50, 0.50]),
            'stdv': np.array([0.02, 0.02, 0.02])
        },
        'low_accuracy': {
            'stdp': np.array([1.00, 1.00, 1.00]),
            'stdv': np.array([0.05, 0.05, 0.05])
        }
    },
    'odo': {
        'no_error': {
            'scale': 1.00,
            'stdv': 0.0
        },
        'high_accuracy': {
            'scale': 1.00,
            'stdv': 0.01
        },
        'mid_accuracy': {
            'scale': 1.00,
            'stdv': 0.05
        },
        'low_accuracy': {
            'scale': 1.00,
            'stdv': 0.10
        }
    }
}


def get_init_pose(stamp, motion_def_file):
    """
    Get init pose from motion def file as ROS nav_msgs::Odometry
    """
    # parse:
    (
        lat, lon, alt,
        vx, vy, vz,
        yaw, pitch, roll
    ) = np.genfromtxt(
        motion_def_file, delimiter=',', skip_header=True, max_rows=1
    )

    (
        rm, rn,
        g,
        sl, cl,
        w_ie
    ) = geoparams.geo_param(
        np.array([lat, lon, alt])
    )

    gm = geomag.GeoMag("WMM.COF")
    # units in nT and deg:
    geo_mag = gm.GeoMag(lat/D2R, lon/D2R, alt) 
    # nT to uT:
    (
        B_E, B_N, B_U
    ) = np.array([geo_mag.bx, geo_mag.by, geo_mag.bz]) / 1000.0
          
    rospy.logwarn(
        """
        Earth Params:
        \tRm: {}
        \tRn: {}
        \tG: {}
        \tsin(Lat): {}
        \tcos(Lat): {}
        \tw_ie: {}
        \tMag:
        \t\tB_E:{}
        \t\tB_N:{}
        \t\tB_U:{}
        """.format(
            rm, rn,
            g,
            sl, cl,
            w_ie,
            B_E, B_N, B_U
        )
    )

    # init:
    init_pose_msg = Odometry()

    # set header:
    init_pose_msg.header.stamp = stamp
    init_pose_msg.header.frame_id = '/map'
    init_pose_msg.child_frame_id = '/imu_link'

    # a. navigation frame, position:
    """
    (
        init_pose_msg.pose.pose.position.x, 
        init_pose_msg.pose.pose.position.y, 
        init_pose_msg.pose.pose.position.z
    ) = geoparams.lla2ecef(
        [
            lat*D2R, lon*D2R, alt
        ]
    )
    """
    (
        init_pose_msg.pose.pose.position.x, 
        init_pose_msg.pose.pose.position.y, 
        init_pose_msg.pose.pose.position.z
    ) = [
        0.0, 0.0, 0.0
    ]
    # b. navigation frame, orientation
    (
        init_pose_msg.pose.pose.orientation.w, 
        init_pose_msg.pose.pose.orientation.x, 
        init_pose_msg.pose.pose.orientation.y, 
        init_pose_msg.pose.pose.orientation.z
    ) = attitude.euler2quat(
        np.asarray(
            [yaw*D2R, pitch*D2R, roll*D2R]
        ),
        rot_seq='zyx'
    )

    # c. body frame, velocity:
    (
        init_pose_msg.twist.twist.linear.x, 
        init_pose_msg.twist.twist.linear.y, 
        init_pose_msg.twist.twist.linear.z
    ) = (
        vx, vy, vz
    )

    # finally:
    return init_pose_msg


def get_imu_msg(stamp, gyro, accel):
    """
    Get IMU measurement as ROS sensor_msgs::Imu
    """
    # init:
    imu_msg = Imu()

    # a. set header:
    imu_msg.header.stamp = stamp
    imu_msg.header.frame_id = '/imu_link'

    # b. set orientation estimation:
    imu_msg.orientation.w = 1.0
    imu_msg.orientation.x = 0.0
    imu_msg.orientation.y = 0.0
    imu_msg.orientation.z = 0.0

    # c. gyro:
    (
        imu_msg.angular_velocity.x,
        imu_msg.angular_velocity.y,
        imu_msg.angular_velocity.z
    ) = gyro

    # d. accel:
    (
        imu_msg.linear_acceleration.x,
        imu_msg.linear_acceleration.y,
        imu_msg.linear_acceleration.z 
    ) = accel
    
    # finally:
    return imu_msg 


def get_mag_msg(stamp, mag):
    """
    Get magnetometer measurement as ROS sensor_msgs::MagneticField
    """
    # init:
    mag_msg = MagneticField()

    # a. set header:
    mag_msg.header.stamp = stamp
    mag_msg.header.frame_id = '/imu_link'

    # b. mag:
    (
        mag_msg.magnetic_field.x,
        mag_msg.magnetic_field.y,
        mag_msg.magnetic_field.z
    ) = mag

    # finally:
    return mag_msg


def get_gps_pos_msg(stamp, gps_pos):
    """
    Get GPS position in LLA as ROS sensor_msgs::NavSatFix
    """
    # init:
    gps_pos_msg = NavSatFix()

    # set header:
    gps_pos_msg.header.stamp = stamp
    gps_pos_msg.header.frame_id = '/imu_link'

    # set LLA:
    (
        gps_pos_msg.latitude,
        gps_pos_msg.longitude,
        gps_pos_msg.altitude
    ) = (
        gps_pos[0] * R2D,
        gps_pos[1] * R2D,
        gps_pos[2]
    )

    return gps_pos_msg


def get_gps_vel_msg(stamp, gps_vel):
    """
    Get vehicle speed in NED frame as ROS geometry_msgs::TwistStamped
    """
    # init:
    gps_vel_msg = TwistStamped()

    # set header:
    gps_vel_msg.header.stamp = stamp
    gps_vel_msg.header.frame_id = '/imu_link'

    # set NEU speed:
    (
        gps_vel_msg.twist.linear.x,
        gps_vel_msg.twist.linear.y,
        gps_vel_msg.twist.linear.z
    ) = gps_vel 

    return gps_vel_msg


def get_odo_msg(stamp, odo):
    """
    Get vehicle speed in body frame as ROS geometry_msgs::TwistStamped
    """
    # init:
    odo_msg = TwistStamped()

    # set header:
    odo_msg.header.stamp = stamp
    odo_msg.header.frame_id = '/imu_link'

    # set NEU speed:
    (
        odo_msg.twist.linear.x,
        odo_msg.twist.linear.y,
        odo_msg.twist.linear.z
    ) = odo 

    return odo_msg


def get_pose_msg(stamp, ref_pos, ref_vel, ref_att_quat):
    """
    Get reference pose in (P, V, Orientation) as ROS nav_msgs::Odometry
    """
    # init:
    pose_msg = Odometry()

    # set header:
    pose_msg.header.stamp = stamp
    pose_msg.header.frame_id = '/map'
    pose_msg.child_frame_id = '/map'

    # a. navigation frame, position:
    (
        pose_msg.pose.pose.position.x, 
        pose_msg.pose.pose.position.y, 
        pose_msg.pose.pose.position.z
    ) = (
        ref_pos[0] * R2D,
        ref_pos[1] * R2D,
        ref_pos[2]
    )
    
    # b. navigation frame, orientation
    (
        pose_msg.pose.pose.orientation.w, 
        pose_msg.pose.pose.orientation.x, 
        pose_msg.pose.pose.orientation.y, 
        pose_msg.pose.pose.orientation.z
    ) = ref_att_quat

    # c. body frame, velocity:
    (
        pose_msg.twist.twist.linear.x, 
        pose_msg.twist.twist.linear.y, 
        pose_msg.twist.twist.linear.z
    ) = ref_vel
    
    # finally:
    return pose_msg 


def get_gnss_ins_sim(
    motion_def_file, 
    fs_imu, fs_gps,
    imu_error_level = 'high_accuracy',
    mag_error_level = 'mid_accuracy', gps_error_level = 'mid_accuracy', odo_error_level = 'mid_accuracy'
):
    """
    Generate simulated 
        IMU, 
        magnetometer, 
        GPS, 
        odometer 
    data using specified trajectory
    """ 
    #
    # set IMU model:
    #

    # for error-state Kalman filter observability & the degree of observability analysis
    # remove deterministic error and random noise:
    imu_err = config['imu'][imu_error_level].copy()
    imu_err.update(config['mag'][mag_error_level])
    
    gps_err = config['gps'][gps_error_level]
    odo_err = config['odo'][odo_error_level]

    # show device error level config:
    for k in imu_err:
        rospy.logwarn("{}: {}".format(k, imu_err[k]))
    for k in gps_err:
        rospy.logwarn("{}: {}".format(k, gps_err[k]))
    for k in odo_err:
        rospy.logwarn("{}: {}".format(k, odo_err[k]))

    # generate GPS and magnetometer data:
    imu = imu_model.IMU(
        accuracy=imu_err, 
        axis=9, 
        gps=True, gps_opt=gps_err,
        odo=True, odo_opt=odo_err
    )

    # init simulation:
    sim = ins_sim.Sim(
        # here sync GPS with other measurements as marker:
        [fs_imu, fs_imu, fs_imu],
        motion_def_file,
        # use NED frame:
        ref_frame=0,
        imu=imu,
        mode=None,
        env=None,
        algorithm=None
    )
    
    # run:
    sim.run(1)

    # get simulated data:
    rospy.logwarn(
        """
        GNSS-INS-Sim Summary:
        \tIMU Measurement:
        \t\tGyroscope: {}
        \t\tAccelerometer: {}
        \t\tMagnetometer: {}
        \tGNSS Measurement:
        \t\tLLA: {}
        \t\tNED Velocity: {}
        \tOdometry:
        \t\tVelocity: {}
        \tReference Trajectory:
        \t\tPosition: {}
        \t\tVelocity: {}
        \t\tOrientation in Quaternion: {}
        """.format(
            # a. IMU:
            repr(sim.dmgr.get_data_all('gyro').data[0].shape),
            repr(sim.dmgr.get_data_all('accel').data[0].shape),
            repr(sim.dmgr.get_data_all('mag').data[0].shape),
            # b. GNSS:
            repr(sim.dmgr.get_data_all('gps').data[0][:, :3].shape),
            repr(sim.dmgr.get_data_all('gps').data[0][:, 3:].shape),
            # c. odometry:
            repr(sim.dmgr.get_data_all('odo').data[0].shape),
            # d. reference trajectory:
            repr(sim.dmgr.get_data_all('ref_pos').data.shape),
            repr(sim.dmgr.get_data_all('ref_vel').data.shape),
            repr(sim.dmgr.get_data_all('ref_att_quat').data.shape),
        )
    )

    # init timer:
    timestamp_start = rospy.Time.now()
    STEP_SIZE = 1.0 / fs_imu

    # yield init pose:
    init_pose_msg = get_init_pose(timestamp_start, motion_def_file)
    yield init_pose_msg

    # yield measurements:
    for i, (
        # a. IMU:
        gyro, accel, mag,
        # b. GNSS:
        gps_pos, gps_vel,
        # c. odometry:
        odo_x,
        # d. reference trajectory:
        ref_pos, ref_vel, ref_att_quat
    ) in enumerate(
        zip(
            # a. IMU:
            sim.dmgr.get_data_all('gyro').data[0],
            sim.dmgr.get_data_all('accel').data[0],
            sim.dmgr.get_data_all('mag').data[0], 
            # b. GNSS:
            sim.dmgr.get_data_all('gps').data[0][:, :3],
            sim.dmgr.get_data_all('gps').data[0][:, 3:],
            # c. odometry velocity:
            sim.dmgr.get_data_all('odo').data[0],
            # d. reference trajectory:
            sim.dmgr.get_data_all('ref_pos').data,
            sim.dmgr.get_data_all('ref_vel').data,
            sim.dmgr.get_data_all('ref_att_quat').data
        )
    ):  
        # generate timestamp:
        stamp = timestamp_start + rospy.Duration.from_sec(i * STEP_SIZE) 

        # a. IMU:
        imu_msg = get_imu_msg(
            stamp, gyro, accel
        )

        # b. magnetometer:
        mag_msg = get_mag_msg(
            stamp, mag
        )

        # c. GNSS:
        gps_pos_msg = get_gps_pos_msg(
            stamp, gps_pos
        )
        gps_vel_msg = get_gps_vel_msg(
            stamp, gps_vel
        )

        # d. odometer:
        odo_msg = get_odo_msg(
            stamp, 
            # measurement is only available at forward direction
            np.array([odo_x, 0.0, 0.0])
        )

        # e. reference trajectory:
        reference_pose_msg = get_pose_msg(
            stamp, ref_pos, ref_vel, ref_att_quat
        )

        yield (
            imu_msg, mag_msg,
            gps_pos_msg, gps_vel_msg, 
            odo_msg,
            reference_pose_msg
        )


def gnss_ins_sim_recorder():
    """
    Record simulated sensor data as ROS bag
    """
    # ensure gnss_ins_sim_node is unique:
    rospy.init_node('gnss_ins_sim_recorder_node')

    #
    # parse params:
    #
    # a. motion file:
    motion_def_name = rospy.get_param('/gnss_ins_sim_recorder_node/motion_file')
    # b. sample frequency:
    sample_freq_imu = rospy.get_param('/gnss_ins_sim_recorder_node/sample_frequency/imu')
    sample_freq_gps = rospy.get_param('/gnss_ins_sim_recorder_node/sample_frequency/gps')
    # c. device specification:
    imu_error_level = rospy.get_param('/gnss_ins_sim_recorder_node/device_error_level/imu')
    mag_error_level = rospy.get_param('/gnss_ins_sim_recorder_node/device_error_level/mag')
    gps_error_level = rospy.get_param('/gnss_ins_sim_recorder_node/device_error_level/gps')
    odo_error_level = rospy.get_param('/gnss_ins_sim_recorder_node/device_error_level/odo')
    # d. ROS bag topic names:
    # 1. IMU:
    topic_name_imu = rospy.get_param('/gnss_ins_sim_recorder_node/topic_name/imu')
    # 2. magnetometer:
    topic_name_mag = rospy.get_param('/gnss_ins_sim_recorder_node/topic_name/mag')
    # 3. GNSS
    topic_name_gps_pos = rospy.get_param('/gnss_ins_sim_recorder_node/topic_name/gps_pos')
    # 4. GNSS/IMU
    topic_name_gps_vel = rospy.get_param('/gnss_ins_sim_recorder_node/topic_name/gps_vel')
    # 5. odometer:
    topic_name_odo = rospy.get_param('/gnss_ins_sim_recorder_node/topic_name/odo')
    # 6. reference trajectory
    topic_name_init_pose = rospy.get_param('/gnss_ins_sim_recorder_node/topic_name/init_pose')
    topic_name_reference_trajectory = rospy.get_param('/gnss_ins_sim_recorder_node/topic_name/reference_trajectory')
    # e. output path:
    rosbag_output_path = rospy.get_param('/gnss_ins_sim_recorder_node/output_path')
    rosbag_output_name = rospy.get_param('/gnss_ins_sim_recorder_node/output_name')

    #
    # generate simulated data:
    #
    # a. identify motion file:
    motion_def_path = os.path.join(
        rospkg.RosPack().get_path('gnss_ins_sim'), 'config', 'motion_def', motion_def_name
    )
    # b. init simulator:
    imu_simulator = get_gnss_ins_sim(
        # motion def file:
        motion_def_path,
        # gyro-accel/gyro-accel-mag sample rate:
        sample_freq_imu,
        # GPS sample rate:
        sample_freq_gps,
        # IMU error level:
        imu_error_level,
        # other error levels:
        mag_error_level, gps_error_level, odo_error_level
    )

    #
    # write to ROS bag:
    #
    with rosbag.Bag(
        os.path.join(rosbag_output_path, rosbag_output_name), 'w'
    ) as bag:
        # write init pose:
        init_pose_msg = next(imu_simulator)
        bag.write(topic_name_init_pose, init_pose_msg, init_pose_msg.header.stamp)

        # write measurements:
        for measurement in imu_simulator:
            # parse:
            (
                imu_msg, mag_msg,
                gps_pos_msg, gps_vel_msg, 
                odo_msg,
                reference_pose_msg
            ) = measurement

            # write:
            # a. IMU:
            bag.write(topic_name_imu, imu_msg, imu_msg.header.stamp)
            # b. mag:
            bag.write(topic_name_mag, mag_msg, mag_msg.header.stamp)
            # c. GNSS:
            bag.write(topic_name_gps_pos, gps_pos_msg, gps_pos_msg.header.stamp)
            bag.write(topic_name_gps_vel, gps_vel_msg, gps_vel_msg.header.stamp)
            # d. odo:
            bag.write(topic_name_odo, odo_msg, odo_msg.header.stamp)
            # e. reference pose:
            bag.write(topic_name_reference_trajectory, reference_pose_msg, reference_pose_msg.header.stamp)


if __name__ == '__main__':
    try:
        gnss_ins_sim_recorder()
    except rospy.ROSInterruptException:
        pass