# GNSS-INS-Sim for IMU Kalman Filter Fusion

ROS wrapper package for [gnss-ins-sim](https://github.com/Aceinna/gnss-ins-sim) data `ROS bag recording`, which includes:

---

# Data & Corresponding ROS Message

* IMU 
    
    * `Gyroscope and Accelerometer` as `ROS sensor_msgs::Imu`

    * `Magnetometer` as `ROS sensor_msgs::MagneticField`

* GNSS

    * `Latitude, Longitude and Altitude` as `ROS sensor_msgs::NavSatFix`

    * `Vehicle Speed in NED frame` as `ROS geometry_msgs::TwistStamped`

* Odometer

    * `Vehicle Speed (Only in X/Forward Direction) in Body Frame` as `ROS geometry_msgs::TwistStamped`

* Reference Trajectory

    * `Reference Pose (P, V, Orientation) Series` as `ROS nav_msgs::Odometry`

---

# Sample ROS Bag Output

```bash
$ rosbag info virtual_proving_ground.bag 

path:        virtual_proving_ground.bag
version:     2.0
duration:    1:37s (97s)
start:       Nov 21 2020 07:34:38.69 (1605944078.69)
end:         Nov 21 2020 07:36:16.68 (1605944176.68)
size:        16.6 MB
messages:    58801
compression: none [22/22 chunks]
types:       geometry_msgs/TwistStamped [98d34b0043a2093cf9d9345ab6eef12e]
             nav_msgs/Odometry          [cd5e73d190d741a2f92e81eda573aca7]
             sensor_msgs/Imu            [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/MagneticField  [2f3b0b43eed0c9501de0fa3ff89a45aa]
             sensor_msgs/NavSatFix      [2d3a8cd499b9b4a0249fb98fd05cfa48]
topics:      /init_pose               1 msg     : nav_msgs/Odometry         
             /reference_pose       9800 msgs    : nav_msgs/Odometry         
             /sim/sensor/gps/fix   9800 msgs    : sensor_msgs/NavSatFix     
             /sim/sensor/gps/vel   9800 msgs    : geometry_msgs/TwistStamped
             /sim/sensor/imu       9800 msgs    : sensor_msgs/Imu           
             /sim/sensor/imu/mag   9800 msgs    : sensor_msgs/MagneticField 
             /sim/sensor/odo       9800 msgs    : geometry_msgs/TwistStamped

```