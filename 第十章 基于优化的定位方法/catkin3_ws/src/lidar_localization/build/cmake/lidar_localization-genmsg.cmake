# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "lidar_localization: 6 messages, 1 services")

set(MSG_I_FLAGS "-Ilidar_localization:/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Inav_msgs:/opt/ros/melodic/share/nav_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(lidar_localization_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVelMag.msg" NAME_WE)
add_custom_target(_lidar_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lidar_localization" "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVelMag.msg" "geometry_msgs/Vector3:geometry_msgs/Point:std_msgs/Header"
)

get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/LidarMeasurement.msg" NAME_WE)
add_custom_target(_lidar_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lidar_localization" "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/LidarMeasurement.msg" "nav_msgs/Odometry:sensor_msgs/PointCloud2:sensor_msgs/PointField:geometry_msgs/TwistWithCovariance:geometry_msgs/Vector3:geometry_msgs/Pose:geometry_msgs/PoseWithCovariance:std_msgs/Header:geometry_msgs/Twist:sensor_msgs/Imu:geometry_msgs/Point:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/IMUGNSSMeasurement.msg" NAME_WE)
add_custom_target(_lidar_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lidar_localization" "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/IMUGNSSMeasurement.msg" "std_msgs/Header:geometry_msgs/TwistWithCovariance:geometry_msgs/Vector3:geometry_msgs/Pose:geometry_msgs/PoseWithCovariance:nav_msgs/Odometry:geometry_msgs/Twist:sensor_msgs/Imu:geometry_msgs/Point:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVel.msg" NAME_WE)
add_custom_target(_lidar_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lidar_localization" "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVel.msg" "geometry_msgs/Vector3:geometry_msgs/Point:std_msgs/Header"
)

get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/srv/saveOdometry.srv" NAME_WE)
add_custom_target(_lidar_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lidar_localization" "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/srv/saveOdometry.srv" ""
)

get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/ESKFStd.msg" NAME_WE)
add_custom_target(_lidar_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lidar_localization" "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/ESKFStd.msg" "std_msgs/Header"
)

get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/EKFStd.msg" NAME_WE)
add_custom_target(_lidar_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lidar_localization" "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/EKFStd.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVelMag.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_localization
)
_generate_msg_cpp(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/LidarMeasurement.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_localization
)
_generate_msg_cpp(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/IMUGNSSMeasurement.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_localization
)
_generate_msg_cpp(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_localization
)
_generate_msg_cpp(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/ESKFStd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_localization
)
_generate_msg_cpp(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/EKFStd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_localization
)

### Generating Services
_generate_srv_cpp(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/srv/saveOdometry.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_localization
)

### Generating Module File
_generate_module_cpp(lidar_localization
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_localization
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(lidar_localization_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(lidar_localization_generate_messages lidar_localization_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVelMag.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_cpp _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/LidarMeasurement.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_cpp _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/IMUGNSSMeasurement.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_cpp _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVel.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_cpp _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/srv/saveOdometry.srv" NAME_WE)
add_dependencies(lidar_localization_generate_messages_cpp _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/ESKFStd.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_cpp _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/EKFStd.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_cpp _lidar_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_localization_gencpp)
add_dependencies(lidar_localization_gencpp lidar_localization_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_localization_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVelMag.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_localization
)
_generate_msg_eus(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/LidarMeasurement.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_localization
)
_generate_msg_eus(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/IMUGNSSMeasurement.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_localization
)
_generate_msg_eus(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_localization
)
_generate_msg_eus(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/ESKFStd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_localization
)
_generate_msg_eus(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/EKFStd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_localization
)

### Generating Services
_generate_srv_eus(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/srv/saveOdometry.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_localization
)

### Generating Module File
_generate_module_eus(lidar_localization
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_localization
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(lidar_localization_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(lidar_localization_generate_messages lidar_localization_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVelMag.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_eus _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/LidarMeasurement.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_eus _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/IMUGNSSMeasurement.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_eus _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVel.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_eus _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/srv/saveOdometry.srv" NAME_WE)
add_dependencies(lidar_localization_generate_messages_eus _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/ESKFStd.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_eus _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/EKFStd.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_eus _lidar_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_localization_geneus)
add_dependencies(lidar_localization_geneus lidar_localization_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_localization_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVelMag.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_localization
)
_generate_msg_lisp(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/LidarMeasurement.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_localization
)
_generate_msg_lisp(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/IMUGNSSMeasurement.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_localization
)
_generate_msg_lisp(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_localization
)
_generate_msg_lisp(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/ESKFStd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_localization
)
_generate_msg_lisp(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/EKFStd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_localization
)

### Generating Services
_generate_srv_lisp(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/srv/saveOdometry.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_localization
)

### Generating Module File
_generate_module_lisp(lidar_localization
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_localization
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(lidar_localization_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(lidar_localization_generate_messages lidar_localization_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVelMag.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_lisp _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/LidarMeasurement.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_lisp _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/IMUGNSSMeasurement.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_lisp _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVel.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_lisp _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/srv/saveOdometry.srv" NAME_WE)
add_dependencies(lidar_localization_generate_messages_lisp _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/ESKFStd.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_lisp _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/EKFStd.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_lisp _lidar_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_localization_genlisp)
add_dependencies(lidar_localization_genlisp lidar_localization_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_localization_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVelMag.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_localization
)
_generate_msg_nodejs(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/LidarMeasurement.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_localization
)
_generate_msg_nodejs(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/IMUGNSSMeasurement.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_localization
)
_generate_msg_nodejs(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_localization
)
_generate_msg_nodejs(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/ESKFStd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_localization
)
_generate_msg_nodejs(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/EKFStd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_localization
)

### Generating Services
_generate_srv_nodejs(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/srv/saveOdometry.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_localization
)

### Generating Module File
_generate_module_nodejs(lidar_localization
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_localization
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(lidar_localization_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(lidar_localization_generate_messages lidar_localization_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVelMag.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_nodejs _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/LidarMeasurement.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_nodejs _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/IMUGNSSMeasurement.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_nodejs _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVel.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_nodejs _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/srv/saveOdometry.srv" NAME_WE)
add_dependencies(lidar_localization_generate_messages_nodejs _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/ESKFStd.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_nodejs _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/EKFStd.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_nodejs _lidar_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_localization_gennodejs)
add_dependencies(lidar_localization_gennodejs lidar_localization_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_localization_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVelMag.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_localization
)
_generate_msg_py(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/LidarMeasurement.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_localization
)
_generate_msg_py(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/IMUGNSSMeasurement.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/Imu.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_localization
)
_generate_msg_py(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_localization
)
_generate_msg_py(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/ESKFStd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_localization
)
_generate_msg_py(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/EKFStd.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_localization
)

### Generating Services
_generate_srv_py(lidar_localization
  "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/srv/saveOdometry.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_localization
)

### Generating Module File
_generate_module_py(lidar_localization
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_localization
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(lidar_localization_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(lidar_localization_generate_messages lidar_localization_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVelMag.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_py _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/LidarMeasurement.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_py _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/IMUGNSSMeasurement.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_py _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVel.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_py _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/srv/saveOdometry.srv" NAME_WE)
add_dependencies(lidar_localization_generate_messages_py _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/ESKFStd.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_py _lidar_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/EKFStd.msg" NAME_WE)
add_dependencies(lidar_localization_generate_messages_py _lidar_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lidar_localization_genpy)
add_dependencies(lidar_localization_genpy lidar_localization_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lidar_localization_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lidar_localization
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(lidar_localization_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(lidar_localization_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(lidar_localization_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(lidar_localization_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lidar_localization
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(lidar_localization_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(lidar_localization_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(lidar_localization_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(lidar_localization_generate_messages_eus nav_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lidar_localization
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(lidar_localization_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(lidar_localization_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(lidar_localization_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(lidar_localization_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lidar_localization
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(lidar_localization_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(lidar_localization_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(lidar_localization_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(lidar_localization_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_localization)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_localization\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lidar_localization
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(lidar_localization_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(lidar_localization_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(lidar_localization_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(lidar_localization_generate_messages_py nav_msgs_generate_messages_py)
endif()
