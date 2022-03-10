cmake_minimum_required(VERSION 3.10.2)
project(lidar_localization)

set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS "-std=c++14 -O3 -Wall")
SET(CMAKE_CXX_FLAGS_RELEASE "-std=c++14 -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  pcl_ros
  tf
  eigen_conversions
  message_generation 
  std_srvs
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    # IMU:
    IMUGNSSMeasurement.msg
    # lidar:
    LidarMeasurement.msg
    # ESKF covariance monitor:
    ESKFStd.msg
    # EKF covariance monitor:
    EKFStd.msg
    # synced GNSS-odo measurement:
    PosVel.msg
    # synced GNSS-odo-mag measurement:
    PosVelMag.msg
)

add_service_files(
   FILES
   saveScanContext.srv
   saveMap.srv
   optimizeMap.srv
   saveOdometry.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
   nav_msgs
)

set(ALL_TARGET_LIBRARIES "")

include(cmake/glog.cmake)
include(cmake/YAML.cmake)
include(cmake/geographic.cmake)
include(cmake/protobuf.cmake)
include(cmake/PCL.cmake)
include(cmake/sophus.cmake)
include(cmake/g2o.cmake)

include_directories(include ${catkin_INCLUDE_DIRS})
include(cmake/global_defination.cmake)
catkin_package()

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})

add_executable(data_pretreat_node src/apps/data_pretreat_node.cpp ${ALL_SRCS})
add_dependencies(data_pretreat_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(data_pretreat_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(front_end_node src/apps/front_end_node.cpp ${ALL_SRCS})
add_dependencies(front_end_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(front_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(loop_closing_node src/apps/loop_closing_node.cpp ${ALL_SRCS})
add_dependencies(loop_closing_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(loop_closing_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(lio_back_end_node src/apps/lio_back_end_node.cpp ${ALL_SRCS})
add_dependencies(lio_back_end_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(lio_back_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(viewer_node src/apps/viewer_node.cpp ${ALL_SRCS})
add_dependencies(viewer_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(viewer_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS 
        data_pretreat_node
        front_end_node
        lio_back_end_node
        loop_closing_node
        viewer_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY 
        include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

## Mark other directories for installation:
install(DIRECTORY
        launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)
install(DIRECTORY
        config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
)
install(DIRECTORY
        rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
    FILES_MATCHING PATTERN "*.rviz"
)
