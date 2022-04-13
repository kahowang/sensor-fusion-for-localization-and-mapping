# Install script for directory: /media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE PROGRAM FILES "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE PROGRAM FILES "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.bash;/usr/local/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/catkin_generated/installspace/setup.bash"
    "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.sh;/usr/local/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/catkin_generated/installspace/setup.sh"
    "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/setup.zsh;/usr/local/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES
    "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/catkin_generated/installspace/setup.zsh"
    "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local" TYPE FILE FILES "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lidar_localization/msg" TYPE FILE FILES
    "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/IMUGNSSMeasurement.msg"
    "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/LidarMeasurement.msg"
    "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/ESKFStd.msg"
    "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/EKFStd.msg"
    "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVel.msg"
    "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/msg/PosVelMag.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lidar_localization/srv" TYPE FILE FILES "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/srv/saveOdometry.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lidar_localization/cmake" TYPE FILE FILES "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/catkin_generated/installspace/lidar_localization-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/devel/include/lidar_localization")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/devel/share/roseus/ros/lidar_localization")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/devel/share/common-lisp/ros/lidar_localization")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/devel/share/gennodejs/ros/lidar_localization")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/devel/lib/python2.7/dist-packages/lidar_localization")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/devel/lib/python2.7/dist-packages/lidar_localization")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/catkin_generated/installspace/lidar_localization.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lidar_localization/cmake" TYPE FILE FILES "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/catkin_generated/installspace/lidar_localization-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lidar_localization/cmake" TYPE FILE FILES
    "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/catkin_generated/installspace/lidar_localizationConfig.cmake"
    "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/catkin_generated/installspace/lidar_localizationConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lidar_localization" TYPE FILE FILES "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lidar_localization/data_pretreat_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lidar_localization/data_pretreat_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lidar_localization/data_pretreat_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/lidar_localization" TYPE EXECUTABLE FILES "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/devel/lib/lidar_localization/data_pretreat_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lidar_localization/data_pretreat_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lidar_localization/data_pretreat_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lidar_localization/data_pretreat_node"
         OLD_RPATH "/opt/ros/melodic/lib:/usr/lib/x86_64-linux-gnu/hdf5/openmpi:/usr/lib/x86_64-linux-gnu/openmpi/lib:/usr/local/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lidar_localization/data_pretreat_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lidar_localization/lio_matching_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lidar_localization/lio_matching_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lidar_localization/lio_matching_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/lidar_localization" TYPE EXECUTABLE FILES "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/devel/lib/lidar_localization/lio_matching_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lidar_localization/lio_matching_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lidar_localization/lio_matching_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lidar_localization/lio_matching_node"
         OLD_RPATH "/opt/ros/melodic/lib:/usr/lib/x86_64-linux-gnu/hdf5/openmpi:/usr/lib/x86_64-linux-gnu/openmpi/lib:/usr/local/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lidar_localization/lio_matching_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lidar_localization/sliding_window_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lidar_localization/sliding_window_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lidar_localization/sliding_window_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/lidar_localization" TYPE EXECUTABLE FILES "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/devel/lib/lidar_localization/sliding_window_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lidar_localization/sliding_window_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lidar_localization/sliding_window_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lidar_localization/sliding_window_node"
         OLD_RPATH "/opt/ros/melodic/lib:/usr/lib/x86_64-linux-gnu/hdf5/openmpi:/usr/lib/x86_64-linux-gnu/openmpi/lib:/usr/local/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/lidar_localization/sliding_window_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/lidar_localization" TYPE DIRECTORY FILES "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/include/" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/\\.svn$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lidar_localization/launch" TYPE DIRECTORY FILES "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/launch/" FILES_MATCHING REGEX "/[^/]*\\.launch$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lidar_localization/config" TYPE DIRECTORY FILES "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/config/" FILES_MATCHING REGEX "/[^/]*\\.yaml$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lidar_localization/rviz" TYPE DIRECTORY FILES "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/rviz/" FILES_MATCHING REGEX "/[^/]*\\.rviz$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/media/lory/528b0d15-4535-4e6b-842c-5644ad3e33ac/home/kaho/Desktop/多传感器融合定位第四期/第十章 基于优化的定位方法/catkin3_ws/src/lidar_localization/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
