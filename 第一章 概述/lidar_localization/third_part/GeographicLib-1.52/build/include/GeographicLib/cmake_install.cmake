# Install script for directory: /home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/GeographicLib" TYPE FILE FILES
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/Accumulator.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/AlbersEqualArea.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/AzimuthalEquidistant.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/CassiniSoldner.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/CircularEngine.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/Constants.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/DMS.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/Ellipsoid.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/EllipticFunction.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/GARS.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/GeoCoords.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/Geocentric.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/Geodesic.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/GeodesicExact.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/GeodesicLine.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/GeodesicLineExact.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/Geohash.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/Geoid.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/Georef.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/Gnomonic.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/GravityCircle.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/GravityModel.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/LambertConformalConic.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/LocalCartesian.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/MGRS.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/MagneticCircle.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/MagneticModel.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/Math.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/NearestNeighbor.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/NormalGravity.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/OSGB.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/PolarStereographic.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/PolygonArea.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/Rhumb.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/SphericalEngine.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/SphericalHarmonic.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/SphericalHarmonic1.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/SphericalHarmonic2.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/TransverseMercator.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/TransverseMercatorExact.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/UTMUPS.hpp"
    "/home/x/sensor fusion/third part/GeographicLib-1.52/include/GeographicLib/Utility.hpp"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/GeographicLib" TYPE FILE FILES "/home/x/sensor fusion/third part/GeographicLib-1.52/build/include/GeographicLib/Config.h")
endif()

