cmake_minimum_required(VERSION 3.5)
project(avt_341)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(LIDAR_OBSTACLE_DETECTOR_NODE_SOURCES
  include/lidar_obstacle_detector/box.hpp
  include/lidar_obstacle_detector/obstacle_detector.hpp
  src/lidar_obstacle_detector/obstacle_detector_node.cpp
)

if($ENV{ROS_VERSION} EQUAL 1)
  include("CMakeLists_ros1.cmake")
elseif($ENV{ROS_VERSION} EQUAL 2)
  include("CMakeLists_ros2.cmake")
else()
  message(ERROR "Unable to detect ROS version. Did you source /opt/ros/${version}/setup.bash?")
endif()