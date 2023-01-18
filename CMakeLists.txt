cmake_minimum_required(VERSION 3.5)
project(avt_341)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(LIDAR_OBSTACLE_DETECTOR_NODE_SOURCES
  include/avt_341/perception/box.hpp
  include/avt_341/perception/lidar_obstacle_detector.hpp
  src/perception/lidar_obstacle_detector_node.cpp
)

set(OBJECT_FUSION_NODE_SOURCES
  src/perception/object_fusion_node.cpp
)

if($ENV{ROS_VERSION} EQUAL 1)
  include("CMakeLists_ros1.cmake")
elseif($ENV{ROS_VERSION} EQUAL 2)
  include("CMakeLists_ros2.cmake")
else()
  message(ERROR "Unable to detect ROS version. Did you source /opt/ros/${version}/setup.bash?")
endif()