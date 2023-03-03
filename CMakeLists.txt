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


###########################
## download the UAB dll ##
##########################
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/uab_perception/perception_wrapper.dll")
   # do nothing
else()
file(DOWNLOAD
  https://drive.google.com/uc?export=download&id=1j6TEM9lfAfgaeCVbMfLkTCo0M9c7FW8X&confirm=t&uuid=07647e49-ca01-4076-9147-fc8efa6a3e1a&at=ALgDtsyupXZiPm3DJaloDSGQrRXG:1675708587711
  ${CMAKE_CURRENT_SOURCE_DIR}/uab_perception/perception_wrapper.dll
  SHOW_PROGRESS
)
endif()

if($ENV{ROS_VERSION} EQUAL 1)
  include("CMakeLists_ros1.cmake")
elseif($ENV{ROS_VERSION} EQUAL 2)
  include("CMakeLists_ros2.cmake")
else()
  message(ERROR "Unable to detect ROS version. Did you source /opt/ros/${version}/setup.bash?")
endif()