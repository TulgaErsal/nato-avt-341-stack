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
  https://www.dropbox.com/s/7m0fy3fx3uitxj5/perception_wrapper.dll?dl=1
  ${CMAKE_CURRENT_SOURCE_DIR}/uab_perception/perception_wrapper.dll
  SHOW_PROGRESS
)
endif()

###################################
## download the offline cost map ##
###################################
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config/KRC_CP06_Scaled_Transposed.csv")
   # do nothing
else()
file(DOWNLOAD
  https://drive.google.com/uc?export=download&id=1j_LEJl1xSeoTjgZRY5cXHq2fikBY70GD&confirm=t&uuid=07647e49-ca01-4076-9147-fc8efa6a3e1a&at=ANzk5s6La1ierbYFY2fWj8PXnCNZ:1681522058224
  ${CMAKE_CURRENT_SOURCE_DIR}/config/KRC_CP06_Scaled_Transposed.csv
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
