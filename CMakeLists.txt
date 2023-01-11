cmake_minimum_required(VERSION 3.7)

if($ENV{ROS_VERSION} EQUAL 1)
  include("CMakeLists_ros1.cmake")
elseif($ENV{ROS_VERSION} EQUAL 2)
  include("CMakeLists_ros2.cmake")
else()
  message(ERROR "Unable to detect ROS version. Did you source /opt/ros/{version}/setup.bash?")
endif()