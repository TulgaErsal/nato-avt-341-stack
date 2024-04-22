cmake_minimum_required(VERSION 3.5)

project(avt_341)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Download the UAB perception library
file(DOWNLOAD
       https://www.dropbox.com/s/7m0fy3fx3uitxj5/perception_wrapper.dll?dl=1
       "${CMAKE_CURRENT_SOURCE_DIR}/uab_perception/perception_wrapper.dll"
     EXPECTED_HASH SHA256=8b1563225bd5e547f375903eb0d5a5a696566091bb65d76476aff4fab452d14f
     SHOW_PROGRESS)

# Download the offline cost map
file(DOWNLOAD
       https://www.dropbox.com/s/n6img54q02j8tbs/20230816_KRC_CP06_Scaled_FlippedVertical.csv?dl=1
       "${CMAKE_CURRENT_SOURCE_DIR}/config/20230816_KRC_CP06_Scaled_FlippedVertical.csv"
     EXPECTED_HASH SHA256=ee537193d028f54ffe08ce9ba78879ee2758fa129942a66f81001382c90fb588
     SHOW_PROGRESS)

if($ENV{ROS_VERSION} EQUAL 1)
  include("CMakeLists_ros1.cmake")
elseif($ENV{ROS_VERSION} EQUAL 2)
  include("CMakeLists_ros2.cmake")
else()
  message(ERROR "Unable to detect ROS version. Did you source /opt/ros/${version}/setup.bash?")
endif()
