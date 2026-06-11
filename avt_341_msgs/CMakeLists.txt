cmake_minimum_required(VERSION 3.5)

project(avt_341_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Define the messages to be built.
set(MESSAGE_DEFINITIONS
    "Communication"
    "DwaInfo"
    "DwaObjective"
    "DwaTrajectory"
    "FollowerStatus"
    "LiorfCloudInfo"
    "MissionTaskStatus"
    "Obstacles"
    "OccupiedCell"
    "OccupiedCells"
    "Sinkage"
    "TrackerInfo"
    "NavGoal"
    "NavGoalSequence"
    "NavState"
)

set(SERVICE_DEFINITIONS
    "LiorfSaveMap"
    "SetTarget"
    "ComputeGlobalPath"
    "SetNavPointDefinitions"
)

# Append the ".msg" extension for both builds and preprend the "msg/" prefix to
# message definitions for the ament build.
string(REGEX REPLACE "([^;]+)" "\\1.msg" MESSAGE_DEFINITIONS_CATKIN
                     "${MESSAGE_DEFINITIONS}")
string(REGEX REPLACE "([^;]+)" "msg/\\1.msg" MESSAGE_DEFINITIONS_AMENT
                     "${MESSAGE_DEFINITIONS}")
string(REGEX REPLACE "([^;]+)" "\\1.srv" SERVICE_DEFINITIONS_CATKIN
                     "${SERVICE_DEFINITIONS}")
string(REGEX REPLACE "([^;]+)" "srv/\\1.srv" SERVICE_DEFINITIONS_AMENT
                      "${SERVICE_DEFINITIONS}")

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${MESSAGE_DEFINITIONS_AMENT}
  ${SERVICE_DEFINITIONS_AMENT}
  DEPENDENCIES
  builtin_interfaces
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
)

install(FILES "ros_bridge_mappings.yaml" DESTINATION "share/${PROJECT_NAME}")

ament_export_dependencies(rosidl_default_runtime)

ament_package()

