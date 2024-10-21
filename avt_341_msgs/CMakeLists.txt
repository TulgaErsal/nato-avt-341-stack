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
    "BoundingBox2d"
    "Communication"
    "Detection2d"
    "Detection2dArray"
    "FollowerStatus"
    "Hypothesis"
    "Obstacles"
    "OccupiedCell"
    "OccupiedCells"
    "Sinkage"
    "StaticObstacleArray"
    "StaticObstacle")

# Append the ".msg" extension for both builds and preprend the "msg/" prefix to
# message definitions for the ament build.
string(REGEX REPLACE "([^;]+)" "\\1.msg" MESSAGE_DEFINITIONS_CATKIN
                     "${MESSAGE_DEFINITIONS}")
string(REGEX REPLACE "([^;]+)" "msg/\\1.msg" MESSAGE_DEFINITIONS_AMENT
                     "${MESSAGE_DEFINITIONS}")

if($ENV{ROS_VERSION} EQUAL 1)
  find_package(catkin REQUIRED COMPONENTS std_msgs nav_msgs message_generation)

  add_message_files(FILES ${MESSAGE_DEFINITIONS_CATKIN})

  generate_messages(DEPENDENCIES std_msgs nav_msgs)

  catkin_package(CATKIN_DEPENDS std_msgs message_runtime)
elseif($ENV{ROS_VERSION} EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME} ${MESSAGE_DEFINITIONS_AMENT}
                             DEPENDENCIES builtin_interfaces std_msgs nav_msgs)

  install(FILES "ros_bridge_mappings.yaml" DESTINATION "share/${PROJECT_NAME}")

  ament_export_dependencies(rosidl_default_runtime)

  ament_package()
else()
  message(
    ERROR
    "Unable to detect ROS version. Did you source /opt/ros/${version}/setup.bash?"
  )
endif()
