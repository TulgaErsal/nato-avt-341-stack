cmake_minimum_required(VERSION 3.5)

project(avt_341)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if(WIN32 OR WIN64)
  add_compile_definitions(-D _USE_MATH_DEFINES)
endif() # if(WIN32 OR WIN64)

if(SET_BIG_OBJ)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
  message(STATUS "/bigobj added to CMAKE_CXX_FLAGS")
endif(SET_BIG_OBJ)

set(AVT341_SOURCE_FILES
    "communication/comms_publisher"
    "communication/communication"
    "control/avt_bot_state_publisher"
    "control/control"
    "control/speed_control"
    "control/speed_zones"
    "daq/data_acquisition"
    "mission/goal_filtering"
    "mission/mission_manager"
    "perception/detection/common"
    "perception/detection/training"
    "perception/detection/object_detector"
    "perception/tracking"
    "perception/global_segmentation_grid"
    "perception/lidar_obstacle_detector"
    "perception/perception"
    "perception/uab_terrain_perception"
    "perception/obstacles_processor"
    "perception/lidar_normal_estimation"
    "perception/normal_segmentation_grid"
    "perception/segmentation_grid_processor"
    "perception/geotiff_map"
    "planning/local/local_planner"
    "planning/local/potential_field"
    "planning/local/dwa"
    "planning/local/goal_point_processor"
    "planning/local/vehicle_converter"
    "planning/local/mpc_planner"
    "planning/global/global_path"
    "planning/global/pf_global_path")

# Append src/ subdirectory to the path of each target.
string(REGEX REPLACE "([^;]+)" "src/\\1" AVT341_SOURCE_FILES
                     "${AVT341_SOURCE_FILES}")

find_package(ackermann_msgs)
find_package(ament_cmake REQUIRED)
find_package(avt_341_param_lib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(avt_341_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 3.4 REQUIRED)

set(link_libs ${OpenCV_LIBS})

# Since ROS Humble several header file endpoints changed (tf2_sensor_msgs, tf2_geometry_msgs, tf2_eigen, etc.)
if($ENV{ROS_DISTRO} STREQUAL "humble" OR $ENV{ROS_DISTRO} STREQUAL "jazzy")
  add_definitions(-DGTE_ROS_HUMBLE)
endif()

if($ENV{ROS_DISTRO} STREQUAL "jazzy")
  add_definitions(-DGTE_ROS_JAZZY)
endif()

# Build
include_directories(${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})

# Generated parameter DTO and service libraries:
# <stem>_params_dto and <stem>_params_service.
avt_341_generate_cpp_parameters(parameters/templates)

# Node proxy library
# ------------------
add_library(${PROJECT_NAME}_proxy
        "src/node/tf_interface.cpp"
        "src/node/occupancy_grid_subscriber.cpp"
        "src/core/grid_components.cpp"
        "src/core/running_stats.cpp"
        "src/core/compute_time_recorder.cpp"
        "src/core/coord_transform.cpp"
        "src/core/waypoint_file_parser.cpp"
)

target_include_directories(
  ${PROJECT_NAME}_proxy
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  ${PROJECT_NAME}_proxy ackermann_msgs avt_341_msgs rclcpp tf2_geometry_msgs tf2_sensor_msgs
  tf2_ros tf2_eigen Eigen3 visualization_msgs map_msgs)

foreach(SOURCE_FILE ${AVT341_SOURCE_FILES})
  add_subdirectory(${SOURCE_FILE})
endforeach()

# Install the Python package
ament_python_install_package(${PROJECT_NAME})

# Simulation test node
add_executable(
  ${PROJECT_NAME}_sim_test_node
  "src/perception/point_cloud_generator.cpp"
  "src/simulation/sim_test_node.cpp")
target_link_libraries(${PROJECT_NAME}_sim_test_node ${PROJECT_NAME}_proxy)
install(
  TARGETS ${PROJECT_NAME}_sim_test_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")

# Test Nodes
# ===============

# Target detection test node
add_executable(test_target_detection_node
               "src/perception/test_target_detection_node.cpp")
target_link_libraries(test_target_detection_node ${PROJECT_NAME}_proxy)
install(
  TARGETS test_target_detection_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")

install(
  PROGRAMS "launch/vehicle_logging.py" "launch/ros_param_dump.py" "launch/tf_to_odometry.py"
            "test/global_path_test_driver.py"
            "test/mpc_terminal_heading_test_driver.py"
            "test/formation_distance_test_driver.py"
            "test/formation_distance_two_follower_test_driver.py"
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config rviz parameters DESTINATION "share/${PROJECT_NAME}")

# Automated Tests
# ===============
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_package()
