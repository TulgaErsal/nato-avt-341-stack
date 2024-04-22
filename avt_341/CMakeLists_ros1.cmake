project(avt_341)

cmake_minimum_required(VERSION 3.5)

set(REQUIRED_ROS_PACKAGES
  avt_341_msgs
  dynamic_reconfigure
  jsk_recognition_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs)

if($ENV{ROS_DISTRO} STREQUAL "noetic" OR $ENV{ROS_DISTRO} STREQUAL "melodic")
  find_package(catkin REQUIRED COMPONENTS
    ${REQUIRED_ROS_PACKAGES}
    pcl_ros)
else()
  find_package(catkin REQUIRED COMPONENTS
    ${REQUIRED_ROS_PACKAGES})
endif()

add_definitions(-DROS_1)

find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options("config/lidar_obstacle_detector.cfg")

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nato_avt_341
  CATKIN_DEPENDS avt_341_msgs
#  DEPENDS system_lib
)

# Build
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# Proxy library
add_library(${PROJECT_NAME}_proxy
    "src/node/node_proxy.cpp")
target_link_libraries(${PROJECT_NAME}_proxy
    ${catkin_LIBRARIES})


# Path manager node
add_executable(path_manager_node
  "src/planning/global/path_manager_node.cpp")
add_dependencies(path_manager_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_manager_node
  ${PROJECT_NAME}_proxy)
install(TARGETS path_manager_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# GPS to ENU coordinates conversion node
add_executable(gps_to_enu_node
  "src/planning/global/coord_conversions/coord_conversions.cpp"
  "src/planning/global/coord_conversions/ellipsoid.cpp"
  "src/planning/global/coord_conversions/matrix.cpp"
  "src/planning/global/gps_to_enu_node.cpp")
add_dependencies(gps_to_enu_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_to_enu_node
  ${PROJECT_NAME}_proxy)
install(TARGETS gps_to_enu_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# GPS spoof node
add_executable(gps_spoof_node
  "src/planning/global/gps_spoof_node.cpp")
add_dependencies(gps_spoof_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_spoof_node
  ${catkin_LIBRARIES})
install(TARGETS gps_spoof_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Mission manager node
add_executable(avt_341_mission_manager_node
  "src/mission/formation_definition.cpp"
  "src/mission/formation_path_generator.cpp"
  "src/mission/formation_speed_control.cpp"
  "src/mission/formation_utils.cpp"
  "src/mission/mission_manager.cpp"
  "src/mission/mission_manager_dto.cpp"
  "src/mission/mission_manager_node.cpp"
  "src/mission/mission_manager_parser.cpp"
  "src/mission/task.cpp"
  "src/mission/task_encircle.cpp"
  "src/mission/task_follow.cpp"
  "src/mission/task_moveto.cpp"
  "src/mission/task_wait_until.cpp")
add_dependencies(avt_341_mission_manager_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(avt_341_mission_manager_node
  ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_mission_manager_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Formation control node
add_executable(avt_341_formation_control_node
  "src/mission/formation_control_node.cpp"
  "src/mission/formation_controller.cpp"
  "src/mission/formation_utils.cpp")
add_dependencies(avt_341_formation_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(avt_341_formation_control_node
  ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_formation_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Perception node
add_executable(${PROJECT_NAME}_perception_node
  "src/perception/costmap_clearing_method.cpp"
  "src/perception/elevation_grid.cpp"
  "src/perception/perception_node.cpp")
add_dependencies(${PROJECT_NAME}_perception_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_perception_node
  ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_perception_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Map publisher node
add_executable(${PROJECT_NAME}_map_publisher_node
  "src/perception/map_publisher_node.cpp")
add_dependencies(${PROJECT_NAME}_map_publisher_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_map_publisher_node
  ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_map_publisher_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Control node
add_executable(${PROJECT_NAME}_control_node
  "src/control/control_node.cpp"
  "src/control/pid_controller.cpp"
  "src/control/pure_pursuit_controller.cpp")
add_dependencies(${PROJECT_NAME}_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_control_node
  ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Speed control node
add_executable(${PROJECT_NAME}_speed_control_node
  "src/control/speed_control_node.cpp"
  "src/control/pid_controller.cpp")
add_dependencies(${PROJECT_NAME}_speed_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(avt_341_speed_control_node
  ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_speed_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_executable(speed_control_test_node
  "src/control/speed_control_test_node.cpp")
add_dependencies(speed_control_test_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(speed_control_test_node
  ${PROJECT_NAME}_proxy)
install(TARGETS speed_control_test_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Local planner node
add_executable(${PROJECT_NAME}_local_planner_node
  "src/planning/local/local_planner_node.cpp"
  "src/planning/local/pf_planner.cpp"
  "src/planning/local/spline_path.cpp"
  "src/planning/local/spline_planner.cpp"
  "src/planning/local/spline_plotter.cpp"
  "src/planning/local/rviz_spline_plotter.cpp"
  "src/visualization/image_visualizer.cpp")
add_dependencies(${PROJECT_NAME}_local_planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_local_planner_node
  ${PROJECT_NAME}_proxy
  X11)
install(TARGETS ${PROJECT_NAME}_local_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Potential field planner node
add_executable(${PROJECT_NAME}_pf_planner_node
  "src/planning/local/pf_planner.cpp"
  "src/planning/local/pf_planner_node.cpp"
  "src/visualization/image_visualizer.cpp")
add_dependencies(${PROJECT_NAME}_pf_planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_pf_planner_node
  ${PROJECT_NAME}_proxy
  X11)
install(TARGETS ${PROJECT_NAME}_pf_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# DWA planner node
add_executable(${PROJECT_NAME}_dwa_planner_node
  "src/planning/local/dwa_planner.cpp"
  "src/planning/local/dwa_planner_node.cpp"
  "src/visualization/image_visualizer.cpp")
add_dependencies(${PROJECT_NAME}_dwa_planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_dwa_planner_node
${PROJECT_NAME}_proxy
  X11)
install(TARGETS ${PROJECT_NAME}_dwa_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Global path node
add_executable(${PROJECT_NAME}_global_path_node
  "src/planning/global/astar.cpp"
  "src/planning/global/global_path_node.cpp"
  "src/visualization/image_visualizer.cpp")
add_dependencies(${PROJECT_NAME}_global_path_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_global_path_node
  ${PROJECT_NAME}_proxy
  X11)
install(TARGETS ${PROJECT_NAME}_global_path_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Simulation test node
add_executable(${PROJECT_NAME}_sim_test_node
  "src/node/clock_publisher.cpp"
  "src/perception/point_cloud_generator.cpp"
  "src/simulation/sim_test_node.cpp")
add_dependencies(${PROJECT_NAME}_sim_test_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_sim_test_node
  ${PROJECT_NAME}_proxy
  ${PCL_LIBRARIES})
install(TARGETS ${PROJECT_NAME}_sim_test_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# AVT bot state publisher node
add_executable(avt_bot_state_publisher_node
  "src/control/avt_bot_state_publisher.cpp")
target_link_libraries(avt_bot_state_publisher_node
   ${catkin_LIBRARIES})
install(TARGETS avt_bot_state_publisher_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_executable(${PROJECT_NAME}_grid_compression_node
  "src/perception/grid_compression_node.cpp")
add_dependencies(${PROJECT_NAME}_grid_compression_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_grid_compression_node
  ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_grid_compression_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Communications node
add_executable(${PROJECT_NAME}_comm_node
  "src/communication/comm_node.cpp"
  "src/communication/tcp_socket_proxy.cpp"
  "src/mission/mission_manager_dto.cpp"
  "src/mission/mission_manager_parser.cpp")
add_dependencies(${PROJECT_NAME}_comm_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_comm_node
  ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_comm_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Communications publisher node
add_executable(${PROJECT_NAME}_comm_publisher_node
  "src/communication/comm_publisher_node.cpp")
add_dependencies(${PROJECT_NAME}_comm_publisher_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_comm_publisher_node
  ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_comm_publisher_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

## Lidar_obstacle_detector node
add_executable(${PROJECT_NAME}_lidar_obstacle_detector_node
  "src/perception/lidar_obstacle_detector_node.cpp")
add_dependencies(${PROJECT_NAME}_lidar_obstacle_detector_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_lidar_obstacle_detector_node
  ${catkin_LIBRARIES}
  ${${PROJECT_NAME}_LIBRARY}
  ${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_lidar_obstacle_detector_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

if (WIN32 OR WIN64)
find_package(Matlab)
  if (Matlab_FOUND)
        # this should point to the installation location of MATLAB Runtime
        set(Matlab_MCLMCRRT_LIB "C:\\Program Files\\MATLAB\\MATLAB Runtime\\R2023a\\extern\\lib\\win64\\microsoft\\mclmcrrt.lib")
        include_directories(
                include
                ${OpenCV_INCLUDE_DIRS}
                ${Matlab_INCLUDE_DIRS})
        add_executable(uab_perception_node
          "src/perception/uab_perception_node.cpp")
        add_dependencies(uab_perception_node ${catkin_EXPORTED_TARGETS})
        target_link_libraries(uab_perception_node
                ${PROJECT_NAME}_proxy
                ${CMAKE_SOURCE_DIR}/nato-avt-341-stack/avt_341/uab_perception/perception_wrapper.lib
                ${Matlab_MCLMCRRT_LIB})
        file(COPY
          ${CMAKE_SOURCE_DIR}/nato-avt-341-stack/avt_341/uab_perception/perception_wrapper.dll
          DESTINATION ${CMAKE_SOURCE_DIR}/../devel/lib/${PROJECT_NAME})
        install(TARGETS
                uab_perception_node
                RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
                LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
 endif()
endif()

# Testing
# -------

# Target detection test node
add_executable(test_target_detection_node
  "src/perception/test_target_detection_node.cpp")
add_dependencies(test_target_detection_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_target_detection_node
  ${PROJECT_NAME}_proxy)
install(TARGETS test_target_detection_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Formation control test node
add_executable(${PROJECT_NAME}_test_formation_control_node
  "src/mission/test_formation_control_node.cpp")
add_dependencies(${PROJECT_NAME}_test_formation_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test_formation_control_node
  ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_test_formation_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

set(LIB_SOURCES
  "src/control/pid_controller.cpp"
  "src/control/pure_pursuit_controller.cpp"
  "src/node/clock_publisher.cpp"
  "src/perception/costmap_clearing_method.cpp"
  "src/perception/elevation_grid.cpp"
  "src/perception/point_cloud_generator.cpp"
  "src/planning/local/spline_path.cpp"
  "src/planning/local/spline_planner.cpp"
  "src/planning/local/spline_plotter.cpp"
  "src/visualization/image_visualizer.cpp")

add_library(avt_341 ${LIB_SOURCES})
add_dependencies(avt_341 ${catkin_EXPORTED_TARGETS})
target_link_libraries(avt_341
  ${PROJECT_NAME}_proxy
  ${PCL_LIBRARIES}
  X11)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME})


# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch")

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY rviz/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
        FILES_MATCHING PATTERN "*.rviz")