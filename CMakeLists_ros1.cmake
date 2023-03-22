project(avt_341)
 
cmake_minimum_required(VERSION 3.5)

set(REQUIRED_ROS_PACKAGES
  roscpp
  rospy
  std_msgs
  nav_msgs
  tf
  message_generation
  tf2_ros
  tf2_geometry_msgs
  dynamic_reconfigure
  jsk_recognition_msgs    
)

if($ENV{ROS_DISTRO} STREQUAL "noetic" OR $ENV{ROS_DISTRO} STREQUAL "melodic")
  find_package(catkin REQUIRED COMPONENTS
    ${REQUIRED_ROS_PACKAGES}
    pcl_ros
  )
else()
  find_package(catkin REQUIRED COMPONENTS
    ${REQUIRED_ROS_PACKAGES}
  )
endif()

add_definitions(-DROS_1)

find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

## Generate dynamic reconfigure parameters in the 'cfg' folder
#generate_dynamic_reconfigure_options(
#  config/lidar_obstacle_detector.cfg
#)

#########################
## add custom messages ##
#########################

add_message_files(
 FILES
 Sinkage.msg
 Obstacles.msg
 Communication.msg
 FollowerStatus.msg
 OccupiedCell.msg
 OccupiedCells.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nato_avt_341
#  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_executable(test_target_detection_node
  src/perception/test_target_detection_node.cpp
  src/node/node_proxy.cpp
)
add_dependencies(test_target_detection_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_target_detection_node
  ${catkin_LIBRARIES}
)

add_executable(path_manager_node
  src/planning/global/path_manager_node.cpp
  src/node/node_proxy.cpp
)
target_link_libraries(path_manager_node
  ${catkin_LIBRARIES}
)

add_executable(gps_to_enu_node
  src/planning/global/gps_to_enu_node.cpp
  src/node/node_proxy.cpp
  src/planning/global/coord_conversions/coord_conversions.cpp
  src/planning/global/coord_conversions/ellipsoid.cpp
  src/planning/global/coord_conversions/matrix.cpp
)
target_link_libraries(gps_to_enu_node
  ${catkin_LIBRARIES}
)

add_executable(gps_spoof_node
  src/planning/global/gps_spoof_node.cpp
)
target_link_libraries(gps_spoof_node
  ${catkin_LIBRARIES}
)


add_executable(avt_341_mission_manager_node
  src/mission/mission_manager_node.cpp
  src/mission/mission_manager.cpp
  src/node/node_proxy.cpp
)
add_dependencies(avt_341_mission_manager_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(avt_341_mission_manager_node
  ${catkin_LIBRARIES}
)

add_executable(avt_341_formation_control_node
  src/mission/formation_control_node.cpp
  src/mission/formation_controller.cpp
  src/node/node_proxy.cpp
)
add_dependencies(avt_341_formation_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(avt_341_formation_control_node
  ${catkin_LIBRARIES}
)

add_executable(avt_341_test_formation_control_node
  src/mission/test_formation_control_node.cpp
  src/node/node_proxy.cpp
)
add_dependencies(avt_341_test_formation_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(avt_341_test_formation_control_node
  ${catkin_LIBRARIES}
)

add_executable(avt_341_perception_node
src/perception/avt_341_perception_node.cpp
src/perception/elevation_grid.cpp
src/node/node_proxy.cpp
src/perception/costmap_clearing_method.cpp
)
target_link_libraries(avt_341_perception_node
  ${catkin_LIBRARIES}
)

add_executable(avt_341_map_publisher_node
src/perception/avt_341_map_publisher_node.cpp
src/node/node_proxy.cpp
)
target_link_libraries(avt_341_map_publisher_node
  ${catkin_LIBRARIES}
)

add_executable(avt_341_control_node
  src/control/avt_341_control_node.cpp
  src/control/pure_pursuit_controller.cpp
  src/control/pid_controller.cpp
  src/node/node_proxy.cpp
)
target_link_libraries(avt_341_control_node
  ${catkin_LIBRARIES}
)

add_executable(avt_341_speed_control_node
  src/control/avt_341_speed_control_node.cpp
  src/control/pid_controller.cpp
  src/node/node_proxy.cpp
)
target_link_libraries(avt_341_speed_control_node
  ${catkin_LIBRARIES}
)

add_executable(speed_control_test_node
  src/control/speed_control_test_node.cpp
  src/node/node_proxy.cpp
)
target_link_libraries(speed_control_test_node
  ${catkin_LIBRARIES}
)

add_executable(avt_341_local_planner_node
  src/planning/local/avt_341_local_planner_node.cpp
  src/planning/local/spline_path.cpp
  src/planning/local/spline_planner.cpp
  src/planning/local/spline_plotter.cpp
  src/planning/local/pf_planner.cpp
  src/node/node_proxy.cpp
  src/visualization/image_visualizer.cpp
  src/planning/local/rviz_spline_plotter.cpp
)
target_link_libraries(avt_341_local_planner_node
  ${catkin_LIBRARIES}
  X11
)

add_executable(avt_341_pf_planner_node
  src/planning/local/avt_341_pf_planner_node.cpp
  src/planning/local/pf_planner.cpp
  src/node/node_proxy.cpp
  src/visualization/image_visualizer.cpp
)
target_link_libraries(avt_341_pf_planner_node
  ${catkin_LIBRARIES}
  X11
)

add_executable(avt_341_dwa_planner_node
  src/planning/local/avt_341_dwa_planner_node.cpp
  src/planning/local/dwa_planner.cpp
  src/node/node_proxy.cpp
  src/visualization/image_visualizer.cpp
)
target_link_libraries(avt_341_dwa_planner_node
  ${catkin_LIBRARIES}
  X11
)

add_executable(avt_341_global_path_node
  src/planning/global/avt_341_global_path_node.cpp
  src/planning/global/astar.cpp
  src/node/node_proxy.cpp
  src/visualization/image_visualizer.cpp
)
target_link_libraries(avt_341_global_path_node
  ${catkin_LIBRARIES}
  X11
)

add_executable(avt_341_sim_test_node
  src/simulation/avt_341_sim_test_node.cpp
  src/node/node_proxy.cpp
  src/node/clock_publisher.cpp
  src/perception/point_cloud_generator.cpp
)
target_link_libraries(avt_341_sim_test_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(avt_bot_state_publisher_node
  src/control/avt_bot_state_publisher.cpp
)
target_link_libraries(avt_bot_state_publisher_node
   ${catkin_LIBRARIES}
)


add_executable(avt_341_grid_compression_node
        src/perception/avt_341_grid_compression_node.cpp
        src/node/node_proxy.cpp
        )
add_dependencies(avt_341_grid_compression_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        )
target_link_libraries(avt_341_grid_compression_node
        ${catkin_LIBRARIES}
        )


add_executable(avt_341_comm_node
  src/communication/avt_341_comm_node.cpp
  src/node/node_proxy.cpp
)
target_link_libraries(avt_341_comm_node
  ${catkin_LIBRARIES}
)

add_executable(avt_341_comm_publisher_node
  src/communication/avt_341_comm_publisher_node.cpp
  src/node/node_proxy.cpp
)
target_link_libraries(avt_341_comm_publisher_node
  ${catkin_LIBRARIES}
)

## lidar_obstacle_detector node
add_executable(avt_341_lidar_obstacle_detector_node
  ${LIDAR_OBSTACLE_DETECTOR_NODE_SOURCES}
)
#add_dependencies(avt_341_lidar_obstacle_detector_node 
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#  ${catkin_EXPORTED_TARGETS}
#  # obstacle_detector_gencfg
#  ${PROJECT_NAME}
#)
#target_link_libraries(avt_341_lidar_obstacle_detector_node
#  ${catkin_LIBRARIES}
#  ${${PROJECT_NAME}_LIBRARY}
#  ${PROJECT_NAME}
#)

set(LIB_SOURCES
src/control/pid_controller.cpp
src/control/pure_pursuit_controller.cpp
src/perception/elevation_grid.cpp
src/perception/point_cloud_generator.cpp
src/planning/local/spline_path.cpp
src/planning/local/spline_planner.cpp
src/planning/local/spline_plotter.cpp
src/visualization/image_visualizer.cpp
src/node/node_proxy.cpp
src/node/clock_publisher.cpp
)

add_library(avt_341 ${LIB_SOURCES})
target_link_libraries(avt_341
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  X11
)

catkin_package(INCLUDE_DIRS include
               LIBRARIES avt_341)

#############
## Install ##
#############

install(TARGETS
test_target_detection_node
avt_341_perception_node
avt_341_map_publisher_node
avt_341_control_node
avt_341_speed_control_node
avt_341_local_planner_node
avt_341_pf_planner_node
avt_341_dwa_planner_node
avt_341_global_path_node
avt_341_grid_compression_node
avt_341_lidar_obstacle_detector_node
avt_341_sim_test_node
gps_to_enu_node
gps_spoof_node
path_manager_node
speed_control_test_node
avt_bot_state_publisher_node
avt_341_comm_node
avt_341_comm_publisher_node
avt_341_mission_manager_node
avt_341_formation_control_node
avt_341_test_formation_control_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS avt_341
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
 )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch"
        )

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
        )

install(DIRECTORY rviz/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
        FILES_MATCHING PATTERN "*.rviz"
        )
