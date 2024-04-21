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
  tf2_sensor_msgs
  dynamic_reconfigure
  jsk_recognition_msgs
  avt_341_msgs
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
generate_dynamic_reconfigure_options(
  config/lidar_obstacle_detector.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nato_avt_341
  CATKIN_DEPENDS avt_341_msgs
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


add_executable(avt_341_proj_visibility_node
  src/perception/avt_341_proj_visibility_node.cpp
  src/node/node_proxy.cpp
)
add_dependencies(avt_341_proj_visibility_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(avt_341_proj_visibility_node
  ${catkin_LIBRARIES}
)

add_executable(avt_341_lidar_occlusion_map_node
  src/perception/avt_341_lidar_occlusion_map_node.cpp
  src/perception/voxel_grid.cpp
  src/node/node_proxy.cpp
)
add_dependencies(avt_341_lidar_occlusion_map_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(avt_341_lidar_occlusion_map_node
  ${catkin_LIBRARIES}
)

add_executable(avt_341_lidar_occlusion_node
  src/perception/avt_341_lidar_occlusion_node.cpp
  src/node/node_proxy.cpp
)
add_dependencies(avt_341_lidar_occlusion_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(avt_341_lidar_occlusion_node
  ${catkin_LIBRARIES}
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
add_dependencies(path_manager_node ${catkin_EXPORTED_TARGETS})
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
add_dependencies(gps_to_enu_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_to_enu_node
  ${catkin_LIBRARIES}
)

add_executable(gps_spoof_node
  src/planning/global/gps_spoof_node.cpp
)
add_dependencies(gps_spoof_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_spoof_node
  ${catkin_LIBRARIES}
)


add_executable(avt_341_mission_manager_node
  src/mission/mission_manager_node.cpp
  src/mission/mission_manager.cpp
  src/mission/task.cpp
  src/mission/task_encircle.cpp
  src/mission/task_follow.cpp
  src/mission/task_moveto.cpp
  src/mission/task_wait_until.cpp
  src/mission/formation_utils.cpp
  src/mission/formation_definition.cpp
  src/mission/formation_speed_control.cpp
  src/mission/formation_path_generator.cpp
  src/mission/mission_manager_dto.cpp
  src/mission/mission_manager_parser.cpp
  src/node/node_proxy.cpp
)
add_dependencies(avt_341_mission_manager_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(avt_341_mission_manager_node
  ${catkin_LIBRARIES}
)

add_executable(avt_341_formation_control_node
  src/mission/formation_control_node.cpp
  src/mission/formation_controller.cpp
  src/mission/formation_utils.cpp
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
add_dependencies(avt_341_perception_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(avt_341_perception_node
  ${catkin_LIBRARIES}
)

add_executable(avt_341_map_publisher_node
src/perception/avt_341_map_publisher_node.cpp
src/node/node_proxy.cpp
)
add_dependencies(avt_341_map_publisher_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(avt_341_map_publisher_node
  ${catkin_LIBRARIES}
)

add_executable(avt_341_control_node
  src/control/avt_341_control_node.cpp
  src/control/pure_pursuit_controller.cpp
  src/control/pid_controller.cpp
  src/node/node_proxy.cpp
)
add_dependencies(avt_341_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(avt_341_control_node
  ${catkin_LIBRARIES}
)

add_executable(avt_341_speed_control_node
  src/control/avt_341_speed_control_node.cpp
  src/control/pid_controller.cpp
  src/node/node_proxy.cpp
)
add_dependencies(avt_341_speed_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(avt_341_speed_control_node
  ${catkin_LIBRARIES}
)

add_executable(speed_control_test_node
  src/control/speed_control_test_node.cpp
  src/node/node_proxy.cpp
)
add_dependencies(speed_control_test_node ${catkin_EXPORTED_TARGETS})
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
add_dependencies(avt_341_local_planner_node ${catkin_EXPORTED_TARGETS})
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
add_dependencies(avt_341_pf_planner_node ${catkin_EXPORTED_TARGETS})
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
add_dependencies(avt_341_dwa_planner_node ${catkin_EXPORTED_TARGETS})
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
add_dependencies(avt_341_global_path_node ${catkin_EXPORTED_TARGETS})
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
add_dependencies(avt_341_sim_test_node ${catkin_EXPORTED_TARGETS})
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
add_dependencies(avt_341_grid_compression_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(avt_341_grid_compression_node
        ${catkin_LIBRARIES}
        )


add_executable(avt_341_comm_node
  src/communication/avt_341_comm_node.cpp
  src/communication/tcp_socket_proxy.cpp
  src/mission/mission_manager_dto.cpp
  src/mission/mission_manager_parser.cpp
  src/node/node_proxy.cpp
)
add_dependencies(avt_341_comm_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(avt_341_comm_node
  ${catkin_LIBRARIES}
)

add_executable(avt_341_comm_publisher_node
  src/communication/avt_341_comm_publisher_node.cpp
  src/node/node_proxy.cpp
)
add_dependencies(avt_341_comm_publisher_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(avt_341_comm_publisher_node
  ${catkin_LIBRARIES}
)

## lidar_obstacle_detector node
add_executable(avt_341_lidar_obstacle_detector_node
  ${LIDAR_OBSTACLE_DETECTOR_NODE_SOURCES}
)
add_dependencies(avt_341_lidar_obstacle_detector_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  # obstacle_detector_gencfg
  ${PROJECT_NAME}
)
target_link_libraries(avt_341_lidar_obstacle_detector_node
  ${catkin_LIBRARIES}
  ${${PROJECT_NAME}_LIBRARY}
  ${PROJECT_NAME}
)

if (WIN32 OR WIN64)
find_package(Matlab)

  if (Matlab_FOUND)
        # this should point to the installation location of MATLAB Runtime
        set(Matlab_MCLMCRRT_LIB "C:\\Program Files\\MATLAB\\MATLAB Runtime\\R2023a\\extern\\lib\\win64\\microsoft\\mclmcrrt.lib")
        include_directories(
                include
                ${OpenCV_INCLUDE_DIRS}
                ${Matlab_INCLUDE_DIRS}
        )
        add_executable(uab_perception_node
                src/perception/uab_perception_node.cpp
                src/node/node_proxy.cpp
        )
        add_dependencies(uab_perception_node ${catkin_EXPORTED_TARGETS})
        target_link_libraries(uab_perception_node
                ${catkin_LIBRARIES}
                ${CMAKE_SOURCE_DIR}/nato-avt-341-stack/avt_341/uab_perception/perception_wrapper.lib
                ${Matlab_MCLMCRRT_LIB}
        )
        file(COPY
          ${CMAKE_SOURCE_DIR}/nato-avt-341-stack/avt_341/uab_perception/perception_wrapper.dll
          DESTINATION ${CMAKE_SOURCE_DIR}/../devel/lib/${PROJECT_NAME})
        install(TARGETS
                uab_perception_node
                RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
                LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
 endif()

endif()

set(LIB_SOURCES
src/control/pid_controller.cpp
src/control/pure_pursuit_controller.cpp
src/perception/elevation_grid.cpp
src/perception/point_cloud_generator.cpp
src/perception/costmap_clearing_method.cpp
src/planning/local/spline_path.cpp
src/planning/local/spline_planner.cpp
src/planning/local/spline_plotter.cpp
src/visualization/image_visualizer.cpp
src/node/node_proxy.cpp
src/node/clock_publisher.cpp
)

add_library(avt_341 ${LIB_SOURCES})
add_dependencies(avt_341 ${catkin_EXPORTED_TARGETS})
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
avt_341_lidar_occlusion_node
avt_341_lidar_occlusion_map_node
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

