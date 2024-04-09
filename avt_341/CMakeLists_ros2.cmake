#project(avt_341)

#cmake_minimum_required(VERSION 3.5)

cmake_policy(VERSION 3.16)

set(CMAKE_COMPILE_WARNING_AS_ERROR OFF)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(avt_341_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(GDAL CONFIG REQUIRED)
find_package(PCL REQUIRED)
### MPC ###
if (MPC)
        find_package(Boost REQUIRED COMPONENTS filesystem)
        find_package(casadi REQUIRED)
endif ()
### END MPC ###

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(NOT TARGET GDAL::GDAL)
  add_library(GDAL::GDAL ALIAS ${GDAL_LIBRARY})
endif()
include_directories(${GDAL_INCLUDE_DIRS})

if (WIN32 OR WIN64)
set (link_libs
${OpenCV_LIBS}
)
else()
 find_package(X11 REQUIRED)
set (link_libs
${OpenCV_LIBS}
X11
)
endif()

if($ENV{ROS_DISTRO} STREQUAL "humble")
add_definitions(-DROS_HUMBLE)
endif()

set(dependencies
        rclcpp
        sensor_msgs
        nav_msgs
        geometry_msgs
        visualization_msgs
        avt_341_msgs
        std_msgs
        tf2_ros
        tf2_sensor_msgs
        tf2_geometry_msgs
        pcl_msgs
)

###########
## Build ##
###########
include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(test_target_detection_node
    src/perception/test_target_detection_node.cpp
    src/node/node_proxy.cpp
)
ament_target_dependencies(test_target_detection_node ${dependencies})

add_executable(path_manager_node
    src/planning/global/path_manager_node.cpp
    src/node/node_proxy.cpp
)
ament_target_dependencies(path_manager_node ${dependencies})

add_executable(avt_341_mission_manager_node
    src/mission/mission_manager_node.cpp
    src/mission/mission_manager.cpp
    src/mission/task.cpp
    src/mission/task_encircle.cpp
    src/mission/task_follow.cpp
    src/mission/task_moveto.cpp
    src/mission/task_pathfollow.cpp
    src/mission/task_wait_until.cpp
    src/mission/formation_utils.cpp
    src/mission/formation_definition.cpp
    src/mission/formation_speed_control.cpp
    src/mission/formation_path_generator.cpp
    src/mission/mission_manager_dto.cpp
    src/mission/mission_manager_parser.cpp
    src/node/node_proxy.cpp
)
ament_target_dependencies(avt_341_mission_manager_node ${dependencies})

add_executable(avt_341_formation_control_node
    src/mission/formation_control_node.cpp
    src/mission/formation_controller.cpp
    src/mission/formation_utils.cpp
    src/node/node_proxy.cpp
)
ament_target_dependencies(avt_341_formation_control_node ${dependencies})

add_executable(avt_341_test_formation_control_node
    src/mission/test_formation_control_node.cpp
    src/node/node_proxy.cpp
)
ament_target_dependencies(avt_341_test_formation_control_node ${dependencies})

add_executable(avt_341_comm_node
    src/communication/avt_341_comm_node.cpp
    src/communication/tcp_socket_proxy.cpp
    src/mission/mission_manager_dto.cpp
    src/mission/mission_manager_parser.cpp
    src/node/node_proxy.cpp
)
ament_target_dependencies(avt_341_comm_node ${dependencies})

IF (WIN32 OR WIN64)
    find_package(Boost REQUIRED)
    include_directories(${Boost_INCLUDE_DIRS})
    target_link_directories(avt_341_comm_node PRIVATE $ENV{BOOST_LIBRARYDIR})
endif()

add_executable(avt_341_comm_publisher_node
    src/communication/avt_341_comm_publisher_node.cpp
    src/node/node_proxy.cpp
)
ament_target_dependencies(avt_341_comm_publisher_node ${dependencies})

add_executable(avt_341_perception_node
        src/perception/avt_341_perception_node.cpp
        src/perception/elevation_grid.cpp
        src/node/node_proxy.cpp
        src/perception/costmap_clearing_method.cpp)
ament_target_dependencies(avt_341_perception_node ${dependencies})

add_executable(avt_341_map_publisher_node
        src/perception/avt_341_map_publisher_node.cpp
        src/node/node_proxy.cpp
        )
ament_target_dependencies(avt_341_map_publisher_node
        ${dependencies}
        )

add_executable(avt_341_control_node
        src/control/avt_341_control_node.cpp
        src/control/pure_pursuit_controller.cpp
        src/control/pid_controller.cpp
        src/node/node_proxy.cpp
        )
ament_target_dependencies(avt_341_control_node ${dependencies})

add_executable(avt_341_speed_control_node
        src/control/avt_341_speed_control_node.cpp
        src/control/pid_controller.cpp
        src/node/node_proxy.cpp
        )
ament_target_dependencies(avt_341_speed_control_node ${dependencies})

add_executable(speed_control_test_node
        src/control/speed_control_test_node.cpp
        src/node/node_proxy.cpp
        )
ament_target_dependencies(speed_control_test_node ${dependencies})

add_executable(avt_341_local_planner_node
        src/planning/local/avt_341_local_planner_node.cpp
        src/planning/local/spline_path.cpp
        src/planning/local/spline_planner.cpp
        src/planning/local/spline_plotter.cpp
        src/node/node_proxy.cpp
        src/visualization/image_visualizer.cpp
        src/planning/local/rviz_spline_plotter.cpp
        )
ament_target_dependencies(avt_341_local_planner_node ${dependencies} OpenCV)
target_link_libraries(avt_341_local_planner_node
        ${link_libs}
        )

add_executable(avt_341_pf_planner_node
        src/planning/local/avt_341_pf_planner_node.cpp
        src/planning/local/pf_planner.cpp
        src/node/node_proxy.cpp
        src/visualization/image_visualizer.cpp
      )
ament_target_dependencies(avt_341_pf_planner_node ${dependencies} )
target_link_libraries(avt_341_pf_planner_node
        ${link_libs}
        )

add_executable(avt_341_dwa_planner_node
      src/planning/local/avt_341_dwa_planner_node.cpp
      src/planning/local/dwa_planner.cpp
      src/node/node_proxy.cpp
      src/visualization/image_visualizer.cpp
    )
ament_target_dependencies(avt_341_dwa_planner_node ${dependencies} )
target_link_libraries(avt_341_dwa_planner_node
        ${link_libs}
        )

add_executable(avt_341_global_path_node
        src/planning/global/avt_341_global_path_node.cpp
        src/planning/global/astar.cpp
        src/node/node_proxy.cpp
        src/visualization/image_visualizer.cpp
        src/planning/global/dubins_smoothing.cpp
        )
ament_target_dependencies(avt_341_global_path_node ${dependencies} OpenCV)
target_link_libraries(avt_341_global_path_node
        ${link_libs}
        )

add_executable(avt_341_sim_test_node
        src/simulation/avt_341_sim_test_node.cpp
        src/node/node_proxy.cpp
        src/node/clock_publisher.cpp
        src/perception/point_cloud_generator.cpp
        )
ament_target_dependencies(avt_341_sim_test_node ${dependencies})

add_executable(avt_bot_state_publisher_node
        src/control/avt_bot_state_publisher.cpp
        src/node/node_proxy.cpp
        )
ament_target_dependencies(avt_bot_state_publisher_node ${dependencies})

add_executable(avt_341_grid_compression_node
        src/perception/avt_341_grid_compression_node.cpp
        src/node/node_proxy.cpp
        )
ament_target_dependencies(avt_341_grid_compression_node ${dependencies})

add_executable(avt_341_global_segmentation_grid_node
        src/perception/avt_341_global_segmentation_grid_node.cpp
        src/node/node_proxy.cpp
        )
ament_target_dependencies(avt_341_global_segmentation_grid_node ${dependencies})

add_executable(avt_341_lidar_obstacle_detector_node
        include/avt_341/perception/box.hpp
        include/avt_341/perception/lidar_obstacle_detector.hpp
        src/perception/lidar_obstacle_detector_node.cpp
        src/node/node_proxy.cpp
)
ament_target_dependencies(avt_341_lidar_obstacle_detector_node ${dependencies})
target_link_libraries(avt_341_lidar_obstacle_detector_node
        ${PCL_LIBRARIES}
)

add_executable(data_acquisition_node
        src/daq/data_acquisition_node.cpp
        src/node/node_proxy.cpp
        )
ament_target_dependencies(data_acquisition_node ${dependencies})

# Geotiff Map Publisher
add_executable(avt_341_geotiff_map_publisher_node
  src/perception/avt_341_geotiff_map_publisher_node.cpp
  src/perception/geotiff_dataset.cpp
  src/node/node_proxy.cpp
)
ament_target_dependencies(avt_341_geotiff_map_publisher_node ${dependencies})
target_link_libraries(avt_341_geotiff_map_publisher_node GDAL::GDAL)

add_executable(avt_341_local_occupancy_grid_node
        src/perception/avt_341_local_occupancy_grid_node.cpp
        src/perception/local_occupancy_grid.cpp
        src/node/node_proxy.cpp
        )
ament_target_dependencies(avt_341_local_occupancy_grid_node ${dependencies})
target_link_libraries(avt_341_local_occupancy_grid_node
        ${PCL_LIBRARIES}
)

add_executable(avt_341_costmap_layered_node
        src/perception/avt_341_costmap_layered_node.cpp
        include/avt_341/perception/costmap_layer.h
        src/node/node_proxy.cpp
        )
ament_target_dependencies(avt_341_costmap_layered_node ${dependencies})

add_executable(obstacles_converter_node
        src/perception/obstacles_converter_node.cpp
        src/node/node_proxy.cpp
        )
ament_target_dependencies(obstacles_converter_node ${dependencies})

### MPC BUILDING ###
if (MPC)
        message("Building MPC planner")
        add_executable(veh_converter_node
        src/planning/local/veh_converter_node.cpp
        src/node/node_proxy.cpp
        )
        ament_target_dependencies(veh_converter_node ${dependencies})
        target_link_libraries(veh_converter_node ${link_libs})

        add_executable(avt_341_mpc_planner_node
        src/planning/local/avt_341_mpc_planner_node.cpp
        src/planning/local/mpc_planner_solver.cpp
        src/planning/local/mpc_planner.cpp
        src/node/node_proxy.cpp
        )
        ament_target_dependencies(avt_341_mpc_planner_node ${dependencies} )
        target_link_libraries(avt_341_mpc_planner_node
        ${Boost_LIBRARIES}
        casadi
        ${link_libs}
        )

        add_executable(obstacle_processor_node
        src/planning/local/obstacles_processor_node.cpp
        src/node/node_proxy.cpp
        )
        ament_target_dependencies( obstacle_processor_node ${dependencies} )
        target_link_libraries( obstacle_processor_node ${link_libs} )
endif ()
### END MPC BUILDING ###

if (WIN32 OR WIN64)
# this should point to the installation location of MATLAB Runtime
find_package(Matlab)

 if (Matlab_FOUND)
         set(Matlab_MCLMCRRT_LIB "C:\\Program Files\\MATLAB\\MATLAB Runtime\\v912\\extern\\lib\\win64\\microsoft\\mclmcrrt.lib")
         include_directories(
                 include
                 ${OpenCV_INCLUDE_DIRS}
                 ${Matlab_INCLUDE_DIRS}
         )
         add_executable(uab_perception_node
                 src/perception/uab_perception_node.cpp
                 src/node/node_proxy.cpp
         )
         ament_target_dependencies(uab_perception_node ${dependencies})
         target_link_libraries(uab_perception_node
                 ${CMAKE_SOURCE_DIR}/uab_perception/perception_wrapper.lib
                 ${Matlab_MCLMCRRT_LIB}
         )
         install(FILES
                 ${CMAKE_SOURCE_DIR}/uab_perception/perception_wrapper.dll
                 DESTINATION lib/${PROJECT_NAME})
         install(TARGETS
                 uab_perception_node
                 EXPORT export_${PROJECT_NAME}
                 DESTINATION lib/${PROJECT_NAME})
 endif()

endif()

install(DIRECTORY
        launch
        config
        rviz
        parameters
        maps
        DESTINATION share/${PROJECT_NAME}
        )

install(TARGETS
        avt_341_perception_node
        avt_341_map_publisher_node
        avt_341_control_node
        avt_341_speed_control_node
        avt_341_local_planner_node
        avt_341_pf_planner_node
        avt_341_dwa_planner_node
        avt_341_global_path_node
        avt_341_sim_test_node
        avt_bot_state_publisher_node
        speed_control_test_node
        avt_341_comm_node
        avt_341_comm_publisher_node
        avt_341_mission_manager_node
        avt_341_test_formation_control_node
        avt_341_global_segmentation_grid_node
        test_target_detection_node
        avt_341_formation_control_node
        avt_341_grid_compression_node
        avt_341_global_segmentation_grid_node
        avt_341_lidar_obstacle_detector_node
        data_acquisition_node
        avt_341_geotiff_map_publisher_node
        avt_341_local_occupancy_grid_node
        avt_341_costmap_layered_node
        obstacles_converter_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})

if (MPC)
        install(TARGETS
                avt_341_mpc_planner_node
                veh_converter_node
                obstacle_processor_node
                EXPORT export_${PROJECT_NAME}
                DESTINATION lib/${PROJECT_NAME})
endif ()

install(
        DIRECTORY include/
        DESTINATION include
)
ament_export_include_directories(include)

ament_package()
