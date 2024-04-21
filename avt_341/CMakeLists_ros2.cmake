message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(avt_341_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

if(WIN32 OR WIN64)
    set(link_libs
        ${OpenCV_LIBS}
    )
else()
    find_package(X11 REQUIRED)
    set(link_libs
        ${OpenCV_LIBS}
        X11
    )
endif()

if($ENV{ROS_DISTRO} STREQUAL "humble")
    add_definitions(-DROS_HUMBLE)
endif()

# Build
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
)

IF(WIN32 OR WIN64)
    set(Boost_USE_STATIC_LIBS ON)
    find_package(Boost REQUIRED COMPONENTS system date_time regex)
    include_directories(${Boost_INCLUDE_DIRS})
    target_link_directories(${PROJECT_NAME} PRIVATE $ENV{BOOST_LIBRARYDIR})
endif()

# Node proxy library
# ------------------
add_library(${PROJECT_NAME}_proxy
    "src/node/node_proxy.cpp")

ament_target_dependencies(${PROJECT_NAME}_proxy
    avt_341_msgs
    rclcpp
    tf2_geometry_msgs
    tf2_sensor_msgs
    visualization_msgs)

# Path manager node
add_executable(path_manager_node
    "src/planning/global/path_manager_node.cpp")
target_link_libraries(path_manager_node
    ${PROJECT_NAME}_proxy)
install(TARGETS path_manager_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")

# Mission manager node
add_executable(${PROJECT_NAME}_mission_manager_node
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
target_link_libraries(${PROJECT_NAME}_mission_manager_node
    ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_mission_manager_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION "lib/${PROJECT_NAME}")

# Formation control node
add_executable(${PROJECT_NAME}_formation_control_node
    "src/mission/formation_control_node.cpp"
    "src/mission/formation_controller.cpp"
    "src/mission/formation_utils.cpp")
target_link_libraries(${PROJECT_NAME}_formation_control_node
    ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_formation_control_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION "lib/${PROJECT_NAME}")

# Communications node
add_executable(${PROJECT_NAME}_comm_node
    "src/communication/avt_341_comm_node.cpp"
    "src/communication/tcp_socket_proxy.cpp"
    "src/mission/mission_manager_dto.cpp"
    "src/mission/mission_manager_parser.cpp")
target_link_libraries(${PROJECT_NAME}_comm_node
    ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_comm_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION "lib/${PROJECT_NAME}")

# Communications publisher node
add_executable(${PROJECT_NAME}_comm_publisher_node
    "src/communication/avt_341_comm_publisher_node.cpp")

target_link_libraries(${PROJECT_NAME}_comm_publisher_node
    ${PROJECT_NAME}_proxy)

install(TARGETS ${PROJECT_NAME}_comm_publisher_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")

# Perception node
add_executable(${PROJECT_NAME}_perception_node
    "src/perception/avt_341_perception_node.cpp"
    "src/perception/costmap_clearing_method.cpp"
    "src/perception/elevation_grid.cpp")
target_link_libraries(${PROJECT_NAME}_perception_node
    ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_perception_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION "lib/${PROJECT_NAME}")

# Map publisher node
add_executable(${PROJECT_NAME}_map_publisher_node
    "src/perception/avt_341_map_publisher_node.cpp")
target_link_libraries(${PROJECT_NAME}_map_publisher_node
    ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_map_publisher_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")

# Control node
add_executable(${PROJECT_NAME}_control_node
    "src/control/avt_341_control_node.cpp"
    "src/control/pid_controller.cpp"
    "src/control/pure_pursuit_controller.cpp")
target_link_libraries(${PROJECT_NAME}_control_node
    ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_control_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION "lib/${PROJECT_NAME}")

# Speed control node
add_executable(${PROJECT_NAME}_speed_control_node
    "src/control/avt_341_speed_control_node.cpp"
    "src/control/pid_controller.cpp")
target_link_libraries(${PROJECT_NAME}_speed_control_node
    ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_speed_control_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION "lib/${PROJECT_NAME}")

# Local planner node
add_executable(${PROJECT_NAME}_local_planner_node
    "src/planning/local/avt_341_local_planner_node.cpp"
    "src/planning/local/rviz_spline_plotter.cpp"
    "src/planning/local/spline_path.cpp"
    "src/planning/local/spline_planner.cpp"
    "src/planning/local/spline_plotter.cpp"
    "src/visualization/image_visualizer.cpp")
target_link_libraries(${PROJECT_NAME}_local_planner_node
    ${PROJECT_NAME}_proxy)
target_link_libraries(${PROJECT_NAME}_local_planner_node
    ${link_libs})
install(TARGETS ${PROJECT_NAME}_local_planner_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")

# Potential field planner node
# ----------------------------
add_executable(${PROJECT_NAME}_pf_planner_node
    "src/planning/local/avt_341_pf_planner_node.cpp"
    "src/planning/local/pf_planner.cpp")
target_link_libraries(${PROJECT_NAME}_pf_planner_node
    ${PROJECT_NAME}_proxy
    ${link_libs})
install(TARGETS ${PROJECT_NAME}_pf_planner_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")

# Dynamic window approach planner node
add_executable(${PROJECT_NAME}_dwa_planner_node
    "src/planning/local/avt_341_dwa_planner_node.cpp"
    "src/planning/local/dwa_planner.cpp")
target_link_libraries(${PROJECT_NAME}_dwa_planner_node
    ${link_libs}
    ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_dwa_planner_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")

# Global path planner node
add_executable(${PROJECT_NAME}_global_path_node
    "src/planning/global/astar.cpp"
    "src/planning/global/avt_341_global_path_node.cpp"
    "src/visualization/image_visualizer.cpp")
target_link_libraries(${PROJECT_NAME}_global_path_node
    ${link_libs}
    ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_global_path_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")

# Simulation test node
add_executable(${PROJECT_NAME}_sim_test_node
    "src/node/clock_publisher.cpp"
    "src/perception/point_cloud_generator.cpp"
    "src/simulation/avt_341_sim_test_node.cpp")
target_link_libraries(${PROJECT_NAME}_sim_test_node
    ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_sim_test_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION "lib/${PROJECT_NAME}")

# AVT Bot state publisher node
add_executable(avt_bot_state_publisher_node
    "src/control/avt_bot_state_publisher.cpp")
target_link_libraries(avt_bot_state_publisher_node
    ${PROJECT_NAME}_proxy)
install(TARGETS avt_bot_state_publisher_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")

# Occupancy grid compression node
add_executable(${PROJECT_NAME}_grid_compression_node
    "src/perception/avt_341_grid_compression_node.cpp")
target_link_libraries(${PROJECT_NAME}_grid_compression_node
    ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_grid_compression_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")

# Global segmentation grid node
add_executable(${PROJECT_NAME}_global_segmentation_grid_node
    "src/perception/avt_341_global_segmentation_grid_node.cpp")
target_link_libraries(${PROJECT_NAME}_global_segmentation_grid_node
    ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_global_segmentation_grid_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION "lib/${PROJECT_NAME}")

# UAB perception node
if(WIN32 OR WIN64)
    find_package(Matlab)

    if(Matlab_FOUND)
        # this should point to the installation location of MATLAB Runtime
        set(Matlab_MCLMCRRT_LIB "C:\\Program Files\\MATLAB\\MATLAB Runtime\\R2023a\\extern\\lib\\win64\\microsoft\\mclmcrrt.lib")
        set(Matlab_INCLUDE_DIRS "C:\\Program Files\\MATLAB\\MATLAB Runtime\\R2023a\\extern\\include")
        include_directories(
            include
            ${OpenCV_INCLUDE_DIRS}
            ${Matlab_INCLUDE_DIRS})

        add_executable(uab_perception_node
            "src/perception/uab_perception_node.cpp")
        target_link_libraries(uab_perception_node
            ${PROJECT_NAME}_proxy)
        target_link_libraries(uab_perception_node
            "${CMAKE_SOURCE_DIR}/uab_perception/perception_wrapper.lib"
            ${Matlab_MCLMCRRT_LIB})
        install(FILES "${CMAKE_SOURCE_DIR}/uab_perception/perception_wrapper.dll"
                DESTINATION "lib/${PROJECT_NAME}")
        install(TARGETS uab_perception_node
                EXPORT export_${PROJECT_NAME}
                DESTINATION "lib/${PROJECT_NAME}")
    endif()
endif()

# Testing
# =======

# Target detection test node
add_executable(test_target_detection_node
    "src/perception/test_target_detection_node.cpp")
target_link_libraries(test_target_detection_node
    ${PROJECT_NAME}_proxy)
install(TARGETS test_target_detection_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")

# Speed control test node
add_executable(speed_control_test_node
    "src/control/speed_control_test_node.cpp")

target_link_libraries(speed_control_test_node
    ${PROJECT_NAME}_proxy)

install(TARGETS speed_control_test_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION "lib/${PROJECT_NAME}")

# Formation control test node
add_executable(${PROJECT_NAME}_test_formation_control_node
    "src/mission/test_formation_control_node.cpp")
target_link_libraries(${PROJECT_NAME}_test_formation_control_node
    ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_test_formation_control_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION "lib/${PROJECT_NAME}")

install(DIRECTORY
    launch
    config
    rviz
    DESTINATION "share/${PROJECT_NAME}")

ament_package()
