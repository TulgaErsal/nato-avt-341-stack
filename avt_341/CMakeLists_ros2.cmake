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
find_package(pcl_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
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

add_subdirectory(src/communication/comms_publisher/ros2)
add_subdirectory(src/communication/communication/ros2)
add_subdirectory(src/control/avt_bot_state_publisher/ros2)
add_subdirectory(src/control/control/ros2)
add_subdirectory(src/control/speed_control/ros2)
add_subdirectory(src/control/speed_control_test/ros2)
add_subdirectory(src/mission/formation_control/ros2)
add_subdirectory(src/mission/mission_manager/ros2)
add_subdirectory(src/perception/map_publisher/ros2)
add_subdirectory(src/perception/global_segmentation_grid/ros2)
add_subdirectory(src/perception/grid_compression/ros2)
add_subdirectory(src/perception/perception/ros2)
add_subdirectory(src/perception/lidar_obstacle_detector/ros2)
add_subdirectory(src/perception/local_occupancy_grid/ros2)
add_subdirectory(src/planning/local/local_planner/ros2)
add_subdirectory(src/planning/local/potential_field/ros2)
add_subdirectory(src/planning/local/dwa/ros2)
add_subdirectory(src/planning/global/global_path/ros2)
add_subdirectory(src/daq/data_acquisition/ros2)

# Simulation test node
add_executable(${PROJECT_NAME}_sim_test_node
    "src/node/clock_publisher.cpp"
    "src/perception/point_cloud_generator.cpp"
    "src/simulation/sim_test_node.cpp")
target_link_libraries(${PROJECT_NAME}_sim_test_node
    ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_sim_test_node
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
        parameters
        maps
        DESTINATION share/${PROJECT_NAME}
        )

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
