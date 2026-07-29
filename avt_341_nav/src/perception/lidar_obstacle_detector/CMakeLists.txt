message(STATUS "Building LiDAR obstacle detector node ...")

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

add_definitions(${PCL_DEFINITIONS})

# Lidar obstacle detector node
add_executable(lidar_obstacle_detector_node
        "lidar_obstacle_detector_node.cpp"
)
target_link_libraries(lidar_obstacle_detector_node
                      ${PROJECT_NAME}_proxy ${${PROJECT_NAME}_LIBRARY} ${PCL_LIBRARIES})

target_include_directories(
  lidar_obstacle_detector_node
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(lidar_obstacle_detector_node
                          pcl_conversions
                          visualization_msgs)

install(
    TARGETS lidar_obstacle_detector_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")

# Generated parameter library (avt_341_param_lib templates)
if(TARGET lidar_obstacle_detector_node)
  target_link_libraries(
    lidar_obstacle_detector_node
    obstacle_detector_params_service)
endif()
