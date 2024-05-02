# Lidar obstacle detector node
add_executable(${PROJECT_NAME}_lidar_obstacle_detector_node
        "../lidar_obstacle_detector_node.cpp"
)
target_link_libraries(${PROJECT_NAME}_lidar_obstacle_detector_node
                      ${PROJECT_NAME}_proxy
                      ${PCL_LIBRARIES})
install(
    TARGETS ${PROJECT_NAME}_lidar_obstacle_detector_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
