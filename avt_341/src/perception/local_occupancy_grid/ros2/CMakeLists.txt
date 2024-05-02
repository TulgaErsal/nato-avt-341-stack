# Local occupancy grid node
add_executable(${PROJECT_NAME}_local_occupancy_grid_node
        "../avt_341_local_occupancy_grid_node.cpp"
        "../local_occupancy_grid.cpp"
)
target_link_libraries(${PROJECT_NAME}_local_occupancy_grid_node
                      ${PROJECT_NAME}_proxy
                      ${PCL_LIBRARIES})
install(
    TARGETS ${PROJECT_NAME}_local_occupancy_grid_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
