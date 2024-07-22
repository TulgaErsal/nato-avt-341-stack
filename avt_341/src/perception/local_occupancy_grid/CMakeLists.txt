if(USING_CATKIN)
    message(WARNING "Skipping local occupancy grid node, not supported in ROS 1 ...")
endif() # if(USING_CATKIN)

if(USING_AMENT)
    message(STATUS "Building local occupancy grid node ...")
    # Local occupancy grid node
    add_executable(${PROJECT_NAME}_local_occupancy_grid_node
        "local_occupancy_grid_node.cpp"
        "local_occupancy_grid.cpp"
    )
    target_link_libraries(${PROJECT_NAME}_local_occupancy_grid_node
        ${PROJECT_NAME}_proxy
        ${PCL_LIBRARIES})
    install(
        TARGETS ${PROJECT_NAME}_local_occupancy_grid_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)