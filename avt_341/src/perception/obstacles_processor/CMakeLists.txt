if(USING_CATKIN)
    message(WARNING "Skipping obstacles processor node, not supported in ROS 1 ...")
endif() # if(USING_CATKIN)

if(USING_AMENT)
    message(WARNING "Building obstacles processor node ...")
    add_executable(obstacle_processor_node
        "obstacles_processor_node.cpp"
    )
    target_link_libraries(obstacle_processor_node ${dependencies}
        ${PROJECT_NAME}_proxy)
    install(
        TARGETS obstacle_processor_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)