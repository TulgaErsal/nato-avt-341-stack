
if(USING_CATKIN)
    message(WARNING "Obstacle avoidance goal filter node not supported in ROS1")
endif() # if(USING_CATKIN)

if(USING_AMENT)

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(Eigen3 REQUIRED)

    add_executable(${PROJECT_NAME}_obs_avoidance_goal_filter_node
            obs_avoidance_goal_filter_node.cpp
            obs_avoidance_goal_filter.cpp
            obs_avoidance_goal_filter_utils.cpp
            formation_goal_filter.cpp
    )

    target_link_libraries(${PROJECT_NAME}_obs_avoidance_goal_filter_node
            ${PROJECT_NAME}_proxy)

    install(TARGETS ${PROJECT_NAME}_obs_avoidance_goal_filter_node
            EXPORT export_${PROJECT_NAME}
            DESTINATION "lib/${PROJECT_NAME}")

endif ()
