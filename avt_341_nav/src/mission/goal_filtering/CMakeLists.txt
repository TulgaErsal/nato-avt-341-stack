find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(obs_avoid_goal_filter_node
        obs_avoid_goal_filter_node.cpp
        obs_avoid_goal_filter.cpp
        obs_avoid_goal_filter_utils.cpp
        goal_filter_factory.cpp
)

target_link_libraries(obs_avoid_goal_filter_node
        ${PROJECT_NAME}_proxy)

install(TARGETS obs_avoid_goal_filter_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION "lib/${PROJECT_NAME}")

# Generated parameter library (avt_341_param_lib templates)
if(TARGET obs_avoid_goal_filter_node)
  target_link_libraries(
    obs_avoid_goal_filter_node mission_manager_params_dto)
endif()
