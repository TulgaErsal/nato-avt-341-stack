message(STATUS "Building missiong manager node ...")

add_executable(
  ${PROJECT_NAME}_mission_manager_node

  "mission_manager.cpp"
  "mission_manager_dto.cpp"
  "mission_manager_node.cpp"
  "mission_manager_parser.cpp"

  "task.cpp"
  "task_encircle.cpp"
  "task_follow.cpp"
  "task_moveto.cpp"
  "task_wait_until.cpp"
  "task_pathfollow.cpp"

  "../formation_definition.cpp"
  "../formation_path_generator.cpp"
  "../formation_speed_control.cpp"
  "../formation_utils.cpp"

  "../goal_filtering/goal_filter_factory.cpp"
  "../goal_filtering/obs_avoid_goal_filter.cpp"
  "../goal_filtering/obs_avoid_goal_filter_utils.cpp"
)

target_link_libraries(${PROJECT_NAME}_mission_manager_node
                      ${PROJECT_NAME}_proxy)
install(
  TARGETS ${PROJECT_NAME}_mission_manager_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")

# Generated parameter library (avt_341_param_lib templates)
if(TARGET ${PROJECT_NAME}_mission_manager_node)
  target_link_libraries(
    ${PROJECT_NAME}_mission_manager_node mission_manager_params_service)
endif()
