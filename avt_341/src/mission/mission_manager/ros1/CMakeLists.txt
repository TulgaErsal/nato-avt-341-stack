# Mission manager node
add_executable(
  avt_341_mission_manager_node
  "../../formation_definition.cpp"
  "../../formation_path_generator.cpp"
  "../../formation_speed_control.cpp"
  "../../formation_utils.cpp"
  "../mission_manager.cpp"
  "../mission_manager_dto.cpp"
  "../mission_manager_node.cpp"
  "../mission_manager_parser.cpp"
  "../task.cpp"
  "../task_encircle.cpp"
  "../task_follow.cpp"
  "../task_moveto.cpp"
  "../task_wait_until.cpp"
  "../task_pathfollow.cpp")
add_dependencies(avt_341_mission_manager_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(avt_341_mission_manager_node ${PROJECT_NAME}_proxy)
install(
  TARGETS ${PROJECT_NAME}_mission_manager_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
