# Formation control node
add_executable(
  ${PROJECT_NAME}_formation_control_node
  "../formation_control_node.cpp" "../formation_controller.cpp"
  "../../formation_utils.cpp")
target_link_libraries(${PROJECT_NAME}_formation_control_node
                      ${PROJECT_NAME}_proxy)
install(
  TARGETS ${PROJECT_NAME}_formation_control_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")
