# Control node
add_executable(
  ${PROJECT_NAME}_control_node "../control_node.cpp" "../../pid_controller.cpp"
                               "../../pure_pursuit_controller.cpp")
target_link_libraries(${PROJECT_NAME}_control_node ${PROJECT_NAME}_proxy)
install(
  TARGETS ${PROJECT_NAME}_control_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")
