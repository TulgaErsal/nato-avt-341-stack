# Control node
add_executable(
  ${PROJECT_NAME}_control_node "../control_node.cpp" "../../pid_controller.cpp"
                               "../../pure_pursuit_controller.cpp")
add_dependencies(${PROJECT_NAME}_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_control_node ${PROJECT_NAME}_proxy)
install(
  TARGETS ${PROJECT_NAME}_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
