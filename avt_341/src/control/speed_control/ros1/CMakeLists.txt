# Speed control node
add_executable(${PROJECT_NAME}_speed_control_node "../speed_control_node.cpp"
                                                  "../../pid_controller.cpp")
add_dependencies(${PROJECT_NAME}_speed_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(avt_341_speed_control_node ${PROJECT_NAME}_proxy)
install(
  TARGETS ${PROJECT_NAME}_speed_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
