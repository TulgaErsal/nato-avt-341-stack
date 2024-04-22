# Speed control test node
add_executable(speed_control_test_node "../speed_control_test_node.cpp")
add_dependencies(speed_control_test_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(speed_control_test_node ${PROJECT_NAME}_proxy)
install(
  TARGETS speed_control_test_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
