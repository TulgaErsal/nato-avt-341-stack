# Speed control test node
add_executable(speed_control_test_node "../speed_control_test_node.cpp")
target_link_libraries(speed_control_test_node ${PROJECT_NAME}_proxy)
install(
  TARGETS speed_control_test_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")
