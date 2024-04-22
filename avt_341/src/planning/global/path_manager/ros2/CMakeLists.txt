# Path manager node
add_executable(path_manager_node "../path_manager_node.cpp")
target_link_libraries(path_manager_node ${PROJECT_NAME}_proxy)
install(
  TARGETS path_manager_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")
