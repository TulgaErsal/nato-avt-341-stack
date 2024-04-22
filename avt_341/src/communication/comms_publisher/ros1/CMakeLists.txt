# Communications publisher node
add_executable(${PROJECT_NAME}_comm_publisher_node "../comm_publisher_node.cpp")
add_dependencies(${PROJECT_NAME}_comm_publisher_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_comm_publisher_node ${PROJECT_NAME}_proxy)
install(
  TARGETS ${PROJECT_NAME}_comm_publisher_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
