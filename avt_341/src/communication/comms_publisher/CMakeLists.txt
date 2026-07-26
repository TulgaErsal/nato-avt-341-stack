message(STATUS "Building communications publisher node ...")

add_executable(${PROJECT_NAME}_comm_publisher_node "comm_publisher_node.cpp")
target_link_libraries(${PROJECT_NAME}_comm_publisher_node
                      ${PROJECT_NAME}_proxy)
install(
  TARGETS ${PROJECT_NAME}_comm_publisher_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")
