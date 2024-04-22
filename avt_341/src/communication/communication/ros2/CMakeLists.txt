# Communications node
add_executable(
  ${PROJECT_NAME}_comm_node
  "../comm_node.cpp" "../tcp_socket_proxy.cpp"
  "${PROJECT_SOURCE_DIR}/src/mission/mission_manager/mission_manager_dto.cpp"
  "${PROJECT_SOURCE_DIR}/src/mission/mission_manager/mission_manager_parser.cpp")
target_link_libraries(${PROJECT_NAME}_comm_node ${PROJECT_NAME}_proxy)
install(
  TARGETS ${PROJECT_NAME}_comm_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")
