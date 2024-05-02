# Communications node
add_executable(
  ${PROJECT_NAME}_comm_node
  "../comm_node.cpp" "../tcp_socket_proxy.cpp"
  "${PROJECT_SOURCE_DIR}/src/mission/mission_manager/mission_manager_dto.cpp"
  "${PROJECT_SOURCE_DIR}/src/mission/mission_manager/mission_manager_parser.cpp")
add_dependencies(${PROJECT_NAME}_comm_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_comm_node ${PROJECT_NAME}_proxy)
install(
  TARGETS ${PROJECT_NAME}_comm_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
