message(STATUS "Building communications node ...")

add_executable(
  comm_node
  "comm_node.cpp"
  "tcp_socket_proxy.cpp"
  "${PROJECT_SOURCE_DIR}/src/mission/mission_manager/mission_manager_dto.cpp"
  "${PROJECT_SOURCE_DIR}/src/mission/mission_manager/mission_manager_parser.cpp"
)
IF(WIN32 OR WIN64)
  find_package(Boost REQUIRED COMPONENTS system date_time regex)
  include_directories(${Boost_INCLUDE_DIRS})
  target_link_directories(comm_node PRIVATE $ENV{BOOST_LIBRARYDIR})
endif()
target_link_libraries(comm_node ${PROJECT_NAME}_proxy)
install(
  TARGETS comm_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")

# Generated parameter library (avt_341_param_lib templates)
if(TARGET comm_node)
  target_link_libraries(comm_node socket_comms_params_service)
endif()
