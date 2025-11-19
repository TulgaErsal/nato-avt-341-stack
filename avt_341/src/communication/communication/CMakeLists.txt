message(STATUS "Building communications node ...")

if(USING_CATKIN)
  add_executable(
    ${PROJECT_NAME}_comm_node
    "comm_node.cpp"
    "tcp_socket_proxy.cpp"
    "${PROJECT_SOURCE_DIR}/src/mission/mission_manager/mission_manager_dto.cpp"
    "${PROJECT_SOURCE_DIR}/src/mission/mission_manager/mission_manager_parser.cpp"
  )
  add_dependencies(${PROJECT_NAME}_comm_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_comm_node ${PROJECT_NAME}_proxy)
  install(
    TARGETS ${PROJECT_NAME}_comm_node
    RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
endif() # if(USING_CATKIN)

if(USING_AMENT)
  add_executable(
    ${PROJECT_NAME}_comm_node
    "comm_node.cpp"
    "tcp_socket_proxy.cpp"
    "${PROJECT_SOURCE_DIR}/src/mission/mission_manager/mission_manager_dto.cpp"
    "${PROJECT_SOURCE_DIR}/src/mission/mission_manager/mission_manager_parser.cpp"
  )
  IF(WIN32 OR WIN64)
    find_package(Boost REQUIRED COMPONENTS system date_time regex)
    include_directories(${Boost_INCLUDE_DIRS})
    target_link_directories(${PROJECT_NAME}_comm_node PRIVATE $ENV{BOOST_LIBRARYDIR})
  endif()
  target_link_libraries(${PROJECT_NAME}_comm_node ${PROJECT_NAME}_proxy)
  install(
    TARGETS ${PROJECT_NAME}_comm_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)
