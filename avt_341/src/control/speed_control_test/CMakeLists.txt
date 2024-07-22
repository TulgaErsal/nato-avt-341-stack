if(BUILD_TESTING)
  message(STATUS "Building speed control test node ...")

  if(USING_CATKIN)
    add_executable(speed_control_test_node "speed_control_test_node.cpp")
    add_dependencies(speed_control_test_node ${catkin_EXPORTED_TARGETS})
    target_link_libraries(speed_control_test_node ${PROJECT_NAME}_proxy)
    install(
      TARGETS speed_control_test_node
      RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
      LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
  endif() # if(USING_CATKIN)

  if(USING_AMENT)
    add_executable(speed_control_test_node "speed_control_test_node.cpp")
    target_link_libraries(speed_control_test_node ${PROJECT_NAME}_proxy)
    install(
      TARGETS speed_control_test_node
      EXPORT export_${PROJECT_NAME}
      DESTINATION "lib/${PROJECT_NAME}")
  endif() # if(USING_AMENT)
endif() # if(BUILD_TESTING)
