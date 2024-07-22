message(STATUS "Building path manager node ...")

if(USING_CATKIN)
  add_executable(path_manager_node "path_manager_node.cpp")
  add_dependencies(path_manager_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(path_manager_node ${PROJECT_NAME}_proxy)
  install(
    TARGETS path_manager_node
    RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
endif() # if(USING_CATKIN)
if(USING_AMENT)
  add_executable(path_manager_node "path_manager_node.cpp")
  target_link_libraries(path_manager_node ${PROJECT_NAME}_proxy)
  install(
    TARGETS path_manager_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)
