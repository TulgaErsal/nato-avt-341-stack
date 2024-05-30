message(STATUS "Building perception node ...")

if(USING_CATKIN)
  # Perception node
  add_executable(
    ${PROJECT_NAME}_perception_node "costmap_clearing_method.cpp"
                                    "elevation_grid.cpp" "perception_node.cpp")
  add_dependencies(${PROJECT_NAME}_perception_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_perception_node ${PROJECT_NAME}_proxy)
  install(
    TARGETS ${PROJECT_NAME}_perception_node
    RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
endif() # if(USING_CATKIN)

if(USING_AMENT)
  add_executable(
    ${PROJECT_NAME}_perception_node
    "perception_node.cpp" "costmap_clearing_method.cpp" "elevation_grid.cpp")
  target_link_libraries(${PROJECT_NAME}_perception_node ${PROJECT_NAME}_proxy)
  install(
    TARGETS ${PROJECT_NAME}_perception_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)
