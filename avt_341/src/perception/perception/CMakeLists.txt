message(STATUS "Building perception node ...")

add_executable(
        ${PROJECT_NAME}_perception_node
        "perception_node.cpp"
        "clearing_methods/costmap_clearing_method.cpp"
        "clearing_methods/clearing_methods_factory.cpp"
        "clearing_methods/channel_clearing_method.cpp"
        "clearing_methods/raytrace_clearing_method.cpp"
        "clearing_methods/time_clearing_method.cpp"
        "elevation_grid.cpp"
        "point_cloud_filter.cpp"
)

if(USING_CATKIN)
  add_dependencies(${PROJECT_NAME}_perception_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_perception_node ${PROJECT_NAME}_proxy)
  install(
    TARGETS ${PROJECT_NAME}_perception_node
    RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
endif() # if(USING_CATKIN)

if(USING_AMENT)
  target_link_libraries(${PROJECT_NAME}_perception_node ${PROJECT_NAME}_proxy)
  install(
    TARGETS ${PROJECT_NAME}_perception_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)
