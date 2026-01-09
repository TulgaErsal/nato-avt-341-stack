message(STATUS "Building global path node ...")

if(USING_CATKIN)
  add_executable(
    ${PROJECT_NAME}_global_path_node
    "global_path_node.cpp"
    "astar.cpp"
    "${PROJECT_SOURCE_DIR}/src/visualization/image_visualizer.cpp"
    fastmarching.cpp
    d_star_lite.cpp
  )
  add_dependencies(${PROJECT_NAME}_global_path_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_global_path_node ${PROJECT_NAME}_proxy
                        X11)
  install(
    TARGETS ${PROJECT_NAME}_global_path_node
    RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
endif() # if(USING_CATKIN)

if(USING_AMENT)
  add_executable(
    ${PROJECT_NAME}_global_path_node
    "global_path_node.cpp"
    "astar.cpp"
    "${PROJECT_SOURCE_DIR}/src/visualization/image_visualizer.cpp"
    fastmarching.cpp
    d_star_lite.cpp
  )
  target_link_libraries(${PROJECT_NAME}_global_path_node ${link_libs}
                        ${PROJECT_NAME}_proxy)
  install(
    TARGETS ${PROJECT_NAME}_global_path_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)
