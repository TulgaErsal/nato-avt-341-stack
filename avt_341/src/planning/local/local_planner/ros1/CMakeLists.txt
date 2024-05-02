# Local planner node
add_executable(
  ${PROJECT_NAME}_local_planner_node
  "../local_planner_node.cpp" "../spline_path.cpp" "../spline_planner.cpp"
  "../spline_plotter.cpp" "../rviz_spline_plotter.cpp"
  "${PROJECT_SOURCE_DIR}/src/visualization/image_visualizer.cpp")
add_dependencies(${PROJECT_NAME}_local_planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_local_planner_node ${PROJECT_NAME}_proxy
                      X11)
install(
  TARGETS ${PROJECT_NAME}_local_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
