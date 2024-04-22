# Local planner node
add_executable(
  ${PROJECT_NAME}_local_planner_node
  "../local_planner_node.cpp" "../rviz_spline_plotter.cpp" "../spline_path.cpp"
  "../spline_planner.cpp" "../spline_plotter.cpp"
  "${PROJECT_SOURCE_DIR}/src/visualization/image_visualizer.cpp")
target_link_libraries(${PROJECT_NAME}_local_planner_node ${PROJECT_NAME}_proxy)
target_link_libraries(${PROJECT_NAME}_local_planner_node ${link_libs})
install(
  TARGETS ${PROJECT_NAME}_local_planner_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")
