# Global path planner node
add_executable(
  ${PROJECT_NAME}_global_path_node
  "../astar.cpp" "../global_path_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/visualization/image_visualizer.cpp")
target_link_libraries(${PROJECT_NAME}_global_path_node ${link_libs}
                      ${PROJECT_NAME}_proxy)
install(
  TARGETS ${PROJECT_NAME}_global_path_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")
