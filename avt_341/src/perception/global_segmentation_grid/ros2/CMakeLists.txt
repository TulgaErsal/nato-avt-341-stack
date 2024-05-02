# Global segmentation grid node
add_executable(${PROJECT_NAME}_global_segmentation_grid_node
               "../global_segmentation_grid_node.cpp")
target_link_libraries(${PROJECT_NAME}_global_segmentation_grid_node
                      ${PROJECT_NAME}_proxy)
install(
  TARGETS ${PROJECT_NAME}_global_segmentation_grid_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")
