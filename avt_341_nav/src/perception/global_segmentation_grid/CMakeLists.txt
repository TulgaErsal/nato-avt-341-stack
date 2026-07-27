
message(STATUS "Building global segmentation grid node ...")
add_executable(global_segmentation_grid_node
               "global_segmentation_grid_node.cpp")
target_link_libraries(global_segmentation_grid_node
                      ${PROJECT_NAME}_proxy)
install(
  TARGETS global_segmentation_grid_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")
