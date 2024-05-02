# Grid compression node
add_executable(${PROJECT_NAME}_grid_compression_node
               "../grid_compression_node.cpp")
target_link_libraries(${PROJECT_NAME}_grid_compression_node
                      ${PROJECT_NAME}_proxy)
install(
  TARGETS ${PROJECT_NAME}_grid_compression_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")
