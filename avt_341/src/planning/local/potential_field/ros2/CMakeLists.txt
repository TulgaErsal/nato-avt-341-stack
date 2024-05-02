# Potential field planner node
add_executable(${PROJECT_NAME}_pf_planner_node "../pf_planner_node.cpp"
                                               "../pf_planner.cpp")
target_link_libraries(${PROJECT_NAME}_pf_planner_node ${PROJECT_NAME}_proxy
                      ${link_libs})
install(
  TARGETS ${PROJECT_NAME}_pf_planner_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")
