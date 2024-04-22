# Dynamic window approach planner node
add_executable(${PROJECT_NAME}_dwa_planner_node "../dwa_planner_node.cpp"
                                                "../dwa_planner.cpp")
target_link_libraries(${PROJECT_NAME}_dwa_planner_node ${link_libs}
                      ${PROJECT_NAME}_proxy)
install(
  TARGETS ${PROJECT_NAME}_dwa_planner_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")
