# Dynamic window approach planner node
add_executable(${PROJECT_NAME}_dwa_planner_node "../dwa_planner.cpp"
                                                "../dwa_planner_node.cpp")
add_dependencies(${PROJECT_NAME}_dwa_planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_dwa_planner_node ${PROJECT_NAME}_proxy)
install(
  TARGETS ${PROJECT_NAME}_dwa_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
