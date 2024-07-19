# Global path node
add_executable(
  ${PROJECT_NAME}_global_path_node
  "${PROJECT_SOURCE_DIR}/src/planning/local/potential_field/pf_planner.cpp" "../pf_global_path_node.cpp")
add_dependencies(${PROJECT_NAME}_pf_global_path_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_pf_global_path_node ${PROJECT_NAME}_proxy
                      X11)
install(
  TARGETS ${PROJECT_NAME}_pf_global_path_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
