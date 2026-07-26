# Global path node
message(STATUS "Building global path node ...")

add_executable(
  ${PROJECT_NAME}_pf_global_path_node
  "${PROJECT_SOURCE_DIR}/src/planning/local/potential_field/pf_planner.cpp" "pf_global_path_node.cpp")
target_link_libraries(${PROJECT_NAME}_pf_global_path_node ${link_libs}
                      ${PROJECT_NAME}_proxy)
install(
  TARGETS ${PROJECT_NAME}_pf_global_path_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")
# Generated parameter library (avt_341_param_lib templates)
if(TARGET ${PROJECT_NAME}_pf_global_path_node)
  target_link_libraries(
    ${PROJECT_NAME}_pf_global_path_node pf_global_planner_params_service)
endif()
