message(STATUS "Building potential field node ...")

add_executable(${PROJECT_NAME}_pf_planner_node "pf_planner_node.cpp"
                                               "pf_planner.cpp")
target_link_libraries(${PROJECT_NAME}_pf_planner_node ${PROJECT_NAME}_proxy
                      ${link_libs})
install(
  TARGETS ${PROJECT_NAME}_pf_planner_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")

# Generated parameter library (avt_341_param_lib templates)
if(TARGET ${PROJECT_NAME}_pf_planner_node)
  target_link_libraries(
    ${PROJECT_NAME}_pf_planner_node pf_local_planner_params_service)
endif()
