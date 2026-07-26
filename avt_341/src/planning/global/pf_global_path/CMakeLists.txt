# Global path node
message(STATUS "Building global path node ...")

if(USING_CATKIN)
  add_executable(
    ${PROJECT_NAME}_pf_global_path_node
    "${PROJECT_SOURCE_DIR}/src/planning/local/potential_field/pf_planner.cpp" "pf_global_path_node.cpp")
  add_dependencies(${PROJECT_NAME}_pf_global_path_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_pf_global_path_node ${PROJECT_NAME}_proxy)
  install(
    TARGETS ${PROJECT_NAME}_pf_global_path_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
  endif() # if(USING_CATKIN)

if(USING_AMENT)
  add_executable(
    ${PROJECT_NAME}_pf_global_path_node
    "${PROJECT_SOURCE_DIR}/src/planning/local/potential_field/pf_planner.cpp" "pf_global_path_node.cpp")
  target_link_libraries(${PROJECT_NAME}_pf_global_path_node ${link_libs}
                        ${PROJECT_NAME}_proxy)
  install(
    TARGETS ${PROJECT_NAME}_pf_global_path_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)
# Generated parameter library (avt_341_param_lib templates)
if(USING_AMENT AND TARGET ${PROJECT_NAME}_pf_global_path_node)
  target_link_libraries(
    ${PROJECT_NAME}_pf_global_path_node pf_global_planner_params_service)
endif()
