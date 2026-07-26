message(STATUS "Building global path node ...")

add_executable(
  ${PROJECT_NAME}_global_path_node
  "global_path_node.cpp"
  "astar.cpp"
  fastmarching.cpp
  d_star_lite.cpp
  fast_marching_square.cpp
)
target_link_libraries(${PROJECT_NAME}_global_path_node ${link_libs}
                      ${PROJECT_NAME}_proxy)
install(
  TARGETS ${PROJECT_NAME}_global_path_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")

# Generated parameter library (avt_341_param_lib templates)
if(TARGET ${PROJECT_NAME}_global_path_node)
  target_link_libraries(
    ${PROJECT_NAME}_global_path_node global_planner_params_service)
endif()
