message(STATUS "Building global path node ...")

add_executable(
  global_planner_node
  "global_planner_node.cpp"
  "astar.cpp"
  fastmarching.cpp
  d_star_lite.cpp
  fast_marching_square.cpp
)
target_link_libraries(global_planner_node ${link_libs}
                      ${PROJECT_NAME}_proxy)
install(
  TARGETS global_planner_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")

# Generated parameter library (avt_341_param_lib templates)
if(TARGET global_planner_node)
  target_link_libraries(
    global_planner_node global_planner_params_service)
endif()
