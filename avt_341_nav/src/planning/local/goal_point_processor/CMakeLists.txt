message(STATUS "Building goal point processor node ...")

add_executable(goal_point_processor_node
  "goal_point_processor_node.cpp"
)
target_link_libraries(goal_point_processor_node
  ${PROJECT_NAME}_proxy
)
install(
  TARGETS goal_point_processor_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}"
)

# Generated parameter library (avt_341_param_lib templates)
if(TARGET goal_point_processor_node)
  target_link_libraries(
    goal_point_processor_node mpc_local_planner_params_service)
endif()
