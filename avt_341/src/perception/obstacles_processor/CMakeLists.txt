message(STATUS "Building obstacles processor node ...")

add_executable(obstacle_processor_node
    "obstacles_processor_node.cpp"
)
target_link_libraries(obstacle_processor_node ${dependencies}
    ${PROJECT_NAME}_proxy)
install(
    TARGETS obstacle_processor_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
# Generated parameter library (avt_341_param_lib templates)
if(TARGET obstacle_processor_node)
  target_link_libraries(
    obstacle_processor_node mpc_local_planner_params_service)
endif()
