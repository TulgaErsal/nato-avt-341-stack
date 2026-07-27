message(STATUS "Building segmentation grid processor node ...")

add_executable(segmentation_grid_processor_node
    "segmentation_grid_processor_node.cpp"
)
target_link_libraries(segmentation_grid_processor_node ${dependencies}
    ${PROJECT_NAME}_proxy)
install(
    TARGETS segmentation_grid_processor_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
# Generated parameter library (avt_341_param_lib templates)
if(TARGET segmentation_grid_processor_node)
  target_link_libraries(
    segmentation_grid_processor_node mpc_local_planner_params_service)
endif()
