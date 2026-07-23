message(STATUS "Building obstacles processor node ...")

if(USING_CATKIN)
  add_executable(obstacle_processor_node
    "obstacles_processor_node.cpp"
  )
  add_dependencies(obstacle_processor_node
    ${catkin_EXPORTED_TARGETS})
  target_link_libraries(obstacle_processor_node
    ${PROJECT_NAME}_proxy)
  install(
    TARGETS obstacle_processor_node
    RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
endif() # if(USING_CATKIN)

if(USING_AMENT)
    add_executable(obstacle_processor_node
        "obstacles_processor_node.cpp"
    )
    target_link_libraries(obstacle_processor_node ${dependencies}
        ${PROJECT_NAME}_proxy)
    install(
        TARGETS obstacle_processor_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)
# Generated parameter library (avt_341_param_lib templates)
if(USING_AMENT AND TARGET obstacle_processor_node)
  target_link_libraries(
    obstacle_processor_node mpc_local_planner_params_service)
endif()
