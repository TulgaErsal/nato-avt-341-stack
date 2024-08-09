message(STATUS "Building goal point processor node ...")

if(USING_CATKIN)
  add_executable(goal_point_processor_node
    "goal_point_processor_node.cpp"
  )
  add_dependencies(goal_point_processor_node
    ${catkin_EXPORTED_TARGETS})
  target_link_libraries(goal_point_processor_node
    ${PROJECT_NAME}_proxy)
  install(
    TARGETS goal_point_processor_node
    RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
endif() # if(USING_CATKIN)

if(USING_AMENT)
    add_executable(goal_point_processor_node
        "goal_point_processor_node.cpp"
    )
    target_link_libraries(goal_point_processor_node ${dependencies}
        ${PROJECT_NAME}_proxy)
    install(
        TARGETS goal_point_processor_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)