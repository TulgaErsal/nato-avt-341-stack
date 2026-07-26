message(STATUS "Building goal point processor node ...")

if(USING_CATKIN)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
  )
  include_directories(
    ${catkin_INCLUDE_DIRS}
  )
  add_executable(goal_point_processor_node
    "goal_point_processor_node.cpp"
  )
  add_dependencies(goal_point_processor_node
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(goal_point_processor_node
    ${PROJECT_NAME}_proxy
    ${catkin_LIBRARIES}
  )
  install(
    TARGETS goal_point_processor_node
    RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
  )
endif()

if(USING_AMENT)
  add_executable(goal_point_processor_node
    "goal_point_processor_node.cpp"
  )
  target_link_libraries(goal_point_processor_node
    ${dependencies}
    ${PROJECT_NAME}_proxy
  )
  install(
    TARGETS goal_point_processor_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}"
  )
endif()

# Generated parameter library (avt_341_param_lib templates)
if(USING_AMENT AND TARGET goal_point_processor_node)
  target_link_libraries(
    goal_point_processor_node mpc_local_planner_params_service)
endif()
