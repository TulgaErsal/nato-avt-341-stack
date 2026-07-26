message(STATUS "Building potential field node ...")

if(USING_CATKIN)
  # Potential field planner node
  add_executable(${PROJECT_NAME}_pf_planner_node "pf_planner.cpp"
                                                 "pf_planner_node.cpp")
  add_dependencies(${PROJECT_NAME}_pf_planner_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_pf_planner_node ${PROJECT_NAME}_proxy
                        X11)
  install(
    TARGETS ${PROJECT_NAME}_pf_planner_node
    RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
endif() # if(USING_CATKIN)

if(USING_AMENT)
  add_executable(${PROJECT_NAME}_pf_planner_node "pf_planner_node.cpp"
                                                 "pf_planner.cpp")
  target_link_libraries(${PROJECT_NAME}_pf_planner_node ${PROJECT_NAME}_proxy
                        ${link_libs})
  install(
    TARGETS ${PROJECT_NAME}_pf_planner_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)

# Generated parameter library (avt_341_param_lib templates)
if(USING_AMENT AND TARGET ${PROJECT_NAME}_pf_planner_node)
  target_link_libraries(
    ${PROJECT_NAME}_pf_planner_node pf_local_planner_params_service)
endif()
