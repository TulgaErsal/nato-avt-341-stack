message(STATUS "Building road centerline constrained planner node ...")

if(USING_CATKIN)
  add_executable(
    ${PROJECT_NAME}_local_planner_node
    "local_planner_node.cpp" "spline_path.cpp" "spline_planner.cpp"
    "spline_plotter.cpp" "rviz_spline_plotter.cpp"
    "${PROJECT_SOURCE_DIR}/src/visualization/image_visualizer.cpp")
  add_dependencies(${PROJECT_NAME}_local_planner_node
                   ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_local_planner_node
                        ${PROJECT_NAME}_proxy X11)
  install(
    TARGETS ${PROJECT_NAME}_local_planner_node
    RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
endif() # if(USING_CATKIN)

if(USING_AMENT)
  add_executable(
    ${PROJECT_NAME}_local_planner_node
    "local_planner_node.cpp" "rviz_spline_plotter.cpp" "spline_path.cpp"
    "spline_planner.cpp" "spline_plotter.cpp"
    "${PROJECT_SOURCE_DIR}/src/visualization/image_visualizer.cpp")
  target_link_libraries(${PROJECT_NAME}_local_planner_node
                        ${PROJECT_NAME}_proxy)
  target_link_libraries(${PROJECT_NAME}_local_planner_node ${link_libs})
  install(
    TARGETS ${PROJECT_NAME}_local_planner_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)

# Generated parameter library (avt_341_param_lib templates)
if(USING_AMENT AND TARGET ${PROJECT_NAME}_local_planner_node)
  target_link_libraries(
    ${PROJECT_NAME}_local_planner_node rcc_local_planner_params_service)
endif()
