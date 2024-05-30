message(STATUS "Building formation control node ...")

if(USING_CATKIN)
  add_executable(
    ${PROJECT_NAME}_formation_control_node
    "formation_control_node.cpp" "formation_controller.cpp"
    "../formation_utils.cpp")
  add_dependencies(${PROJECT_NAME}_formation_control_node
                   ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_formation_control_node
                        ${PROJECT_NAME}_proxy)
  install(
    TARGETS ${PROJECT_NAME}_formation_control_node
    RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
endif() # if(USING_CATKIN)

if(USING_AMENT)
  add_executable(
    ${PROJECT_NAME}_formation_control_node
    "formation_control_node.cpp" "formation_controller.cpp"
    "../formation_utils.cpp")
  target_link_libraries(${PROJECT_NAME}_formation_control_node
                        ${PROJECT_NAME}_proxy)
  install(
    TARGETS ${PROJECT_NAME}_formation_control_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)
