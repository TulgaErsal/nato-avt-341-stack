message(STATUS "Building speed zones node ...")

if(USING_CATKIN)
  add_executable(${PROJECT_NAME}_speed_zones_node "speed_zones_node.cpp")
  add_dependencies(${PROJECT_NAME}_speed_zones_node
                   ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_speed_zones_node ${PROJECT_NAME}_proxy)
  install(
    TARGETS ${PROJECT_NAME}_speed_zones_node
    RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
endif() # if(USING_CATKIN)

if(USING_AMENT)
  add_executable(${PROJECT_NAME}_speed_zones_node "speed_zones_node.cpp")
  target_link_libraries(${PROJECT_NAME}_speed_zones_node
                        ${PROJECT_NAME}_proxy)
  install(
    TARGETS ${PROJECT_NAME}_speed_zones_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)

# Generated parameter library (avt_341_param_lib templates)
if(USING_AMENT AND TARGET ${PROJECT_NAME}_speed_zones_node)
  target_link_libraries(
    ${PROJECT_NAME}_speed_zones_node speed_zones_params_service)
endif()
