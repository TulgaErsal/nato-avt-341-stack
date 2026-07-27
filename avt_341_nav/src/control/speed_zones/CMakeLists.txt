message(STATUS "Building speed zones node ...")

add_executable(speed_zones_node "speed_zones_node.cpp")
target_link_libraries(speed_zones_node
                      ${PROJECT_NAME}_proxy)
install(
  TARGETS speed_zones_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")

# Generated parameter library (avt_341_param_lib templates)
if(TARGET speed_zones_node)
  target_link_libraries(
    speed_zones_node speed_zones_params_service)
endif()
