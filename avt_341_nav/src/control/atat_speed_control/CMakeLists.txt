message(STATUS "Building ATAT speed control node ...")

add_executable(atat_speed_control_node "atat_speed_control_node.cpp")
target_link_libraries(atat_speed_control_node
                      ${PROJECT_NAME}_proxy)
install(
  TARGETS atat_speed_control_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")

# Generated parameter library (avt_341_param_lib templates)
if(TARGET atat_speed_control_node)
  target_link_libraries(
    atat_speed_control_node atat_speed_control_params_service)
endif()
