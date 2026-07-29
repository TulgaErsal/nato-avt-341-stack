message(STATUS "Building speed control node ...")

add_executable(speed_control_node "speed_control_node.cpp"
                                                  "../pid_controller.cpp")
target_link_libraries(speed_control_node
                      ${PROJECT_NAME}_proxy)
install(
  TARGETS speed_control_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")

# Generated parameter library (avt_341_param_lib templates)
if(TARGET speed_control_node)
  target_link_libraries(
    speed_control_node speed_control_params_service)
endif()
