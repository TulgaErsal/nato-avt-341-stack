message(STATUS "Building control node ...")

add_executable(
  control_node "control_node.cpp" "../pid_controller.cpp"
                               "../pure_pursuit_controller.cpp")
target_link_libraries(control_node ${PROJECT_NAME}_proxy)
install(
  TARGETS control_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")

# Generated parameter library (avt_341_param_lib templates)
if(TARGET control_node)
  target_link_libraries(control_node control_params_service)
endif()
