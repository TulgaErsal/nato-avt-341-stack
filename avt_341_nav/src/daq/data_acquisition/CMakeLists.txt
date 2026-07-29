message(STATUS "Building data aquisition node ...")

# Data aquisition node
add_executable(data_acquisition_node
  "data_acquisition_node.cpp"
)
target_link_libraries(data_acquisition_node
                      ${PROJECT_NAME}_proxy
                      ${PCL_LIBRARIES})
install(
    TARGETS data_acquisition_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")

# Generated parameter library (avt_341_param_lib templates)
if(TARGET data_acquisition_node)
  target_link_libraries(
    data_acquisition_node data_acquisition_params_service)
endif()
