message(STATUS "Building vehicle converter node ...")
add_executable(veh_converter_node
    "veh_converter_node.cpp"
)
target_link_libraries(veh_converter_node
    ${PROJECT_NAME}_proxy)
install(
    TARGETS veh_converter_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")

# Generated parameter library (avt_341_param_lib templates)
if(TARGET veh_converter_node)
  target_link_libraries(
    veh_converter_node veh_converter_params_service)
endif()
