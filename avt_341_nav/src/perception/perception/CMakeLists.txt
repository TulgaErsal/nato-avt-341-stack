message(STATUS "Building perception node ...")

add_executable(
        perception_node
        "perception_node.cpp"
        "clearing_methods/costmap_clearing_method.cpp"
        "clearing_methods/clearing_methods_factory.cpp"
        "clearing_methods/channel_clearing_method.cpp"
        "clearing_methods/raytrace_clearing_method.cpp"
        "clearing_methods/time_clearing_method.cpp"
        "layers/costmap_layer.cpp"
        "layers/point_cloud_layer.cpp"
        "layers/camera_layer.cpp"
        "layers/camera_seg_pc_layer.cpp"
        "layers/polygon_layer.cpp"
        "layers/static_grid_layer.cpp"
        "point_cloud_filter.cpp"
        "costmap.cpp"
)

find_package(image_geometry REQUIRED)
find_package(message_filters REQUIRED)
ament_target_dependencies(
        perception_node image_geometry message_filters)
target_link_libraries(perception_node ${PROJECT_NAME}_proxy)
install(
  TARGETS perception_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")

# Generated parameter library (avt_341_param_lib templates)
if(TARGET perception_node)
  target_link_libraries(perception_node perception_params_service)
endif()
