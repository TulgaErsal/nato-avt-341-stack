# Perception node
add_executable(${PROJECT_NAME}_perception_node
    "../perception_node.cpp"
    "../costmap_clearing_method.cpp"
    "../elevation_grid.cpp")
target_link_libraries(${PROJECT_NAME}_perception_node
    ${PROJECT_NAME}_proxy)
install(TARGETS ${PROJECT_NAME}_perception_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION "lib/${PROJECT_NAME}")