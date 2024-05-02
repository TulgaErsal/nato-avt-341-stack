# Local occupancy grid node
add_executable(data_acquisition_node
        "../data_acquisition_node.cpp"
)
target_link_libraries(data_acquisition_node
                      ${PROJECT_NAME}_proxy
                      ${PCL_LIBRARIES})
install(
    TARGETS data_acquisition_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
