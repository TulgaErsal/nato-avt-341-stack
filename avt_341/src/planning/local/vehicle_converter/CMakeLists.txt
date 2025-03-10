if(USING_CATKIN)
    message(STATUS "Building vehicle converter node with catkin ...")
    add_executable(veh_converter_node
        "veh_converter_node.cpp"
    )
    target_link_libraries(veh_converter_node ${dependencies}
        ${PROJECT_NAME}_proxy)
    install(
        TARGETS veh_converter_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif() # if(USING_CATKIN)

if(USING_AMENT)
    message(STATUS "Building vehicle converter node ...")
    add_executable(veh_converter_node
        "veh_converter_node.cpp"
    )
    target_link_libraries(veh_converter_node ${dependencies}
        ${PROJECT_NAME}_proxy)
    install(
        TARGETS veh_converter_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)