message(STATUS "Building VESA bridge node ...")

if(USING_CATKIN)
  message(
    WARNING
      "VESA bridge node not yet supported in the catkin build - skipping...")
endif() # if(USING_CATKIN)

if(USING_AMENT)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(OpenMP REQUIRED)

  add_executable(${PROJECT_NAME}_vesi_bridge_node
                 "vesi_bridge_node.cpp" "vesi_bridge_node_executor.cpp")

  ament_target_dependencies(
    ${PROJECT_NAME}_vesi_bridge_node
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    rosgraph_msgs
    tf2)


  if(WIN32 OR WIN64)
    find_library(VESIAPI_LIB
            NAMES VESIAPI
            PATHS "$ENV{VESIAPI_DIR}"
            PATH_SUFFIXES "lib/windows"
            DOC "Libraries needed to create an rpc client"
            REQUIRED
    )

    message(STATUS "vesiapi lib path: ${VESIAPI_LIB}")

    target_link_libraries(${PROJECT_NAME}_vesi_bridge_node ${VESIAPI_LIB})
    install(FILES "$ENV{VESIAPI_DIR}/lib/windows/VESIAPI.dll" DESTINATION "lib/${PROJECT_NAME}")

  else()

    target_link_libraries(${PROJECT_NAME}_vesi_bridge_node ${VESIAPI_DIR}/lib/linux/libVESIAPI.so)
    install(FILES ${VESIAPI_DIR}/lib/linux/libVESIAPI.so DESTINATION "lib/${PROJECT_NAME}")

  endif()



  install(
    TARGETS ${PROJECT_NAME}_vesi_bridge_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)
