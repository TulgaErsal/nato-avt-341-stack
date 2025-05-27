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

  # FIXME: The V-ESI ROS node links dynamically to the V-ESI API library.
  # ---------------------------------------------------------------------------
  # target_link_libraries(${PROJECT_NAME}_vesi_bridge_node
  # ${VESIAPI_DIR}/lib/linux/libVESIAPI.so)

  install(
    TARGETS ${PROJECT_NAME}_vesi_bridge_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)
