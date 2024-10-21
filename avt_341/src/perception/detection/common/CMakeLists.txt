message(STATUS "Building detection library ...")

if(USING_CATKIN)
  message(
    WARNING
      "Detection library not yet supported in the catkin build - skipping...")

  # add_executable( ${PROJECT_NAME}_object_detector_node
  # "costmap_clearing_method.cpp" "elevation_grid.cpp" "perception_node.cpp")
  # add_dependencies(${PROJECT_NAME}_object_detector_node
  # ${catkin_EXPORTED_TARGETS})
  # target_link_libraries(${PROJECT_NAME}_object_detector_node
  # ${PROJECT_NAME}_proxy) install( TARGETS ${PROJECT_NAME}_object_detector_node
  # RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}" LIBRARY DESTINATION
  # "${CATKIN_PACKAGE_LIB_DESTINATION}")
endif() # if(USING_CATKIN)

if(USING_AMENT)
  find_package(OpenCV REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(vision_msgs REQUIRED)

  add_library(
    ${PROJECT_NAME}_detection_library "bounding_box_2d.cpp" "detection_2d.cpp"
                                      "hypothesis.cpp" "object_visualizer.cpp")

  target_include_directories(
    ${PROJECT_NAME}_detection_library
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(${PROJECT_NAME}_detection_library cv_bridge OpenCV
                            rclcpp sensor_msgs vision_msgs)

  install(
    TARGETS ${PROJECT_NAME}_detection_library
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)
