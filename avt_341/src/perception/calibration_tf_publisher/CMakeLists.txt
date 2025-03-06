message(STATUS "Building calibration transform publisher node ...")

if(USING_CATKIN)
  add_executable(calibration_tf_publisher_node "calibration_tf_publisher.cpp")
  add_dependencies(calibration_tf_publisher_node
                   ${catkin_EXPORTED_TARGETS})
  target_link_libraries(calibration_tf_publisher_node
                        ${PROJECT_NAME}_proxy
                        yaml-cpp)
  install(
    TARGETS calibration_tf_publisher_node
    RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
endif() # if(USING_CATKIN)

if(USING_AMENT)
  add_executable(calibration_tf_publisher_node "calibration_tf_publisher.cpp")
  target_link_libraries(calibration_tf_publisher_node
                        ${PROJECT_NAME}_proxy)
  install(
    TARGETS calibration_tf_publisher
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)
