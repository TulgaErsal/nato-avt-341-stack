message(STATUS "Building slam image projection node ...")

if(USING_CATKIN)
  # slam image projection node
  if(BUILD_SLAM)
    add_executable(
      slam_image_projection_node
      "image_projection.cpp"
      )
    add_dependencies(
      slam_image_projection_node
      ${catkin_EXPORTED_TARGETS}
      )
    # Pre-built SLAM common library shipped alongside the reference code.
    target_include_directories(
      slam_image_projection_node
      PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../lib"
      )
    target_link_libraries(
      slam_image_projection_node
      "${CMAKE_CURRENT_LIST_DIR}/../lib/libCommonLib.so"
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
      ${OpenCV_LIBRARIES}
      )
    install(
      TARGETS slam_image_projection_node
      RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
      LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
  else()
    message( STATUS "slam image projection node skipped due to BUILD_SLAM == ${BUILD_SLAM}"  )
  endif() # if(BUILD_SLAM)
endif() # if(USING_CATKIN)

if(USING_AMENT)
  message( WARNING "SLAM is currently not supported in ROS2" )
endif() # if(USING_AMENT)
