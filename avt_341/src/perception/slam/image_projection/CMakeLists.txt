message(STATUS "Building slam image projection node ...")

if(USING_CATKIN)
  # slam image projection node
  if(BUILD_SLAM)
    add_executable(
      ${PROJECT_NAME}_slam_image_projection_node
      "image_projection.cpp"
      )
    add_dependencies(
      ${PROJECT_NAME}_slam_image_projection_node
      ${catkin_EXPORTED_TARGETS}
      )
    target_link_libraries(
      ${PROJECT_NAME}_slam_image_projection_node
      ${PROJECT_NAME}_proxy 
      libCommonLib.so 
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
      ${OpenCV_LIBRARIES}
      )
    install(
      TARGETS ${PROJECT_NAME}_slam_image_projection_node
      RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
      LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
  else()
    message( STATUS "slam image projection node skipped due to BUILD_SLAM == ${BUILD_SLAM}"  )
  endif() # if(BUILD_SLAM)
endif() # if(USING_CATKIN)

if(USING_AMENT)
    add_executable(
      ${PROJECT_NAME}_slam_image_projection_node
      "image_projection.cpp"
      ${CommonLib_FILES}
      )
    message(WARNING "image_projection: ${CommonLib_FILES}")
    target_link_libraries(
      ${PROJECT_NAME}_slam_image_projection_node
      ${PROJECT_NAME}_proxy 
      #CommonLib
      #${CommonLib_LIBRARIES}
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
      ${OpenCV_LIBRARIES}
      )
    install(
      TARGETS ${PROJECT_NAME}_slam_image_projection_node
      EXPORT export_${PROJECT_NAME}
      DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)
