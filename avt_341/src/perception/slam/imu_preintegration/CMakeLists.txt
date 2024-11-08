message(STATUS "Building slam imu preintegration node ...")

if(USING_CATKIN)
  # slam imu preintegration node
  if(BUILD_SLAM)
    add_executable(
      ${PROJECT_NAME}_slam_imu_preintegration_node
      "imu_preintegration.cpp"
      )
    add_dependencies(
      ${PROJECT_NAME}_slam_imu_preintegration_node
      ${catkin_EXPORTED_TARGETS}
      )
    target_link_libraries(
      ${PROJECT_NAME}_slam_imu_preintegration_node
      Boost::timer
      libCommonLib.so 
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
      ${OpenCV_LIBRARIES}
      gtsam
      )
    install(
      TARGETS ${PROJECT_NAME}_slam_imu_preintegration_node
      RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
      LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
  else()
    message( STATUS "slam imu preintegration node skipped due to BUILD_SLAM == ${BUILD_SLAM}"  )
  endif() # if(BUILD_SLAM)
endif() # if(USING_CATKIN)

if(USING_AMENT)
  message( WARNING "SLAM is currently not supported in ROS2" )
endif() # if(USING_AMENT)
