message(STATUS "Building slam map optimization node ...")

if(USING_CATKIN)
  # slam map optimization node
  if(BUILD_SLAM) 
    add_executable(
      ${PROJECT_NAME}_slam_map_optimization_node
      "map_optimization.cpp"
      )
    add_dependencies(
      ${PROJECT_NAME}_slam_map_optimization_node
      ${catkin_EXPORTED_TARGETS}
      )
    target_compile_options(
      ${PROJECT_NAME}_slam_map_optimization_node
      PRIVATE ${OpenMP_CXX_FLAGS}
      )
    target_link_libraries(
      ${PROJECT_NAME}_slam_map_optimization_node
      ${PROJECT_NAME}_proxy 
      #Boost::timer
      libCommonLib.so 
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
      ${OpenCV_LIBRARIES}
      ${OpenMP_CXX_FLAGS}
      ${GeographicLib_LIBRARIES}
      gtsam
      )
    install(
      TARGETS ${PROJECT_NAME}_slam_map_optimization_node
      RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
      LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
  else()
    message( STATUS "slam map optimization node skipped due to BUILD_SLAM == ${BUILD_SLAM}"  )
  endif() # if(BUILD_SLAM)
endif() # if(USING_CATKIN)

if(USING_AMENT)
  if(BUILD_SLAM)
    add_executable(
      ${PROJECT_NAME}_slam_map_optimization_node
      "map_optimization.cpp"
      ${CommonLib_FILES}
      )
    target_compile_options(
      ${PROJECT_NAME}_slam_map_optimization_node
      PRIVATE ${OpenMP_CXX_FLAGS}
      )
    target_link_libraries(
      ${PROJECT_NAME}_slam_map_optimization_node
      ${PROJECT_NAME}_proxy 
      #Boost::timer
      #libCommonLib.so 
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
      ${OpenCV_LIBRARIES}
      ${OpenMP_CXX_FLAGS}
      ${GeographicLib_LIBRARIES}
      gtsam
      )
    install(
      TARGETS ${PROJECT_NAME}_slam_map_optimization_node
      EXPORT export_${PROJECT_NAME}
      DESTINATION "lib/${PROJECT_NAME}")
  endif() # if(BUILD_SLAM)
endif() # if(USING_AMENT)
