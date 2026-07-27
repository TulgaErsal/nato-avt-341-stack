message(STATUS "Building slam map optimization node ...")

if(USING_CATKIN)
  # slam map optimization node
  if(BUILD_SLAM)
    add_executable(
      slam_map_optimization_node
      "map_optimization.cpp"
      )
    add_dependencies(
      slam_map_optimization_node
      ${catkin_EXPORTED_TARGETS}
      )
    target_compile_options(
      slam_map_optimization_node
      PRIVATE ${OpenMP_CXX_FLAGS}
      )
    # Pre-built SLAM common library shipped alongside the reference code.
    target_include_directories(
      slam_map_optimization_node
      PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../lib"
      )
    target_link_libraries(
      slam_map_optimization_node
      Boost::timer
      "${CMAKE_CURRENT_LIST_DIR}/../lib/libCommonLib.so"
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
      ${OpenCV_LIBRARIES}
      ${OpenMP_CXX_FLAGS}
      ${GeographicLib_LIBRARIES}
      gtsam
      )
    install(
      TARGETS slam_map_optimization_node
      RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
      LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
  else()
    message( STATUS "slam map optimization node skipped due to BUILD_SLAM == ${BUILD_SLAM}"  )
  endif() # if(BUILD_SLAM)
endif() # if(USING_CATKIN)

if(USING_AMENT)
  message( WARNING "SLAM is currently not supported in ROS2" )
endif() # if(USING_AMENT)
