message(STATUS "Building LiDAR obstacle detector node ...")

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

add_definitions(${PCL_DEFINITIONS})

if(USING_CATKIN)
###################### ROS1 BUILD BROKEN #############################
#  # LiDAR obstacle detector node
#  add_executable(${PROJECT_NAME}_lidar_obstacle_detector_node
#                 "lidar_obstacle_detector_node.cpp")
#  add_dependencies(
#    ${PROJECT_NAME}_lidar_obstacle_detector_node
#    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#  target_link_libraries(${PROJECT_NAME}_lidar_obstacle_detector_node
#                        ${catkin_LIBRARIES} ${${PROJECT_NAME}_LIBRARY})
#  install(
#    TARGETS ${PROJECT_NAME}_lidar_obstacle_detector_node
#    RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
#    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
endif() # if(USING_CATKIN)

if(USING_AMENT)

  # Lidar obstacle detector node
  add_executable(${PROJECT_NAME}_lidar_obstacle_detector_node
          "lidar_obstacle_detector_node.cpp"
  )
  target_link_libraries(${PROJECT_NAME}_lidar_obstacle_detector_node
                        ${PROJECT_NAME}_proxy ${${PROJECT_NAME}_LIBRARY} ${PCL_LIBRARIES})

  target_include_directories(
    ${PROJECT_NAME}_lidar_obstacle_detector_node
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(${PROJECT_NAME}_lidar_obstacle_detector_node
                            pcl_conversions)

  install(
      TARGETS ${PROJECT_NAME}_lidar_obstacle_detector_node
      EXPORT export_${PROJECT_NAME}
      DESTINATION "lib/${PROJECT_NAME}")

endif() # if(USING_AMENT)
