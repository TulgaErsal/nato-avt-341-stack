message(STATUS "Building LiDAR normal estimation node ...")

if(USING_CATKIN)
  # LiDAR normal estimation node
  add_executable(${PROJECT_NAME}_lidar_normal_estimation_node
                 "lidar_normal_estimation_node.cpp")
  add_dependencies(
    ${PROJECT_NAME}_lidar_normal_estimation_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_lidar_normal_estimation_node
                        ${catkin_LIBRARIES} ${${PROJECT_NAME}_LIBRARY})
  install(
    TARGETS ${PROJECT_NAME}_lidar_normal_estimation_node
    RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
endif() # if(USING_CATKIN)

if(USING_AMENT)
  # LiDAR normal estimation node
  add_executable(${PROJECT_NAME}_lidar_normal_estimation_node
          "lidar_normal_estimation_node.cpp"
  )
  target_link_libraries(${PROJECT_NAME}_lidar_normal_estimation_node
                        ${PROJECT_NAME}_proxy
                        ${PCL_LIBRARIES})
  install(
      TARGETS ${PROJECT_NAME}_lidar_normal_estimation_node
      EXPORT export_${PROJECT_NAME}
      DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)
