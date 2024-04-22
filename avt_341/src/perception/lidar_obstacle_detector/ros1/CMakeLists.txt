# LiDAR obstacle detector node
add_executable(${PROJECT_NAME}_lidar_obstacle_detector_node
               "../lidar_obstacle_detector_node.cpp")
add_dependencies(
  ${PROJECT_NAME}_lidar_obstacle_detector_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  ${PROJECT_NAME}_lidar_obstacle_detector_node ${catkin_LIBRARIES}
  ${${PROJECT_NAME}_LIBRARY})
install(
  TARGETS ${PROJECT_NAME}_lidar_obstacle_detector_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
