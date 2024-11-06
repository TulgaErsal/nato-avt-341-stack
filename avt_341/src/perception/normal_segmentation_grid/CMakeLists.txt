message(STATUS "Building normal segmentation grid node ...")

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)

add_definitions(${PCL_DEFINITIONS})

if(USING_CATKIN)
  # Normal segmentation grid node

  add_executable(${PROJECT_NAME}_normal_segmentation_grid_node
                 "normal_grid.cpp" "normal_segmentation_grid_node.cpp")

  add_dependencies(${PROJECT_NAME}_normal_segmentation_grid_node
                   ${catkin_EXPORTED_TARGETS})

  target_link_libraries(${PROJECT_NAME}_normal_segmentation_grid_node
                        ${PROJECT_NAME}_proxy ${PCL_LIBRARIES})

  install(
    TARGETS ${PROJECT_NAME}_normal_segmentation_grid_node
    RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
endif() # if(USING_CATKIN)

if(USING_AMENT)
  add_executable(${PROJECT_NAME}_normal_segmentation_grid_node
                 "normal_grid.cpp" "normal_segmentation_grid_node.cpp")

  target_include_directories(
    ${PROJECT_NAME}_normal_segmentation_grid_node
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

  target_link_libraries(${PROJECT_NAME}_normal_segmentation_grid_node
                        ${PROJECT_NAME}_proxy ${PCL_LIBRARIES})

  ament_target_dependencies(${PROJECT_NAME}_normal_segmentation_grid_node
                            pcl_conversions tf2_eigen)

  install(
    TARGETS ${PROJECT_NAME}_normal_segmentation_grid_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)
