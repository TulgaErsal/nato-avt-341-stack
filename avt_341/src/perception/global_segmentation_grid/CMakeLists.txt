if(USING_CATKIN)
  message(
    WARN
    "Skipping global segmentation grid node build - not yet supported in ROS 1."
  )
endif() # if(USING_CATKIN)

if(USING_AMENT)
  message(STATUS "Building global segmentation grid node ...")
  add_executable(${PROJECT_NAME}_global_segmentation_grid_node
                 "global_segmentation_grid_node.cpp")
  target_link_libraries(${PROJECT_NAME}_global_segmentation_grid_node
                        ${PROJECT_NAME}_proxy)
  install(
    TARGETS ${PROJECT_NAME}_global_segmentation_grid_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)
