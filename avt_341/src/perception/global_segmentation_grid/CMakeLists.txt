if(USING_CATKIN)
  message(
    WARN
    "Skipping global segmentation grid node build - not yet supported in ROS 1."
  )
endif() # if(USING_CATKIN)

if(USING_AMENT)
  message(
    STATUS "Downloading the global Keweenaw Research Center (KRC) costmap ...")
  file(
    DOWNLOAD
    https://www.dropbox.com/s/n6img54q02j8tbs/20230816_KRC_CP06_Scaled_FlippedVertical.csv?dl=1
    "${PROJECT_SOURCE_DIR}/config/20230816_KRC_CP06_Scaled_FlippedVertical.csv"
    EXPECTED_HASH
      SHA256=ee537193d028f54ffe08ce9ba78879ee2758fa129942a66f81001382c90fb588)

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
