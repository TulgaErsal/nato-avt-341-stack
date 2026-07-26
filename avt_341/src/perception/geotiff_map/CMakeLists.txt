find_package(GDAL CONFIG)

if(GDAL_FOUND)

  if(NOT TARGET GDAL::GDAL)
    add_library(GDAL::GDAL ALIAS ${GDAL_LIBRARY})
  endif()
  include_directories(${GDAL_INCLUDE_DIRS})

  message(STATUS "Building geotiff map node ...")

  add_executable(avt_341_geotiff_map_publisher_node
          "avt_341_geotiff_map_publisher_node.cpp"
          "geotiff_dataset.cpp"
  )

  target_link_libraries(${PROJECT_NAME}_geotiff_map_publisher_node
          ${PROJECT_NAME}_proxy
          GDAL::GDAL)
  install(
          TARGETS ${PROJECT_NAME}_geotiff_map_publisher_node
          EXPORT export_${PROJECT_NAME}
          DESTINATION "lib/${PROJECT_NAME}")

else()

  message(WARNING "GDAL Library not found. Geotiff map publisher node will not be run.")

endif()
