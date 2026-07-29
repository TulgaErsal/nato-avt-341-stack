message(STATUS "Building normal segmentation grid node ...")

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)

add_definitions(${PCL_DEFINITIONS})

add_executable(normal_segmentation_grid_node
               "normal_grid.cpp" "normal_segmentation_grid_node.cpp")

target_include_directories(
  normal_segmentation_grid_node
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(normal_segmentation_grid_node
                      ${PROJECT_NAME}_proxy ${PCL_LIBRARIES})

ament_target_dependencies(normal_segmentation_grid_node
                          pcl_conversions tf2_eigen)

install(
  TARGETS normal_segmentation_grid_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")
