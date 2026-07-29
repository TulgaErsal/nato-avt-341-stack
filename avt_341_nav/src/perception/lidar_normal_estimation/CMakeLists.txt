message(STATUS "Building LiDAR normal estimation node ...")

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

add_definitions(${PCL_DEFINITIONS})

# LiDAR normal estimation node
add_executable(lidar_normal_estimation_node
               "lidar_normal_estimation_node.cpp")

target_link_libraries(lidar_normal_estimation_node
                      ${PROJECT_NAME}_proxy ${PCL_LIBRARIES})

target_include_directories(
  lidar_normal_estimation_node
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(lidar_normal_estimation_node
                          pcl_conversions)

install(
  TARGETS lidar_normal_estimation_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")
