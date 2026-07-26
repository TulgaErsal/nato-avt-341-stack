message(STATUS "Building detection library ...")

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_msgs REQUIRED)

add_library(
  ${PROJECT_NAME}_detection_library "bounding_box_2d.cpp" "detection_2d.cpp"
                                    "hypothesis.cpp" "object_visualizer.cpp")

target_include_directories(
  ${PROJECT_NAME}_detection_library
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}_detection_library cv_bridge OpenCV
                          rclcpp sensor_msgs vision_msgs)

install(
  TARGETS ${PROJECT_NAME}_detection_library
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")
