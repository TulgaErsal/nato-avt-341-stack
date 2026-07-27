message(STATUS "Building object tracking node ...")

find_package(avt_341_msgs REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(Eigen3 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

if(PCL_FOUND)
  add_definitions(${PCL_DEFINITIONS})
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
endif() # if(PCL_FOUND)

add_executable(object_tracking_node
        "tracker_params.cpp"
        "tracker_recovery_monitor.cpp"
        "object_tracker.cpp"
        "toi_tracker.cpp"
        "formation_vehicle_tracker.cpp"
        "object_tracking_node.cpp"
        "object_tracking_node_executor.cpp")

target_link_libraries(object_tracking_node
  ${PROJECT_NAME}_proxy
)

ament_target_dependencies(
  object_tracking_node
  avt_341_msgs
  Eigen3
  cv_bridge
  OpenCV
  nav_msgs
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  vision_msgs
  visualization_msgs)

install(
  TARGETS object_tracking_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")

# Generated parameter library (avt_341_param_lib templates)
if(TARGET object_tracking_node)
  target_link_libraries(
    object_tracking_node object_tracking_params_service)
endif()
