message(STATUS "Building Unit Tests")

find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

# MPC Planner Tests
find_package(Julia 1.5.4 EXACT QUIET)
if(Julia_FOUND)
  message(STATUS "Julia found, adding MPC planner unit tests")
  ament_add_gtest(mpc_planner_test local_planning/mpc_planner_test.cpp)
  target_include_directories(mpc_planner_test PUBLIC "${Julia_INCLUDE_DIRS}")
  target_link_libraries(mpc_planner_test "${Julia_LIBRARY}")
  
  # Set the same definitions as in the MPC planner node
  target_compile_definitions(mpc_planner_test
    PUBLIC
      "MPC_PLANNER_MODULE_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/../src/planning/local/mpc_planner/mpc_commands.jl\""
      "MPC_PARAMETERS_MODULE_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/../src/planning/local/mpc_planner/mpc_parameters.jl\""
      "MPC_MODELS_MODULE_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/../src/planning/local/mpc_planner/mpc_models.jl\""
      "MPC_SYSIMAGE_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/../src/planning/local/mpc_planner/sysimage.so\""
      -DJULIA_ENABLE_THREADING
  )
endif()

# Global Path Node Tests
# ----------------------
# Source files shared with global_planner_node (planning algorithms only –
# global_planner_node.cpp is intentionally excluded because it contains main()).
set(GLOBAL_PATH_PLANNER_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/../src/planning/global/global_path/astar.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../src/planning/global/global_path/fastmarching.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../src/planning/global/global_path/d_star_lite.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../src/planning/global/global_path/fast_marching_square.cpp"
)

message(STATUS "Adding global path node unit tests")
ament_add_gtest(global_planner_node_test
  global_planning/global_planner_node_test.cpp
  ${GLOBAL_PATH_PLANNER_SOURCES}
)

target_include_directories(global_planner_node_test
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/../include"
)

target_link_libraries(global_planner_node_test
  ${PROJECT_NAME}_proxy
)

ament_target_dependencies(global_planner_node_test
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
)

# Inject the path to the YAML parameter file so the test can read real settings.
target_compile_definitions(global_planner_node_test
  PRIVATE
    "GLOBAL_PLANNER_YAML_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/../config/parameters/global_planner.yaml\""
)

# Static Grid Layer Tests
# -----------------------
# ParseSizeInfoFromFile is a static, pure method (no ROS node needed at runtime)
# but the translation unit pulls in CostmapLayer, so we compile the layer sources.
set(STATIC_GRID_LAYER_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/../src/perception/perception/layers/static_grid_layer.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../src/perception/perception/layers/costmap_layer.cpp"
)

message(STATUS "Adding static grid layer unit tests")
ament_add_gtest(static_grid_layer_test
  perception/layers/static_grid_layer_test.cpp
  ${STATIC_GRID_LAYER_SOURCES}
)

target_include_directories(static_grid_layer_test
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/../include"
)

target_link_libraries(static_grid_layer_test
  ${PROJECT_NAME}_proxy
  perception_params_dto
)

message(STATUS "Adding utils unit tests")
ament_add_gtest(utils_test
        core/utils_test.cpp
)

target_include_directories(utils_test
        PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/../include"
)

target_link_libraries(utils_test
        ${PROJECT_NAME}_proxy
)

message(STATUS "Adding waypoint file parser unit tests")
ament_add_gtest(waypoint_file_parser_test
        core/waypoint_file_parser_test.cpp
)

target_include_directories(waypoint_file_parser_test
        PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/../include"
)

target_link_libraries(waypoint_file_parser_test
        ${PROJECT_NAME}_proxy
)

ament_target_dependencies(static_grid_layer_test
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
)

# Object Tracking Reset Tests
# ---------------------------
find_package(avt_341_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

message(STATUS "Adding object tracking reset unit tests")
ament_add_gtest(object_tracking_reset_test
  perception/tracking/object_tracking_reset_test.cpp
  "${CMAKE_CURRENT_SOURCE_DIR}/../src/perception/tracking/object_tracking_node.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../src/perception/tracking/object_tracker.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../src/perception/tracking/toi_tracker.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../src/perception/tracking/formation_vehicle_tracker.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../src/perception/tracking/tracker_params.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../src/perception/tracking/tracker_recovery_monitor.cpp"
)

target_include_directories(object_tracking_reset_test
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/../include"
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(object_tracking_reset_test
  ${PROJECT_NAME}_proxy
  ${PCL_LIBRARIES}
  object_tracking_params_service
)

ament_target_dependencies(object_tracking_reset_test
  avt_341_msgs
  cv_bridge
  Eigen3
  nav_msgs
  OpenCV
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  vision_msgs
  visualization_msgs
)

# Tracker Recovery Monitor Tests
# ------------------------------
message(STATUS "Adding tracker recovery monitor unit tests")
ament_add_gtest(tracker_recovery_monitor_test
  perception/tracking/tracker_recovery_monitor_test.cpp
  "${CMAKE_CURRENT_SOURCE_DIR}/../src/perception/tracking/tracker_params.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../src/perception/tracking/tracker_recovery_monitor.cpp"
)

target_include_directories(tracker_recovery_monitor_test
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/../include"
)

target_link_libraries(tracker_recovery_monitor_test
  ${PROJECT_NAME}_proxy
  object_tracking_params_dto
)

ament_target_dependencies(tracker_recovery_monitor_test
  avt_341_msgs
  Eigen3
  nav_msgs
  rclcpp
  sensor_msgs
  visualization_msgs
)


