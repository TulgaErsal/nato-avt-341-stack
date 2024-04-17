message(STATUS "Building the occupancy grid parser node ...")

# Dependencies
# ------------
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenMP)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(avt_341_msgs REQUIRED)
# ------------


# Occupancy grid parser node
# --------------------------
add_executable(${PROJECT_NAME}_occupancy_grid_parser_node
  "obstacle.cpp"
  "occupancy_grid_parser_executor.cpp"
  "occupancy_grid_parser_node.cpp"
  "occupancy_grid.cpp"
  "state.cpp")

ament_target_dependencies(${PROJECT_NAME}_occupancy_grid_parser_node
  avt_341_msgs
  Eigen3
  nav_msgs
  rclcpp
  sensor_msgs
  visualization_msgs)

if(OpenMP_FOUND AND NOT OCCUPANCY_GRID_PARSER_NODE_NO_OPENMP)
  target_link_libraries(${PROJECT_NAME}_occupancy_grid_parser_node
    OpenMP::OpenMP_CXX)

  target_compile_definitions(${PROJECT_NAME}_occupancy_grid_parser_node
    PUBLIC USE_OPENMP)
endif() # if(OpenMP_FOUND AND NOT OCCUPANCY_GRID_PARSER_NODE_NO_OPENMP)

install(TARGETS ${PROJECT_NAME}_occupancy_grid_parser_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})
# --------------------------


# Testing
# _______

# TODO: Move this test node outside of this CMakeLists file.
if(BUILD_TESTING)
  message(STATUS "Building the occupancy grid test node ...")

  find_package(OpenCV REQUIRED)

  # Occupancy grid test node
  # ------------------------
  add_executable(${PROJECT_NAME}_occupancy_grid_test_node
    "occupancy_grid_test_node.cpp")

  ament_target_dependencies(${PROJECT_NAME}_occupancy_grid_test_node
    OpenCV
    nav_msgs
    rclcpp)

  install(TARGETS ${PROJECT_NAME}_occupancy_grid_test_node
          EXPORT export_${PROJECT_NAME}
          DESTINATION lib/${PROJECT_NAME})
  # ------------------------
endif() # if(BUILD_TESTING)
