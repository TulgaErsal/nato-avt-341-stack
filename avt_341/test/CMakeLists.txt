message(STATUS "Building Unit Tests")

find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(null_test null_test.cpp)

# MPC Planner Tests
find_package(Julia 1.5.4 EXACT QUIET)
if(Julia_FOUND)
  message(STATUS "Julia found, adding MPC planner unit tests")
  ament_add_gtest(mpc_planner_test mpc_planner_test.cpp)
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
# Source files shared with avt_341_global_path_node (planning algorithms only –
# global_path_node.cpp is intentionally excluded because it contains main()).
set(GLOBAL_PATH_PLANNER_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/../src/planning/global/global_path/astar.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../src/planning/global/global_path/fastmarching.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../src/planning/global/global_path/d_star_lite.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../src/planning/global/global_path/fast_marching_square.cpp"
)

message(STATUS "Adding global path node unit tests")
ament_add_gtest(global_path_node_test
  global_path_node_test.cpp
  ${GLOBAL_PATH_PLANNER_SOURCES}
)

target_include_directories(global_path_node_test
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/../include"
)

target_link_libraries(global_path_node_test
  ${PROJECT_NAME}_proxy
)

ament_target_dependencies(global_path_node_test
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
)

# Inject the path to the YAML parameter file so the test can read real settings.
target_compile_definitions(global_path_node_test
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
)

ament_target_dependencies(static_grid_layer_test
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
)

