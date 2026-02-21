message(STATUS "Building Unit Tests")

find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(${PROJECT_NAME}_unit_tests null_test.cpp)

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
