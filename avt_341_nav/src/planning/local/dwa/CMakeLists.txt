message(STATUS "Building dynamic window planner node ...")

find_package(Boost REQUIRED)
find_package(OpenMP)

# Dynamic window approach planner node
add_executable(
  dwa_planner_node
  "cells.cpp"
  "dwa_planner_node.cpp"
  "dynamic_window.cpp"
  "obstacles.cpp"
  "path.cpp"
  "planner.cpp"
  "state.cpp"
  "trajectory.cpp")
target_include_directories(
        dwa_planner_node
        PUBLIC $<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>)
target_link_libraries(dwa_planner_node ${Boost_LIBRARIES}
                      ${link_libs} ${PROJECT_NAME}_proxy)
if(OpenMP_FOUND AND NOT (WIN32 OR WIN64))
  target_compile_definitions(dwa_planner_node
                             PUBLIC USE_OPENMP)
  target_link_libraries(dwa_planner_node OpenMP::OpenMP_CXX)

endif() # if(OpenMP_FOUND)
install(
  TARGETS dwa_planner_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")

# Generated parameter library (avt_341_param_lib templates)
if(TARGET dwa_planner_node)
  target_link_libraries(
    dwa_planner_node dwa_local_planner_params_service)
endif()
