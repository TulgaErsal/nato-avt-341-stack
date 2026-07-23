message(STATUS "Building dynamic window planner node ...")

find_package(Boost REQUIRED)
find_package(OpenMP)

if(USING_CATKIN)
  # Dynamic window approach planner node
  add_executable(
    ${PROJECT_NAME}_dwa_planner_node
    "cells.cpp"
    "dwa_planner_node.cpp"
    "dynamic_window.cpp"
    "obstacles.cpp"
    "path.cpp"
    "planner.cpp"
    "state.cpp"
    "trajectory.cpp")
  add_dependencies(${PROJECT_NAME}_dwa_planner_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_dwa_planner_node ${PROJECT_NAME}_proxy
                        ${Boost_LIBRARIES})
  if(OpenMP_FOUND)
    target_compile_definitions(${PROJECT_NAME}_dwa_planner_node
                               PUBLIC USE_OPENMP)
    target_link_libraries(${PROJECT_NAME}_dwa_planner_node OpenMP::OpenMP_CXX)

  endif() # if(OpenMP_FOUND)
  install(
    TARGETS ${PROJECT_NAME}_dwa_planner_node
    RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
endif() # if(USING_CATKIN)

if(USING_AMENT)
  # Dynamic window approach planner node
  add_executable(
    ${PROJECT_NAME}_dwa_planner_node
    "cells.cpp"
    "dwa_planner_node.cpp"
    "dynamic_window.cpp"
    "obstacles.cpp"
    "path.cpp"
    "planner.cpp"
    "state.cpp"
    "trajectory.cpp")
  target_include_directories(
          ${PROJECT_NAME}_dwa_planner_node
          PUBLIC $<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>)
  target_link_libraries(${PROJECT_NAME}_dwa_planner_node ${Boost_LIBRARIES}
                        ${link_libs} ${PROJECT_NAME}_proxy)
  if(OpenMP_FOUND AND NOT (WIN32 OR WIN64))
    target_compile_definitions(${PROJECT_NAME}_dwa_planner_node
                               PUBLIC USE_OPENMP)
    target_link_libraries(${PROJECT_NAME}_dwa_planner_node OpenMP::OpenMP_CXX)

  endif() # if(OpenMP_FOUND)
  install(
    TARGETS ${PROJECT_NAME}_dwa_planner_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)

# Generated parameter library (avt_341_param_lib templates)
if(USING_AMENT AND TARGET ${PROJECT_NAME}_dwa_planner_node)
  target_link_libraries(
    ${PROJECT_NAME}_dwa_planner_node dwa_local_planner_params_service)
endif()
