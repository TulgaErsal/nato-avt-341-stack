message(STATUS "Building dynamic window planner node ...")

find_package(Boost REQUIRED)

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
  target_link_libraries(${PROJECT_NAME}_dwa_planner_node ${Boost_LIBRARIES}
                        ${link_libs} ${PROJECT_NAME}_proxy)
  install(
    TARGETS ${PROJECT_NAME}_dwa_planner_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)
