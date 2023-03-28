#project(avt_341)

#cmake_minimum_required(VERSION 3.5)

set(CMAKE_COMPILE_WARNING_AS_ERROR OFF)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OccupiedCell.msg"
  "msg/OccupiedCells.msg"
  "msg/Sinkage.msg"
  "msg/Obstacles.msg"
  "msg/Communication.msg"
  "msg/FollowerStatus.msg"
  DEPENDENCIES std_msgs nav_msgs
 )
ament_export_dependencies(rosidl_default_runtime)

if (WIN32 OR WIN64)
set (link_libs
${OpenCV_LIBS}
)
else()
 find_package(X11 REQUIRED)
set (link_libs
${OpenCV_LIBS}
X11
)
endif()

set(dependencies
        rclcpp
        sensor_msgs
        nav_msgs
        geometry_msgs
        visualization_msgs
        std_msgs
        tf2_ros
        )

###########
## Build ##
###########
include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(test_target_detection_node
	src/perception/test_target_detection_node.cpp
	src/node/node_proxy.cpp
)
ament_target_dependencies(test_target_detection_node ${dependencies})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(test_target_detection_node ${cpp_typesupport_target} ${link_libs}) 

add_executable(path_manager_node
	src/planning/global/path_manager_node.cpp
	src/node/node_proxy.cpp
)
ament_target_dependencies(path_manager_node ${dependencies})
target_link_libraries(path_manager_node ${cpp_typesupport_target} ${link_libs}) 

add_executable(avt_341_mission_manager_node
	src/mission/mission_manager_node.cpp
	src/mission/mission_manager.cpp
	src/mission/task_encircle.cpp
	src/mission/task_follow.cpp
	src/mission/task_moveto.cpp
	src/mission/task_wait_until.cpp
	src/node/node_proxy.cpp
)
ament_target_dependencies(avt_341_mission_manager_node ${dependencies})
target_link_libraries(avt_341_mission_manager_node ${cpp_typesupport_target} ${link_libs}) 

add_executable(avt_341_formation_control_node
	src/mission/formation_control_node.cpp
	src/mission/formation_controller.cpp
	src/node/node_proxy.cpp
)
ament_target_dependencies(avt_341_formation_control_node ${dependencies})
target_link_libraries(avt_341_formation_control_node ${cpp_typesupport_target} ${link_libs}) 

add_executable(avt_341_test_formation_control_node
	src/mission/test_formation_control_node.cpp
	src/node/node_proxy.cpp
)
ament_target_dependencies(avt_341_test_formation_control_node ${dependencies})
target_link_libraries(avt_341_test_formation_control_node ${cpp_typesupport_target} ${link_libs}) 

add_executable(avt_341_comm_node
	src/communication/avt_341_comm_node.cpp
	src/node/node_proxy.cpp
)
ament_target_dependencies(avt_341_comm_node ${dependencies})
target_link_libraries(avt_341_comm_node ${cpp_typesupport_target} ${link_libs}) 

add_executable(avt_341_comm_publisher_node
	src/communication/avt_341_comm_publisher_node.cpp
	src/node/node_proxy.cpp
)
ament_target_dependencies(avt_341_comm_publisher_node ${dependencies})
target_link_libraries(avt_341_comm_publisher_node ${cpp_typesupport_target} ${link_libs}) 

add_executable(avt_341_perception_node
        src/perception/avt_341_perception_node.cpp
        src/perception/elevation_grid.cpp
        src/node/node_proxy.cpp
        src/perception/costmap_clearing_method.cpp)
ament_target_dependencies(avt_341_perception_node ${dependencies})
target_link_libraries(avt_341_perception_node ${cpp_typesupport_target} ${link_libs}) 

add_executable(avt_341_map_publisher_node
        src/perception/avt_341_map_publisher_node.cpp
        src/node/node_proxy.cpp
        )
ament_target_dependencies(avt_341_map_publisher_node
        ${dependencies}
        )
target_link_libraries(avt_341_map_publisher_node ${cpp_typesupport_target} ${link_libs}) 

add_executable(avt_341_control_node
        src/control/avt_341_control_node.cpp
        src/control/pure_pursuit_controller.cpp
        src/control/pid_controller.cpp
        src/node/node_proxy.cpp
        )
ament_target_dependencies(avt_341_control_node ${dependencies})
target_link_libraries(avt_341_control_node ${cpp_typesupport_target} ${link_libs}) 

add_executable(avt_341_speed_control_node
        src/control/avt_341_speed_control_node.cpp
        src/control/pid_controller.cpp
        src/node/node_proxy.cpp
        )
ament_target_dependencies(avt_341_speed_control_node ${dependencies})
target_link_libraries(avt_341_speed_control_node ${cpp_typesupport_target} ${link_libs}) 

add_executable(speed_control_test_node
        src/control/speed_control_test_node.cpp
        src/node/node_proxy.cpp
        )
ament_target_dependencies(speed_control_test_node ${dependencies})
target_link_libraries(speed_control_test_node ${cpp_typesupport_target} ${link_libs}) 

add_executable(avt_341_local_planner_node
        src/planning/local/avt_341_local_planner_node.cpp
        src/planning/local/spline_path.cpp
        src/planning/local/spline_planner.cpp
        src/planning/local/spline_plotter.cpp
        src/node/node_proxy.cpp
        src/visualization/image_visualizer.cpp
        src/planning/local/rviz_spline_plotter.cpp
        )
ament_target_dependencies(avt_341_local_planner_node ${dependencies} OpenCV)
target_link_libraries(avt_341_local_planner_node ${cpp_typesupport_target} ${link_libs}) 

add_executable(avt_341_pf_planner_node 
        src/planning/local/avt_341_pf_planner_node.cpp 
        src/planning/local/pf_planner.cpp
        src/node/node_proxy.cpp
        src/visualization/image_visualizer.cpp
      )
ament_target_dependencies(avt_341_pf_planner_node ${dependencies} )
target_link_libraries(avt_341_pf_planner_node ${cpp_typesupport_target} ${link_libs}) 

add_executable(avt_341_dwa_planner_node 
      src/planning/local/avt_341_dwa_planner_node.cpp 
      src/planning/local/dwa_planner.cpp
      src/node/node_proxy.cpp
      src/visualization/image_visualizer.cpp
    )
ament_target_dependencies(avt_341_dwa_planner_node ${dependencies} )
target_link_libraries(avt_341_dwa_planner_node
${cpp_typesupport_target} 
${link_libs}
    )

add_executable(avt_341_global_path_node
        src/planning/global/avt_341_global_path_node.cpp
        src/planning/global/astar.cpp
        src/node/node_proxy.cpp
        src/visualization/image_visualizer.cpp
        )
ament_target_dependencies(avt_341_global_path_node ${dependencies} OpenCV)
target_link_libraries(avt_341_global_path_node
${cpp_typesupport_target} 
${link_libs}
        )

add_executable(avt_341_sim_test_node
        src/simulation/avt_341_sim_test_node.cpp
        src/node/node_proxy.cpp
        src/node/clock_publisher.cpp
        src/perception/point_cloud_generator.cpp
        )
ament_target_dependencies(avt_341_sim_test_node ${dependencies})
target_link_libraries(avt_341_sim_test_node ${cpp_typesupport_target} ${link_libs}) 

add_executable(avt_bot_state_publisher_node
        src/control/avt_bot_state_publisher.cpp
        src/node/node_proxy.cpp
        )
ament_target_dependencies(avt_bot_state_publisher_node ${dependencies})
target_link_libraries(avt_341_control_node ${cpp_typesupport_target} ${link_libs}) 




# # formation control stuff using custom messages
# # see: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html#use-an-interface-from-the-same-package
# rosidl_get_typesupport_target(cpp_typesupport_target
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")

# add_executable(avt_341_formation_control_node
#   src/mission/formation_control_node.cpp
#   src/mission/formation_controller.cpp
#   src/node/node_proxy.cpp
# )

# ament_target_dependencies(avt_341_formation_control_node ${dependencies} )
# target_link_libraries(avt_341_formation_control_node ${link_libs} ${cpp_typesupport_target} )

# add_executable(avt_341_test_formation_control_node
#   src/mission/test_formation_control_node.cpp
#   src/node/node_proxy.cpp
# )
# ament_target_dependencies(avt_341_test_formation_control_node ${dependencies} )
# target_link_libraries(avt_341_test_formation_control_node ${link_libs} ${cpp_typesupport_target})



add_executable(avt_341_grid_compression_node
        src/perception/avt_341_grid_compression_node.cpp
        src/node/node_proxy.cpp
        )
ament_target_dependencies(avt_341_grid_compression_node ${dependencies})
target_link_libraries(avt_341_grid_compression_node ${cpp_typesupport_target} ${link_libs}) 

if (WIN32 OR WIN64)
# this should point to the installation location of MATLAB Runtime
find_package(Matlab)

 if (Matlab_FOUND)
         set(Matlab_MCLMCRRT_LIB "C:\\Program Files\\MATLAB\\MATLAB Runtime\\v912\\extern\\lib\\win64\\microsoft\\mclmcrrt.lib")
         include_directories(
                 include
                 ${OpenCV_INCLUDE_DIRS}
                 ${Matlab_INCLUDE_DIRS}
         )
         add_executable(uab_perception_node
                 src/perception/uab_perception_node.cpp
                 src/node/node_proxy.cpp
         )
         ament_target_dependencies(uab_perception_node ${dependencies})
         target_link_libraries(uab_perception_node
                 ${CMAKE_SOURCE_DIR}/uab_perception/perception_wrapper.lib
                 ${Matlab_MCLMCRRT_LIB}
         )
         install(FILES
                 ${CMAKE_SOURCE_DIR}/uab_perception/perception_wrapper.dll
                 DESTINATION lib/${PROJECT_NAME})
         install(TARGETS
                 uab_perception_node
                 EXPORT export_${PROJECT_NAME}
                 DESTINATION lib/${PROJECT_NAME})
 endif()

endif()

install(DIRECTORY
        launch
        config
        rviz
        DESTINATION share/${PROJECT_NAME}
        )

install(TARGETS
        avt_341_perception_node
        avt_341_map_publisher_node
        avt_341_control_node
        avt_341_speed_control_node
        avt_341_local_planner_node
        avt_341_pf_planner_node
        avt_341_dwa_planner_node
        avt_341_global_path_node
        avt_341_sim_test_node
        avt_bot_state_publisher_node
        speed_control_test_node
        avt_341_comm_node
	avt_341_comm_publisher_node
	avt_341_mission_manager_node
	avt_341_test_formation_control_node
        test_target_detection_node
        avt_341_formation_control_node
        avt_341_grid_compression_node
        EXPORT export_${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})

ament_package()
