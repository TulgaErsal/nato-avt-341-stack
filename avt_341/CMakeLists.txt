cmake_minimum_required(VERSION 3.5)

project(avt_341)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


set(BUILD_SLAM OFF)

if (BUILD_SLAM)
  set(CMAKE_BUILD_TYPE "Release")
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if(WIN32 OR WIN64)
  add_compile_definitions(-D _USE_MATH_DEFINES)
endif() # if(WIN32 OR WIN64)

if($ENV{ROS_VERSION} EQUAL 1)
  set(USING_CATKIN ON)

  # cmake-format: off
  message(STATUS "========================================================================")
  message(STATUS " Compiling NATO AVT-341 autonomy stack with CATKIN under ROS ${version} ")
  message(STATUS "========================================================================")
  # cmake-format: on

elseif($ENV{ROS_VERSION} EQUAL 2)
  set(USING_AMENT ON)

  message(STATUS "========================================================")
  message(STATUS " Compiling NATO AVT-341 autonomy stack stack with AMENT ")
  message(STATUS "========================================================")
else()
  message(
    ERROR
    "Unable to detect ROS version. Did you source /opt/ros/${version}/setup.bash?"
  )
endif()

set(AVT341_SOURCE_FILES
    "communication/comms_publisher"
    "communication/communication"
    "control/avt_bot_state_publisher"
    "control/control"
    "control/speed_control"
    "control/speed_control_test"
    "control/speed_zones"
    "daq/data_acquisition"
    "mission/formation_control"
    "mission/mission_manager"
    "perception/detection/common"
    "perception/detection/training"
    "perception/detection/object_detector"
    "perception/tracking"
    "perception/map_publisher"
    "perception/global_segmentation_grid"
    "perception/grid_compression"
    "perception/lidar_obstacle_detector"
    "perception/perception"
    "perception/uab_terrain_perception"
    "perception/obstacles_processor"
    "perception/local_occupancy_grid"
    "perception/lidar_normal_estimation"
    "perception/normal_segmentation_grid"
    "perception/segmentation_grid_processor"
    #"perception/obstacle_tracker"
    "perception/geotiff_map"
    "perception/slam/imu_preintegration"
    "perception/slam/image_projection"
    "perception/slam/map_optimization"
    "planning/local/local_planner"
    "planning/local/potential_field"
    "planning/local/dwa"
    "planning/local/goal_point_processor"
    "planning/local/vehicle_converter"
    "planning/local/mpc_planner"
    "planning/global/global_path"
    "planning/global/pf_global_path")

# Append src/ subdirectory to the path of each target.
string(REGEX REPLACE "([^;]+)" "src/\\1" AVT341_SOURCE_FILES
                     "${AVT341_SOURCE_FILES}")

if(USING_CATKIN)
  set(REQUIRED_ROS_PACKAGES
      ackermann_msgs
      avt_341_msgs
      dynamic_reconfigure
      jsk_recognition_msgs
      message_generation
      nav_msgs
      roscpp
      rospy
      std_msgs
      tf
      tf2_geometry_msgs
      tf2_ros
      tf2_sensor_msgs)

  if($ENV{ROS_DISTRO} STREQUAL "noetic" OR $ENV{ROS_DISTRO} STREQUAL "melodic")
    find_package(catkin REQUIRED COMPONENTS ${REQUIRED_ROS_PACKAGES} pcl_ros)
  else()
    find_package(catkin REQUIRED COMPONENTS ${REQUIRED_ROS_PACKAGES})
  endif()

  add_definitions(-DROS_1)

  find_package(PCL REQUIRED)
  add_definitions(${PCL_DEFINITIONS})

  if (BUILD_SLAM)
    find_package(GTSAM REQUIRED QUIET)
    find_package(Boost REQUIRED COMPONENTS timer)
    find_package(OpenCV REQUIRED QUIET)
    find_package(OpenMP REQUIRED)
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
    find_package(GeographicLib REQUIRED)
  endif()

  # Generate dynamic reconfigure parameters in the 'cfg' folder
  generate_dynamic_reconfigure_options("config/lidar_obstacle_detector.cfg")

  catkin_package(
    # INCLUDE_DIRS include LIBRARIES nato_avt_341
    DEPENDS PCL GTSAM
    CATKIN_DEPENDS avt_341_msgs # DEPENDS system_lib
  )

  # Build
  include_directories(
    include
    lib
    ${catkin_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
	  ${GTSAM_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    )
  link_directories(
    lib
    ${PCL_LIBRARY_DIRS}
    ${GTSAM_LIBRARY_DIRS}
    ${OpenCV_LIBRARY_DIRS}
    )

  # Proxy library
  add_library(${PROJECT_NAME}_proxy "src/node/node_proxy.cpp")
  target_link_libraries(${PROJECT_NAME}_proxy ${catkin_LIBRARIES})

  foreach(SOURCE_FILE ${AVT341_SOURCE_FILES})
    add_subdirectory(${SOURCE_FILE})
  endforeach()

  # GPS to ENU coordinates conversion node
  add_executable(
    gps_to_enu_node
    "src/planning/global/coord_conversions/coord_conversions.cpp"
    "src/planning/global/coord_conversions/ellipsoid.cpp"
    "src/planning/global/coord_conversions/matrix.cpp"
    "src/planning/global/gps_to_enu_node.cpp")
  add_dependencies(gps_to_enu_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(gps_to_enu_node ${PROJECT_NAME}_proxy)
  install(
    TARGETS gps_to_enu_node
    RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")

  # GPS spoof node
  add_executable(gps_spoof_node "src/planning/global/gps_spoof_node.cpp")
  add_dependencies(gps_spoof_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(gps_spoof_node ${catkin_LIBRARIES})
  install(
    TARGETS gps_spoof_node
    RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")

  # Simulation test node
  add_executable(
    ${PROJECT_NAME}_sim_test_node
    "src/node/clock_publisher.cpp" "src/perception/point_cloud_generator.cpp"
    "src/simulation/sim_test_node.cpp")
  add_dependencies(${PROJECT_NAME}_sim_test_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_sim_test_node ${PROJECT_NAME}_proxy
                        ${PCL_LIBRARIES})
  install(
    TARGETS ${PROJECT_NAME}_sim_test_node
    RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")

  # Testing
  # -------

  # Target detection test node
  add_executable(test_target_detection_node
                 "src/perception/test_target_detection_node.cpp")
  add_dependencies(test_target_detection_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_target_detection_node ${PROJECT_NAME}_proxy)
  install(
    TARGETS test_target_detection_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

  # Formation control test node
  add_executable(${PROJECT_NAME}_test_formation_control_node
                 "src/mission/test_formation_control_node.cpp")
  add_dependencies(${PROJECT_NAME}_test_formation_control_node
                   ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_test_formation_control_node
                        ${PROJECT_NAME}_proxy)
  install(
    TARGETS ${PROJECT_NAME}_test_formation_control_node
    RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")

  set(LIB_SOURCES
      "src/control/pid_controller.cpp"
      "src/control/pure_pursuit_controller.cpp"
      "src/node/clock_publisher.cpp"
      "src/perception/costmap_clearing_method.cpp"
      "src/perception/elevation_grid.cpp"
      "src/perception/point_cloud_generator.cpp"
      "src/planning/local/spline_path.cpp"
      "src/planning/local/spline_planner.cpp"
      "src/planning/local/spline_plotter.cpp"
      "src/visualization/image_visualizer.cpp")

  # add_library(avt_341 ${LIB_SOURCES}) add_dependencies(avt_341
  # ${catkin_EXPORTED_TARGETS}) target_link_libraries(avt_341
  # ${PROJECT_NAME}_proxy ${PCL_LIBRARIES} X11)

  catkin_package(INCLUDE_DIRS "include" LIBRARIES ${PROJECT_NAME})

  catkin_install_python(PROGRAMS "launch/vehicle_logging.py"
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  # Install proxy library
  install(
    TARGETS ${PROJECT_NAME}_proxy
    ARCHIVE DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
    RUNTIME DESTINATION "${CATKIN_GLOBAL_BIN_DESTINATION}")

  install(
    DIRECTORY "include/${PROJECT_NAME}/"
    DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
    FILES_MATCHING
    PATTERN "*.h"
    PATTERN "*.hpp")

  install(
    DIRECTORY "launch/"
    DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/launch"
    FILES_MATCHING
    PATTERN "*.launch")

  install(DIRECTORY "config/"
          DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/config")

  install(DIRECTORY "parameters/"
          DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/parameters")

  install(
    DIRECTORY "rviz/"
    DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz"
    FILES_MATCHING
    PATTERN "*.rviz")
endif() # if(USING_CATKIN)

if(USING_AMENT)
  find_package(ackermann_msgs)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(avt_341_msgs REQUIRED)
  find_package(OpenCV REQUIRED)
  find_package(rclpy REQUIRED)
  find_package(tf2_sensor_msgs REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(PCL REQUIRED)

  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  if(WIN32 OR WIN64)
    set(link_libs ${OpenCV_LIBS})
  else()
    find_package(X11 REQUIRED)
    set(link_libs ${OpenCV_LIBS} X11)
  endif()

  # Since ROS Humble several header file endpoints changed (tf2_sensor_msgs, tf2_geometry_msgs, tf2_eigen, etc.)
  if($ENV{ROS_DISTRO} STREQUAL "humble" OR $ENV{ROS_DISTRO} STREQUAL "jazzy")
    add_definitions(-DGTE_ROS_HUMBLE)
  endif()

  # Build
  include_directories(${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})

  # Node proxy library
  # ------------------
  add_library(${PROJECT_NAME}_proxy "src/node/node_proxy.cpp")

  target_include_directories(
    ${PROJECT_NAME}_proxy
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(
    ${PROJECT_NAME}_proxy ackermann_msgs avt_341_msgs rclcpp tf2_geometry_msgs tf2_sensor_msgs
    visualization_msgs)

  foreach(SOURCE_FILE ${AVT341_SOURCE_FILES})
    add_subdirectory(${SOURCE_FILE})
  endforeach()

  # Install the Python package
  ament_python_install_package(${PROJECT_NAME})

  # Simulation test node
  add_executable(
    ${PROJECT_NAME}_sim_test_node
    "src/node/clock_publisher.cpp" "src/perception/point_cloud_generator.cpp"
    "src/simulation/sim_test_node.cpp")
  target_link_libraries(${PROJECT_NAME}_sim_test_node ${PROJECT_NAME}_proxy)
  install(
    TARGETS ${PROJECT_NAME}_sim_test_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")

  # Testing
  # =======

  # Target detection test node
  add_executable(test_target_detection_node
                 "src/perception/test_target_detection_node.cpp")
  target_link_libraries(test_target_detection_node ${PROJECT_NAME}_proxy)
  install(
    TARGETS test_target_detection_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")

  # Formation control test node
  add_executable(${PROJECT_NAME}_test_formation_control_node
                 "src/mission/test_formation_control_node.cpp")
  target_link_libraries(${PROJECT_NAME}_test_formation_control_node
                        ${PROJECT_NAME}_proxy)
  install(
    TARGETS ${PROJECT_NAME}_test_formation_control_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
  
  install(
    PROGRAMS "launch/vehicle_logging.py"
    DESTINATION lib/${PROJECT_NAME})
  
  install(DIRECTORY launch config rviz parameters DESTINATION "share/${PROJECT_NAME}")
  ament_package()
endif() # if(USING_AMENT)
