cmake_minimum_required(VERSION 3.5)

project(avt_341)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# TODO: Move to UAB perception module folder Download the UAB perception library
file(
  DOWNLOAD https://www.dropbox.com/s/7m0fy3fx3uitxj5/perception_wrapper.dll?dl=1
  "${CMAKE_CURRENT_SOURCE_DIR}/uab_perception/perception_wrapper.dll"
  EXPECTED_HASH
    SHA256=8b1563225bd5e547f375903eb0d5a5a696566091bb65d76476aff4fab452d14f
  SHOW_PROGRESS)

if(WIN32 OR WIN64)
  add_compile_definitions(-D_USE_MATH_DEFINES)
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
    "mission/formation_control"
    "mission/mission_manager"
    "perception/map_publisher"
    "perception/global_segmentation_grid"
    "perception/grid_compression"
    "perception/lidar_obstacle_detector"
    "perception/perception"
    "planning/local/local_planner"
    "planning/local/potential_field"
    "planning/local/dwa"
    "planning/global/global_path")

# Append src/ subdirectory to the path of each target.
string(REGEX REPLACE "([^;]+)" "src/\\1" AVT341_SOURCE_FILES
                     "${AVT341_SOURCE_FILES}")

if(USING_CATKIN)
  set(REQUIRED_ROS_PACKAGES
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

  # Generate dynamic reconfigure parameters in the 'cfg' folder
  generate_dynamic_reconfigure_options("config/lidar_obstacle_detector.cfg")

  catkin_package(
    # INCLUDE_DIRS include LIBRARIES nato_avt_341
    CATKIN_DEPENDS avt_341_msgs # DEPENDS system_lib
  )

  # Build
  include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})

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

  # UAB terrain perception node

  # Manually set MATLAB paths (optional, only needed if find_package(Matlab) fails)
  # These should point to the installation location of MATLAB Runtime 2023a
  #
  # ex: Windows
  # set(Matlab_MCLMCRRT_LIB "C:\\Program Files\\MATLAB\\MATLAB Runtime\\R2023a\\extern\\lib\\win64\\microsoft\\mclmcrrt.lib")
  # set(Matlab_INCLUDE_DIRS "C:\\Program Files\\MATLAB\\MATLAB Runtime\\R2023a\\extern\\include")
  #
  # ex. Linux
  # set(Matlab_MCLMCRRT_LIB /usr/local/MATLAB/MATLAB_Runtime/R2023a/runtime/glnxa64/libmwmclmcrrt.so)
  # set(Matlab_INCLUDE_DIRS /usr/local/MATLAB/MATLAB_Runtime/R2023a/extern/include)

  find_package(Matlab)

  if(Matlab_FOUND OR (Matlab_MCLMCRRT_LIB AND Matlab_INCLUDE_DIRS))
      include_directories(
          include
          ${OpenCV_INCLUDE_DIRS}
          ${Matlab_INCLUDE_DIRS})

      add_executable(uab_perception_node "src/perception/uab_perception_node.cpp")
      add_dependencies(uab_perception_node ${catkin_EXPORTED_TARGETS})

      if(WIN32 OR WIN64)
        # Download the Windows shared library
        file(DOWNLOAD
        "https://www.dropbox.com/scl/fi/elgm351kcurqxngd1c4vj/lib_uab_perception_wrapper.dll?rlkey=ro5uu43knutq9dpd46c9a9k0r&st=6fqeeadg&dl=1"
            "${CMAKE_CURRENT_SOURCE_DIR}/uab_perception/lib_uab_perception_wrapper.dll"
        EXPECTED_HASH SHA256=9f3aa8d240fd99300accfafa57220920b281559bfaa5a2ee5882dd7d0beb844d)

        target_link_libraries(uab_perception_node
            ${PROJECT_NAME}_proxy
            ${CMAKE_SOURCE_DIR}/nato-avt-341-stack/avt_341/uab_perception/lib_uab_perception_wrapper.lib
            ${Matlab_MCLMCRRT_LIB})
        file(COPY
          ${CMAKE_SOURCE_DIR}/nato-avt-341-stack/avt_341/uab_perception/lib_uab_perception_wrapper.dll
          DESTINATION ${CMAKE_SOURCE_DIR}/../devel/lib/${PROJECT_NAME})
      else()
          # Download the Linux shared library
          file(DOWNLOAD
          "https://www.dropbox.com/scl/fi/okkxypdy10kxoe32o4aww/lib_uab_perception_wrapper.so?rlkey=rm8gk7rmubwylibp52esd8f0e&st=fe9g2jvb&dl=1"
              "${CMAKE_CURRENT_SOURCE_DIR}/uab_perception/lib_uab_perception_wrapper.so"
          EXPECTED_HASH SHA256=e32510bdbcd25e9a9f38709ac31090bc51f289ffaa676b29036030716e1134cc)


        target_link_libraries(uab_perception_node
            ${PROJECT_NAME}_proxy
            ${CMAKE_SOURCE_DIR}/nato-avt-341-stack/avt_341/uab_perception/lib_uab_perception_wrapper.so
            ${Matlab_MCLMCRRT_LIB})
        file(COPY
          ${CMAKE_SOURCE_DIR}/nato-avt-341-stack/avt_341/uab_perception/lib_uab_perception_wrapper.so
          DESTINATION ${CMAKE_SOURCE_DIR}/../devel/lib/${PROJECT_NAME})
      endif()

      install(TARGETS
              uab_perception_node
              RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
              LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
  endif()

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

  install(
    DIRECTORY "rviz/"
    DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz"
    FILES_MATCHING
    PATTERN "*.rviz")
endif() # if(USING_CATKIN)

if(USING_AMENT)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(avt_341_msgs REQUIRED)
  find_package(OpenCV REQUIRED)
  find_package(tf2_sensor_msgs REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)

  if(WIN32 OR WIN64)
    set(link_libs ${OpenCV_LIBS})
  else()
    find_package(X11 REQUIRED)
    set(link_libs ${OpenCV_LIBS} X11)
  endif()

  if($ENV{ROS_DISTRO} STREQUAL "humble")
    add_definitions(-DROS_HUMBLE)
  endif()

  # Build
  include_directories(${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})

  if(WIN32 OR WIN64)
    set(Boost_USE_STATIC_LIBS ON)
    find_package(Boost REQUIRED COMPONENTS system date_time regex)
    include_directories(${Boost_INCLUDE_DIRS})
    target_link_directories(${PROJECT_NAME} PRIVATE $ENV{BOOST_LIBRARYDIR})
  endif()

  # Node proxy library
  # ------------------
  add_library(${PROJECT_NAME}_proxy "src/node/node_proxy.cpp")

  target_include_directories(
    ${PROJECT_NAME}_proxy
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(
    ${PROJECT_NAME}_proxy avt_341_msgs rclcpp tf2_geometry_msgs tf2_sensor_msgs
    visualization_msgs)

  foreach(SOURCE_FILE ${AVT341_SOURCE_FILES})
    add_subdirectory(${SOURCE_FILE})
  endforeach()

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

  # UAB terrain perception node

  # Manually set MATLAB paths (optional, only needed if find_package(Matlab) fails)
  # These should point to the installation location of MATLAB Runtime 2023a
  #
  # ex: Windows
  # set(Matlab_MCLMCRRT_LIB "C:\\Program Files\\MATLAB\\MATLAB Runtime\\R2023a\\extern\\lib\\win64\\microsoft\\mclmcrrt.lib")
  # set(Matlab_INCLUDE_DIRS "C:\\Program Files\\MATLAB\\MATLAB Runtime\\R2023a\\extern\\include")
  #
  # ex. Linux
  # set(Matlab_MCLMCRRT_LIB /usr/local/MATLAB/MATLAB_Runtime/R2023a/runtime/glnxa64/libmwmclmcrrt.so)
  # set(Matlab_INCLUDE_DIRS /usr/local/MATLAB/MATLAB_Runtime/R2023a/extern/include)

  find_package(Matlab)

  if(Matlab_FOUND OR (Matlab_MCLMCRRT_LIB AND Matlab_INCLUDE_DIRS))
      include_directories(
          include
          ${OpenCV_INCLUDE_DIRS}
          ${Matlab_INCLUDE_DIRS})

      add_executable(uab_perception_node
          "src/perception/uab_perception_node.cpp")
      target_link_libraries(uab_perception_node
          ${PROJECT_NAME}_proxy)

      if(WIN32 OR WIN64)
          # Download the Windows shared library
          file(DOWNLOAD
          "https://www.dropbox.com/scl/fi/elgm351kcurqxngd1c4vj/lib_uab_perception_wrapper.dll?rlkey=ro5uu43knutq9dpd46c9a9k0r&st=6fqeeadg&dl=1"
              "${CMAKE_CURRENT_SOURCE_DIR}/uab_perception/lib_uab_perception_wrapper.dll"
          EXPECTED_HASH SHA256=9f3aa8d240fd99300accfafa57220920b281559bfaa5a2ee5882dd7d0beb844d)

          target_link_libraries(uab_perception_node
              "${CMAKE_SOURCE_DIR}/uab_perception/lib_uab_perception_wrapper.lib"
              ${Matlab_MCLMCRRT_LIB})
          install(FILES "${CMAKE_SOURCE_DIR}/uab_perception/lib_uab_perception_wrapper.dll"
                  DESTINATION "lib/${PROJECT_NAME}")
      else()
          # Download the Linux shared library
          file(DOWNLOAD
          "https://www.dropbox.com/scl/fi/okkxypdy10kxoe32o4aww/lib_uab_perception_wrapper.so?rlkey=rm8gk7rmubwylibp52esd8f0e&st=fe9g2jvb&dl=1"
              "${CMAKE_CURRENT_SOURCE_DIR}/uab_perception/lib_uab_perception_wrapper.so"
          EXPECTED_HASH SHA256=e32510bdbcd25e9a9f38709ac31090bc51f289ffaa676b29036030716e1134cc)

          target_link_libraries(uab_perception_node
              "${CMAKE_SOURCE_DIR}/uab_perception/lib_uab_perception_wrapper.so"
              ${Matlab_MCLMCRRT_LIB})
          install(FILES "${CMAKE_SOURCE_DIR}/uab_perception/lib_uab_perception_wrapper.so"
                  DESTINATION "lib")
      endif()

      install(TARGETS uab_perception_node
              EXPORT export_${PROJECT_NAME}
              DESTINATION "lib/${PROJECT_NAME}")
  endif()

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

  install(DIRECTORY launch config rviz DESTINATION "share/${PROJECT_NAME}")
  ament_package()
endif() # if(USING_AMENT)
