# Install script for directory: /home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/vlad/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/avt_341" TYPE FILE FILES "/home/vlad/catkin_ws/devel/include/avt_341/lidar_obstacle_detectorConfig.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/avt_341" TYPE FILE FILES "/home/vlad/catkin_ws/devel/lib/python3/dist-packages/avt_341/__init__.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/vlad/catkin_ws/devel/lib/python3/dist-packages/avt_341/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/avt_341" TYPE DIRECTORY FILES "/home/vlad/catkin_ws/devel/lib/python3/dist-packages/avt_341/cfg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/catkin_generated/installspace/avt_341.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/avt_341/cmake" TYPE FILE FILES
    "/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/catkin_generated/installspace/avt_341Config.cmake"
    "/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/catkin_generated/installspace/avt_341Config-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/avt_341" TYPE FILE FILES "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/gps_to_enu_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/gps_to_enu_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/gps_to_enu_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/avt_341" TYPE EXECUTABLE FILES "/home/vlad/catkin_ws/devel/lib/avt_341/gps_to_enu_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/gps_to_enu_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/gps_to_enu_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/gps_to_enu_node"
         OLD_RPATH "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341/lib:/home/vlad/catkin_ws/devel/lib:/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/gps_to_enu_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/gps_spoof_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/gps_spoof_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/gps_spoof_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/avt_341" TYPE EXECUTABLE FILES "/home/vlad/catkin_ws/devel/lib/avt_341/gps_spoof_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/gps_spoof_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/gps_spoof_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/gps_spoof_node"
         OLD_RPATH "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341/lib:/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/gps_spoof_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/avt_341_sim_test_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/avt_341_sim_test_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/avt_341_sim_test_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/avt_341" TYPE EXECUTABLE FILES "/home/vlad/catkin_ws/devel/lib/avt_341/avt_341_sim_test_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/avt_341_sim_test_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/avt_341_sim_test_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/avt_341_sim_test_node"
         OLD_RPATH "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341/lib:/home/vlad/catkin_ws/devel/lib:/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/avt_341_sim_test_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/test_target_detection_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/test_target_detection_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/test_target_detection_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/avt_341" TYPE EXECUTABLE FILES "/home/vlad/catkin_ws/devel/lib/avt_341/test_target_detection_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/test_target_detection_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/test_target_detection_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/test_target_detection_node"
         OLD_RPATH "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341/lib:/home/vlad/catkin_ws/devel/lib:/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/test_target_detection_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/avt_341_test_formation_control_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/avt_341_test_formation_control_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/avt_341_test_formation_control_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/avt_341" TYPE EXECUTABLE FILES "/home/vlad/catkin_ws/devel/lib/avt_341/avt_341_test_formation_control_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/avt_341_test_formation_control_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/avt_341_test_formation_control_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/avt_341_test_formation_control_node"
         OLD_RPATH "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341/lib:/home/vlad/catkin_ws/devel/lib:/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/avt_341/avt_341_test_formation_control_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/catkin_generated/installspace/avt_341.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/avt_341/cmake" TYPE FILE FILES
    "/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/catkin_generated/installspace/avt_341Config.cmake"
    "/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/catkin_generated/installspace/avt_341Config-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/avt_341" TYPE FILE FILES "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/avt_341" TYPE PROGRAM FILES "/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/catkin_generated/installspace/vehicle_logging.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libavt_341_proxy.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libavt_341_proxy.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libavt_341_proxy.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/vlad/catkin_ws/devel/lib/libavt_341_proxy.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libavt_341_proxy.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libavt_341_proxy.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libavt_341_proxy.so"
         OLD_RPATH "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341/lib:/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libavt_341_proxy.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/avt_341" TYPE DIRECTORY FILES "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341/include/avt_341/" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/[^/]*\\.hpp$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/avt_341/launch" TYPE DIRECTORY FILES "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341/launch/" FILES_MATCHING REGEX "/[^/]*\\.launch$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/avt_341/config" TYPE DIRECTORY FILES "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341/config/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/avt_341/parameters" TYPE DIRECTORY FILES "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341/parameters/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/avt_341/rviz" TYPE DIRECTORY FILES "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341/rviz/" FILES_MATCHING REGEX "/[^/]*\\.rviz$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/communication/comms_publisher/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/communication/communication/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/control/avt_bot_state_publisher/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/control/control/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/control/speed_control/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/control/speed_control_test/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/control/speed_zones/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/daq/data_acquisition/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/mission/formation_control/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/mission/mission_manager/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/perception/detection/common/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/perception/detection/training/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/perception/detection/object_detector/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/perception/tracking/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/perception/map_publisher/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/perception/global_segmentation_grid/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/perception/grid_compression/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/perception/lidar_obstacle_detector/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/perception/perception/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/perception/uab_terrain_perception/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/perception/obstacles_processor/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/perception/lidar_normal_estimation/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/perception/normal_segmentation_grid/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/perception/segmentation_grid_processor/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/perception/geotiff_map/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/perception/slam/imu_preintegration/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/perception/slam/image_projection/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/perception/slam/map_optimization/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/planning/local/local_planner/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/planning/local/potential_field/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/planning/local/dwa/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/planning/local/goal_point_processor/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/planning/local/vehicle_converter/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/planning/local/mpc_planner/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/planning/global/global_path/cmake_install.cmake")
  include("/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341/src/planning/global/pf_global_path/cmake_install.cmake")

endif()

