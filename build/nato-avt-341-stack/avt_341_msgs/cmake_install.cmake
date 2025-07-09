# Install script for directory: /home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/avt_341_msgs/msg" TYPE FILE FILES
    "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/BoundingBox2d.msg"
    "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Communication.msg"
    "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2d.msg"
    "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Detection2dArray.msg"
    "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaInfo.msg"
    "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaObjective.msg"
    "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/DwaTrajectory.msg"
    "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/FollowerStatus.msg"
    "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Hypothesis.msg"
    "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Obstacles.msg"
    "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCell.msg"
    "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/OccupiedCells.msg"
    "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/Sinkage.msg"
    "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/msg/LiorfCloudInfo.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/avt_341_msgs/srv" TYPE FILE FILES "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/srv/LiorfSaveMap.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/avt_341_msgs/cmake" TYPE FILE FILES "/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341_msgs/catkin_generated/installspace/avt_341_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/vlad/catkin_ws/devel/include/avt_341_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/vlad/catkin_ws/devel/share/roseus/ros/avt_341_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/vlad/catkin_ws/devel/share/common-lisp/ros/avt_341_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/vlad/catkin_ws/devel/share/gennodejs/ros/avt_341_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/vlad/catkin_ws/devel/lib/python3/dist-packages/avt_341_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/vlad/catkin_ws/devel/lib/python3/dist-packages/avt_341_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341_msgs/catkin_generated/installspace/avt_341_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/avt_341_msgs/cmake" TYPE FILE FILES "/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341_msgs/catkin_generated/installspace/avt_341_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/avt_341_msgs/cmake" TYPE FILE FILES
    "/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341_msgs/catkin_generated/installspace/avt_341_msgsConfig.cmake"
    "/home/vlad/catkin_ws/build/nato-avt-341-stack/avt_341_msgs/catkin_generated/installspace/avt_341_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/avt_341_msgs" TYPE FILE FILES "/home/vlad/catkin_ws/src/nato-avt-341-stack/avt_341_msgs/package.xml")
endif()

