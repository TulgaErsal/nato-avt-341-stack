# ROS1 (catkin) reference build wiring for the SLAM nodes.
#
# SLAM is ROS1-only and is kept as reference code: nothing in the current
# ament (ROS2) build of this package includes this directory. To build it,
# a catkin-based root CMakeLists.txt must:
#   * set(USING_CATKIN ON) and add_definitions(-DROS_1)
#   * find_package(catkin REQUIRED COMPONENTS ... pcl_ros) and
#     find_package(PCL REQUIRED), with
#     include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
#   * set(BUILD_SLAM ON) and CMAKE_BUILD_TYPE "Release" (see README.md)
#   * declare "DEPENDS PCL GTSAM" in catkin_package()
#   * add_subdirectory(src/perception/slam)

message(STATUS "Configuring SLAM (ROS1-only reference code) ...")

if(USING_CATKIN)
  if(BUILD_SLAM)
    # Third-party dependencies formerly located in the root CMakeLists.txt.
    find_package(GTSAM REQUIRED QUIET)
    find_package(Boost REQUIRED COMPONENTS timer)
    find_package(OpenCV REQUIRED QUIET)
    find_package(OpenMP REQUIRED)
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
    find_package(GeographicLib REQUIRED)

    include_directories(${GTSAM_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
    link_directories(${GTSAM_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})
  endif()
endif()

add_subdirectory(image_projection)
add_subdirectory(imu_preintegration)
add_subdirectory(map_optimization)
