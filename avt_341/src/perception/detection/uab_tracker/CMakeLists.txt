if($ENV{ROS_VERSION} EQUAL 1)
  set(USING_CATKIN ON)

  # cmake-format: off
  message(STATUS "========================================================================")
  message(STATUS " Compiling UAB Tracker with CATKIN under ROS ${version} ")
  message(STATUS "========================================================================")
  # cmake-format: on
  add_definitions(-DROS_1)
  message("current_src ${CMAKE_CURRENT_SOURCE_DIR}")
  message("src ${CMAKE_SOURCE_DIR}")
  message("project_src ${PROJECT_SOURCE_DIR}")

elseif($ENV{ROS_VERSION} EQUAL 2)
  set(USING_AMENT ON)

  message(STATUS "========================================================")
  message(STATUS " Compiling UAB Tracker with AMENT under ROS ${version}")
  message(STATUS "========================================================")
else()
  message(
    ERROR
    "Unable to detect ROS version. Did you source /opt/ros/${version}/setup.bash?"
  )
endif()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# Manually set MATLAB paths (optional, only needed if "find_package(Matlab)" fails)
# These should point to the installation location of MATLAB Runtime 2023a
#
# ex: Windows
# set(Matlab_MCLMCRRT_LIB "C:\\Program Files\\MATLAB\\MATLAB Runtime\\R2023a\\extern\\lib\\win64\\microsoft\\mclmcrrt.lib")
# set(Matlab_INCLUDE_DIRS "C:\\Program Files\\MATLAB\\MATLAB Runtime\\R2023a\\extern\\include")
#
# ex. Linux
set(Matlab_MCLMCRRT_LIB /usr/local/MATLAB/MATLAB_Runtime/R2023a/runtime/glnxa64/libmwmclmcrrt.so)
set(Matlab_INCLUDE_DIRS /usr/local/MATLAB/MATLAB_Runtime/R2023a/extern/include)

find_package(Matlab)
if (Matlab_FOUND OR (Matlab_MCLMCRRT_LIB AND Matlab_INCLUDE_DIRS))
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Matlab_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
  )

  if (USING_CATKIN)
    add_executable(uab_tracker_wrapper_node
      "uab_tracker_wrapper_node.cpp")
    add_dependencies(uab_tracker_wrapper_node ${catkin_EXPORTED_TARGETS})

    if (WIN32 OR WIN64)
      # Shared library not configured. See uab_perception for how to setup.
#      DownloadWindowsDll()

      target_link_libraries(uab_tracker_wrapper_node
        ${PROJECT_NAME}_proxy
        ${CMAKE_CURRENT_SOURCE_DIR}/lib_tracker_wrapper.lib
        ${Matlab_MCLMCRRT_LIB})
      file(COPY
        ${PROJECT_SOURCE_DIR}/lib/uab_tracker/lib_tracker_wrapper.dll
        DESTINATION ${CMAKE_SOURCE_DIR}/../devel/lib/${PROJECT_NAME})
    else ()
      # Shared library not configured. See uab_perception for how to setup.
#      DownloadLinuxSharedLibrary()

      target_link_libraries(uab_tracker_wrapper_node
        ${PROJECT_NAME}_proxy
        ${PROJECT_SOURCE_DIR}/lib/uab_tracker/lib_tracker_wrapper.so
        ${Matlab_MCLMCRRT_LIB})
      file(COPY
        ${PROJECT_SOURCE_DIR}/lib/uab_tracker/lib_tracker_wrapper.so
        DESTINATION ${CMAKE_SOURCE_DIR}/../devel/lib/${PROJECT_NAME})
    endif ()

    install(TARGETS
      uab_tracker_wrapper_node
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
  endif () # if(USING_CATKIN)

  if (USING_AMENT)
    add_executable(uab_tracker_wrapper_node
      "src/perception/detection/uab_tracker/uab_tracker_wrapper_node.cpp")
    target_link_libraries(uab_tracker_wrapper_node ${PROJECT_NAME}_proxy)

    if (WIN32 OR WIN64)
      DownloadWindowsDll()

      target_link_libraries(uab_tracker_wrapper_node
        "${PROJECT_SOURCE_DIR}/lib/uab_tracker/lib_tracker_wrapper.lib"
        ${Matlab_MCLMCRRT_LIB})
      install(FILES "lib_tracker_wrapper.dll"
        DESTINATION "lib/${PROJECT_NAME}")
    else ()
#      DownloadLinuxSharedLibrary()

      target_link_libraries(uab_tracker_wrapper_node
        "${PROJECT_SOURCE_DIR}/lib/uab_tracker/lib_tracker_wrapper.so"
        ${Matlab_MCLMCRRT_LIB})
      install(FILES  # "lib_tracker_wrapper.so"
        "${PROJECT_SOURCE_DIR}/lib/uab_tracker/lib_tracker_wrapper.so"
        DESTINATION "lib")
    endif ()

    install(TARGETS uab_tracker_wrapper_node
      EXPORT export_${PROJECT_NAME}
      DESTINATION "lib/${PROJECT_NAME}")
  endif () # if(USING_AMENT)
else ()
  message(
    WARNING
    "Unable to build UAB Tracker node. Failed to find MATLAB R2023a."
  )
endif ()
