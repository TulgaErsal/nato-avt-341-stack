cmake_minimum_required(VERSION 3.0.2)
project(trailblazer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  #  LIBRARIES trailblazer
#  CATKIN_DEPENDS trailblazer_core
  #  DEPENDS system_lib
)


include_directories(include ${catkin_INCLUDE_DIRS})

  # Manually set MATLAB paths (optional, only needed if "find_package(Matlab)" fails)
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

    if(USING_CATKIN)
        add_executable(tracker_wrapper_node "tracker_wrapper_node.cpp")
        add_dependencies(tracker_wrapper_node ${catkin_EXPORTED_TARGETS})

        if(WIN32 OR WIN64)
            DownloadWindowsDll()

            target_link_libraries(tracker_wrapper_node
                ${PROJECT_NAME}_proxy
                ${CMAKE_CURRENT_SOURCE_DIR}/lib_tracker_wrapper.lib
                ${Matlab_MCLMCRRT_LIB})
            file(COPY
		${CMAKE_CURRENT_SOURCE_DIR}/lib_tracker_wrapper.dll
                DESTINATION ${CMAKE_SOURCE_DIR}/../devel/lib/${PROJECT_NAME})
        else()
            DownloadLinuxSharedLibrary()

            target_link_libraries(tracker_wrapper_node
                ${PROJECT_NAME}_proxy
                ${CMAKE_CURRENT_SOURCE_DIR}/lib_tracker_wrapper.so
                ${Matlab_MCLMCRRT_LIB})
            file(COPY
		${CMAKE_CURRENT_SOURCE_DIR}/lib_tracker_wrapper.so
                DESTINATION ${CMAKE_SOURCE_DIR}/../devel/lib/${PROJECT_NAME})
        endif()

        install(TARGETS
                tracker_wrapper_node
                RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
                LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    endif() # if(USING_CATKIN)

    if(USING_AMENT)
        add_executable(tracker_wrapper_node "tracker_wrapper_node.cpp")
        target_link_libraries(tracker_wrapper_node ${PROJECT_NAME}_proxy)

        if(WIN32 OR WIN64)
            DownloadWindowsDll()
            
            target_link_libraries(tracker_wrapper_node
                "${CMAKE_CURRENT_SOURCE_DIR}/lib_tracker_wrapper.lib"
                ${Matlab_MCLMCRRT_LIB})
            install(FILES "lib_tracker_wrapper.dll"
                    DESTINATION "lib/${PROJECT_NAME}")
        else()
            DownloadLinuxSharedLibrary()
        
            target_link_libraries(tracker_wrapper_node
                "${CMAKE_CURRENT_SOURCE_DIR}/lib_tracker_wrapper.so"
                ${Matlab_MCLMCRRT_LIB})
            install(FILES "lib_tracker_wrapper.so"
                    DESTINATION "lib")
        endif()

        install(TARGETS tracker_wrapper_node
                EXPORT export_${PROJECT_NAME}
                DESTINATION "lib/${PROJECT_NAME}")
    endif() # if(USING_AMENT)
else()
    message(
        WARNING
        "Unable to build UAB Terrain Perception node. Failed to find MATLAB R2023a."
    )
endif()
