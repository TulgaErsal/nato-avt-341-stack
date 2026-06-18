function(DownloadWindowsDll)
     # Download the Windows DLL
     message("Downloading Windows DLL and lib for UAB tracker node...")
     file(DOWNLOAD
     "https://www.dropbox.com/scl/fi/zovuqornj4500d8wf0k1p/lib_tracker_wrapper.dll?rlkey=p31u05jabkz1cdbztsonwemgm&st=0dlguoes&dl=1"
         "${CMAKE_CURRENT_SOURCE_DIR}/lib_tracker_wrapper.dll"
     EXPECTED_HASH SHA256=449f3d00e42344b918a40daf4c5158798e03bd73d4cd94aeb3b5887a095c5488
     SHOW_PROGRESS)
     file(DOWNLOAD
     "https://www.dropbox.com/scl/fi/esi44d0xi5864qywryu5d/lib_tracker_wrapper.lib?rlkey=z4sb4tcxt9rd6he9gj7tlzdb6&st=4j15rao1&dl=1"
         "${CMAKE_CURRENT_SOURCE_DIR}/lib_tracker_wrapper.lib"
     EXPECTED_HASH SHA256=115cbf87085da3ef91f5b2fdf5c5fba74962c38be84c566e6b5b0efb6b697023
     SHOW_PROGRESS)
endfunction(DownloadWindowsDll)

function(DownloadLinuxSharedLibrary)
    # Download the Linux shared library
    message("Downloading Linux shared library for UAB tracker node...")
    file(DOWNLOAD
    "https://www.dropbox.com/scl/fi/esi44d0xi5864qywryu5d/lib_tracker_wrapper.lib?rlkey=z4sb4tcxt9rd6he9gj7tlzdb6&st=4j15rao1&dl=1"
        "${CMAKE_CURRENT_SOURCE_DIR}/lib_tracker_wrapper.so"
    EXPECTED_HASH SHA256=93fbfa09187bb644fe1b85874118667356fd4dcda0989964a9d5e850c67a4f13
    SHOW_PROGRESS)
endfunction(DownloadLinuxSharedLibrary)

# Manually set MATLAB paths (optional, only needed if "find_package(Matlab)" fails)
# These should point to the installation location of MATLAB Runtime 2023a
#
# ex: Windows
# set(Matlab_MCLMCRRT_LIB "C:\\Program Files\\MATLAB\\MATLAB Runtime\\R2023a\\extern\\lib\\win64\\microsoft\\mclmcrrt.lib")
# set(Matlab_INCLUDE_DIRS "C:\\Program Files\\MATLAB\\MATLAB Runtime\\R2023a\\extern\\include")
#
# ex. Linux
set(Matlab_MCLMCRRT_LIB /opt/matlab/R2023a/runtime/glnxa64/libmwmclmcrrt.so)
set(Matlab_INCLUDE_DIRS /opt/matlab/R2023a/extern/include)
find_package(Matlab)
if(Matlab_FOUND OR (Matlab_MCLMCRRT_LIB AND Matlab_INCLUDE_DIRS))
    include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${Matlab_INCLUDE_DIRS})

    if(USING_CATKIN)
        add_executable(uab_obstacle_tracker_node "uab_obstacle_tracker_node.cpp")
        add_dependencies(uab_obstacle_tracker_node ${catkin_EXPORTED_TARGETS})

        if(WIN32 OR WIN64)
            DownloadWindowsDll()

            target_link_libraries(uab_obstacle_tracker_node
                ${PROJECT_NAME}_proxy
                ${CMAKE_CURRENT_SOURCE_DIR}/lib_tracker_wrapper.lib
                ${Matlab_MCLMCRRT_LIB})
            file(COPY
		${CMAKE_CURRENT_SOURCE_DIR}/lib_tracker_wrapper.dll
                DESTINATION ${CMAKE_SOURCE_DIR}/../devel/lib/${PROJECT_NAME})
        else()
            DownloadLinuxSharedLibrary()

            target_link_libraries(uab_obstacle_tracker_node
                ${PROJECT_NAME}_proxy
                ${CMAKE_CURRENT_SOURCE_DIR}/lib_tracker_wrapper.so
                ${Matlab_MCLMCRRT_LIB})
            file(COPY
		${CMAKE_CURRENT_SOURCE_DIR}/lib_tracker_wrapper.so
                DESTINATION ${CMAKE_SOURCE_DIR}/../devel/lib/${PROJECT_NAME})
        endif()

        install(TARGETS
                uab_obstacle_tracker_node
                RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
                LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    endif() # if(USING_CATKIN)

    if(USING_AMENT)
        add_executable(uab_obstacle_tracker_node "uab_obstacle_tracker_node.cpp")
        target_link_libraries(uab_obstacle_tracker_node ${PROJECT_NAME}_proxy)

        if(WIN32 OR WIN64)
            DownloadWindowsDll()
            
            target_link_libraries(uab_obstacle_tracker_node
                "${CMAKE_CURRENT_SOURCE_DIR}/lib_tracker_wrapper.lib"
                ${Matlab_MCLMCRRT_LIB})
            install(FILES "lib_tracker_wrapper.dll"
                    DESTINATION "lib/${PROJECT_NAME}")
        else()
            DownloadLinuxSharedLibrary()
        
            target_link_libraries(uab_obstacle_tracker_node
                "${CMAKE_CURRENT_SOURCE_DIR}/lib_tracker_wrapper.so"
                ${Matlab_MCLMCRRT_LIB})
            install(FILES "lib_tracker_wrapper.so"
                    DESTINATION "lib")
        endif()

        install(TARGETS uab_obstacle_tracker_node
                EXPORT export_${PROJECT_NAME}
                DESTINATION "lib/${PROJECT_NAME}")
    endif() # if(USING_AMENT)
else()
    message(
        WARNING
        "Unable to build UAB obstacle tracker node. Failed to find MATLAB R2023a."
    )
endif()
