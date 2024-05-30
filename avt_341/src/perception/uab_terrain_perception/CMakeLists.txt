function(DownloadWindowsDll)
    # Download the Windows DLL
    message("Downloading Windows DLL for UAB terrain perception node...")
    file(DOWNLOAD
    "https://www.dropbox.com/scl/fi/elgm351kcurqxngd1c4vj/lib_uab_perception_wrapper.dll?rlkey=ro5uu43knutq9dpd46c9a9k0r&st=6fqeeadg&dl=1"
        "${CMAKE_CURRENT_SOURCE_DIR}/lib_uab_perception_wrapper.dll"
    EXPECTED_HASH SHA256=9f3aa8d240fd99300accfafa57220920b281559bfaa5a2ee5882dd7d0beb844d
    SHOW_PROGRESS)
endfunction(DownloadWindowsDll)

function(DownloadLinuxSharedLibrary)
    # Download the Linux shared library
    message("Downloading Linux shared library for UAB terrain perception node...")
    file(DOWNLOAD
    "https://www.dropbox.com/scl/fi/okkxypdy10kxoe32o4aww/lib_uab_perception_wrapper.so?rlkey=rm8gk7rmubwylibp52esd8f0e&st=fe9g2jvb&dl=1"
        "${CMAKE_CURRENT_SOURCE_DIR}/lib_uab_perception_wrapper.so"
    EXPECTED_HASH SHA256=e32510bdbcd25e9a9f38709ac31090bc51f289ffaa676b29036030716e1134cc
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
# set(Matlab_MCLMCRRT_LIB /usr/local/MATLAB/MATLAB_Runtime/R2023a/runtime/glnxa64/libmwmclmcrrt.so)
# set(Matlab_INCLUDE_DIRS /usr/local/MATLAB/MATLAB_Runtime/R2023a/extern/include)
find_package(Matlab)
if(Matlab_FOUND OR (Matlab_MCLMCRRT_LIB AND Matlab_INCLUDE_DIRS))
    include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${Matlab_INCLUDE_DIRS})

    if(USING_CATKIN)
        add_executable(uab_perception_node "uab_perception_node.cpp")
        add_dependencies(uab_perception_node ${catkin_EXPORTED_TARGETS})

        if(WIN32 OR WIN64)
            DownloadWindowsDll()

            target_link_libraries(uab_perception_node
                ${PROJECT_NAME}_proxy
                ${CMAKE_CURRENT_SOURCE_DIR}/lib_uab_perception_wrapper.lib
                ${Matlab_MCLMCRRT_LIB})
            file(COPY
		${CMAKE_CURRENT_SOURCE_DIR}/lib_uab_perception_wrapper.dll
                DESTINATION ${CMAKE_SOURCE_DIR}/../devel/lib/${PROJECT_NAME})
        else()
            DownloadLinuxSharedLibrary()

            target_link_libraries(uab_perception_node
                ${PROJECT_NAME}_proxy
                ${CMAKE_CURRENT_SOURCE_DIR}/lib_uab_perception_wrapper.so
                ${Matlab_MCLMCRRT_LIB})
            file(COPY
		${CMAKE_CURRENT_SOURCE_DIR}/lib_uab_perception_wrapper.so
                DESTINATION ${CMAKE_SOURCE_DIR}/../devel/lib/${PROJECT_NAME})
        endif()

        install(TARGETS
                uab_perception_node
                RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
                LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    endif() # if(USING_CATKIN)

    if(USING_AMENT)
        add_executable(uab_perception_node "uab_perception_node.cpp")
        target_link_libraries(uab_perception_node ${PROJECT_NAME}_proxy)

        if(WIN32 OR WIN64)
            DownloadWindowsDll()
            
            target_link_libraries(uab_perception_node
                "${CMAKE_CURRENT_SOURCE_DIR}/lib_uab_perception_wrapper.lib"
                ${Matlab_MCLMCRRT_LIB})
            install(FILES "lib_uab_perception_wrapper.dll"
                    DESTINATION "lib/${PROJECT_NAME}")
        else()
            DownloadLinuxSharedLibrary()
        
            target_link_libraries(uab_perception_node
                "${CMAKE_CURRENT_SOURCE_DIR}/lib_uab_perception_wrapper.so"
                ${Matlab_MCLMCRRT_LIB})
            install(FILES "lib_uab_perception_wrapper.so"
                    DESTINATION "lib")
        endif()

        install(TARGETS uab_perception_node
                EXPORT export_${PROJECT_NAME}
                DESTINATION "lib/${PROJECT_NAME}")
    endif() # if(USING_AMENT)
else()
    message(
        WARNING
        "Unable to build UAB Terrain Perception node. Failed to find MATLAB R2023a."
    )
endif()
