if(AVT341_DISABLE_UAB)
    message(STATUS "UAB build disabled.")
    return()
endif()

function(DownloadWindowsDll)
    # Download the Windows DLL
    message("Downloading Windows DLL for UAB terrain perception node...")
    file(DOWNLOAD
    "https://www.dropbox.com/scl/fi/5qonis63wjqlxfe4wzza9/lib_uab_perception_wrapper.dll?rlkey=b61advoh669vxgp96kjvzo8c0&st=7tpsauq6&dl=1"
        "${CMAKE_CURRENT_SOURCE_DIR}/lib_uab_perception_wrapper.dll"
    EXPECTED_HASH SHA256=2061e9823b1372e3a1741731ca249603b70ad45a86bcba427d60186ad5b90d6b
    SHOW_PROGRESS)
endfunction(DownloadWindowsDll)

function(DownloadLinuxSharedLibrary)
    # Download the Linux shared library
    message("Downloading Linux shared library for UAB terrain perception node...")
    file(DOWNLOAD
    "https://www.dropbox.com/scl/fi/3wypl4ru1f3fgqd6vshil/lib_uab_perception_wrapper.so?rlkey=g33m7iqe1jd2zy9w1heob7cj2&st=yphem7z6&dl=1"
        "${CMAKE_CURRENT_SOURCE_DIR}/lib_uab_perception_wrapper.so"
    EXPECTED_HASH SHA256=ce1f3ed3869ca506fcf365610c212ee15dc028efac55c26b88aa06b0c72f4e65
    SHOW_PROGRESS)
endfunction(DownloadLinuxSharedLibrary)


# Manually set MATLAB paths (optional, only needed if "find_package(Matlab)" fails)
# These should point to the installation location of MATLAB Runtime 2025b
#
# ex: Windows Runtime
# set(Matlab_MCLMCRRT_LIB "C:\\Program Files\\MATLAB\\MATLAB Runtime\\R2025b\\extern\\lib\\win64\\microsoft\\mclmcrrt.lib")
# set(Matlab_INCLUDE_DIRS "C:\\Program Files\\MATLAB\\MATLAB Runtime\\R2025b\\extern\\include")
#
# ex: Windows Full Installation
# set(Matlab_MCLMCRRT_LIB "C:\\Program Files\\MATLAB\\R2025b\\extern\\lib\\win64\\microsoft\\mclmcrrt.lib")
# set(Matlab_INCLUDE_DIRS "C:\\Program Files\\MATLAB\\R2025b\\extern\\include")
#
# ex. Linux Runtime
# set(Matlab_MCLMCRRT_LIB /usr/local/MATLAB/MATLAB_Runtime/R2025b/runtime/glnxa64/libmwmclmcrrt.so)
# set(Matlab_INCLUDE_DIRS /usr/local/MATLAB/MATLAB_Runtime/R2025b/extern/include)
find_package(Matlab)
find_package(cv_bridge REQUIRED)

if(Matlab_FOUND OR (Matlab_MCLMCRRT_LIB AND Matlab_INCLUDE_DIRS))

    if(NOT Matlab_MCLMCRRT_LIB)
        if(WIN32 OR WIN64)
             set(Matlab_MCLMCRRT_LIB "C:\\Program Files\\MATLAB\\MATLAB Runtime\\R2025b\\extern\\lib\\win64\\microsoft\\mclmcrrt.lib")
             set(Matlab_INCLUDE_DIRS "C:\\Program Files\\MATLAB\\MATLAB Runtime\\R2025b\\extern\\include")
        else()
             set(Matlab_MCLMCRRT_LIB /usr/local/MATLAB/MATLAB_Runtime/R2025b/runtime/glnxa64/libmwmclmcrrt.so)
             set(Matlab_INCLUDE_DIRS /usr/local/MATLAB/MATLAB_Runtime/R2025b/extern/include)
        endif()

    endif()

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

        ament_target_dependencies(uab_perception_node cv_bridge)

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
        "Unable to build UAB Terrain Perception node. Failed to find MATLAB R2025b."
    )
endif()
