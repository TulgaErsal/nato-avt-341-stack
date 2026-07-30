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
    EXPECTED_HASH SHA256=a07fdd016a6030873b666f4c1f5838aee8a168be3282a3eeb569603fa66a5b7c
    SHOW_PROGRESS)
endfunction(DownloadLinuxSharedLibrary)

# Manually set MATLAB paths (optional, only needed if "find_package(Matlab)" fails)
# These should point to the installation location of MATLAB Runtime 2023a
#
# ex: Windows Runtime
# set(Matlab_MCLMCRRT_LIB "C:\\Program Files\\MATLAB\\MATLAB Runtime\\R2023a\\extern\\lib\\win64\\microsoft\\mclmcrrt.lib")
# set(Matlab_INCLUDE_DIRS "C:\\Program Files\\MATLAB\\MATLAB Runtime\\R2023a\\extern\\include")
#
# ex: Windows Full Installation
set(Matlab_WIN_FULL_EXTERN_DIR "C:/Program Files/MATLAB/R2023a/extern")
if(WIN32 AND EXISTS "${Matlab_WIN_FULL_EXTERN_DIR}")
    set(Matlab_MCLMCRRT_LIB "${Matlab_WIN_FULL_EXTERN_DIR}/lib/win64/microsoft/mclmcrrt.lib")
    set(Matlab_INCLUDE_DIRS "${Matlab_WIN_FULL_EXTERN_DIR}/include")
endif()
#
# ex. Linux Runtime
# set(Matlab_MCLMCRRT_LIB /usr/local/MATLAB/MATLAB_Runtime/R2023a/runtime/glnxa64/libmwmclmcrrt.so)
# set(Matlab_INCLUDE_DIRS /usr/local/MATLAB/MATLAB_Runtime/R2023a/extern/include)
find_package(Matlab)
find_package(cv_bridge REQUIRED)

# Compute the Runtime-only fallback before the availability check below, not inside it.
if(NOT Matlab_MCLMCRRT_LIB)
    if(WIN32)
         set(Matlab_MCLMCRRT_LIB "C:\\Program Files\\MATLAB\\MATLAB Runtime\\R2023a\\extern\\lib\\win64\\microsoft\\mclmcrrt.lib")
         set(Matlab_INCLUDE_DIRS "C:\\Program Files\\MATLAB\\MATLAB Runtime\\R2023a\\extern\\include")
    else()
         # MCR_ROOT matches krc_node_config.yaml / krc_mrzr.launch.py's runtime lookup.
         if(DEFINED ENV{MCR_ROOT})
             set(Matlab_RUNTIME_ROOT $ENV{MCR_ROOT})
         else()
             set(Matlab_RUNTIME_ROOT /usr/local/MATLAB/MATLAB_Runtime/R2023a)
         endif()
         set(Matlab_MCLMCRRT_LIB ${Matlab_RUNTIME_ROOT}/runtime/glnxa64/libmwmclmcrrt.so)
         set(Matlab_INCLUDE_DIRS ${Matlab_RUNTIME_ROOT}/extern/include)
    endif()
endif()

# EXISTS check: fall through to the graceful skip below instead of a link failure.
if(Matlab_FOUND OR (Matlab_MCLMCRRT_LIB AND Matlab_INCLUDE_DIRS AND EXISTS "${Matlab_MCLMCRRT_LIB}"))

    include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${Matlab_INCLUDE_DIRS})

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
else()
    message(
        WARNING
        "Unable to build UAB Terrain Perception node. Failed to find MATLAB R2023a."
    )
endif()

# Generated parameter library (avt_341_param_lib templates)
if(TARGET uab_perception_node)
  target_link_libraries(uab_perception_node uab_perception_params_service)
endif()
