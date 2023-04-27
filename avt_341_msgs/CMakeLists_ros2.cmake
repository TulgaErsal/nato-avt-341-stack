message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/OccupiedCell.msg"
        "msg/OccupiedCells.msg"
        "msg/Sinkage.msg"
        "msg/Obstacles.msg"
        "msg/Communication.msg"
        "msg/FollowerStatus.msg"
        DEPENDENCIES std_msgs nav_msgs
        )
ament_export_dependencies(rosidl_default_runtime)



ament_package()
