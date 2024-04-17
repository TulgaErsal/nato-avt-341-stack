message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoundingBox2d.msg"
  "msg/Communication.msg"
  "msg/Detection2d.msg"
  "msg/Detection2dArray.msg"
  "msg/FollowerStatus.msg"
  "msg/Hypothesis.msg"
  "msg/Obstacles.msg"
  "msg/OccupiedCell.msg"
  "msg/OccupiedCells.msg"
  "msg/Sinkage.msg"
  DEPENDENCIES
    builtin_interfaces
    nav_msgs
    std_msgs)

ament_export_dependencies(rosidl_default_runtime)

install(FILES "ros_bridge_mappings.yaml"
        DESTINATION share/${PROJECT_NAME})

ament_package()