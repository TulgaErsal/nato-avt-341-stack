cmake_minimum_required(VERSION 3.16)
project(avt_341_param_lib_example)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wold-style-cast)
endif()

find_package(ament_cmake REQUIRED)
find_package(avt_341_param_lib REQUIRED)
find_package(rclcpp REQUIRED)

avt_341_generate_cpp_parameter_file(nav_parameters parameters/nav_params.yaml)
avt_341_generate_cpp_parameter_file(sensor_parameters parameters/sensor.yaml)

add_executable(planner_node src/planner_node.cpp)
target_link_libraries(planner_node PUBLIC nav_parameters rclcpp::rclcpp)

add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node PUBLIC nav_parameters rclcpp::rclcpp)

add_executable(sensor_node src/sensor_node.cpp)
target_link_libraries(sensor_node PUBLIC sensor_parameters rclcpp::rclcpp)

install(TARGETS planner_node controller_node sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

# The template yaml files under parameters/ are also read at runtime by the
# launch files to declare the parameter override launch arguments
install(DIRECTORY launch parameters parameters_override
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
