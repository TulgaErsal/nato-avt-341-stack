cmake_minimum_required(VERSION 3.16)
project(avt_341_param_lib_example)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wold-style-cast)
endif()

find_package(ament_cmake REQUIRED)
find_package(avt_341_param_lib REQUIRED)
find_package(rclcpp REQUIRED)

# Generates DTO and service parameter libraries per yaml file:
# nav.yaml -> nav_params_dto and nav_params_service.
# Mixin templates in parameters/mixins/ are picked up automatically and each
# gets its own shared DTO header, linked into every template DTO library.
avt_341_generate_cpp_parameters(parameters)
# Equivalent explicit per-file form:
# avt_341_generate_cpp_parameter_file(nav parameters/nav.yaml)
# avt_341_generate_cpp_parameter_file(sensor parameters/sensor.yaml)

add_executable(planner_node src/planner_node.cpp)
target_link_libraries(planner_node PUBLIC nav_params_service)

add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node PUBLIC nav_params_service)

add_executable(sensor_node src/sensor_node.cpp)
target_link_libraries(sensor_node PUBLIC sensor_params_service)

# Demonstrates both mixin mechanisms: mixin_ex.yaml inherits inherited_mixin
# and composes composed_mixin.
add_executable(mixin_ex_node src/mixin_ex_node.cpp)
target_link_libraries(mixin_ex_node PUBLIC mixin_ex_params_service)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(float_params_dto_test test/float_params_dto_test.cpp)
  target_link_libraries(float_params_dto_test float_demo_params_dto)

  ament_add_gtest(float_parameters_test test/float_parameters_test.cpp)
  target_link_libraries(float_parameters_test float_demo_params_service)
endif()

install(TARGETS planner_node controller_node sensor_node mixin_ex_node
  DESTINATION lib/${PROJECT_NAME}
)

# The template yaml files under parameters/ are also read at runtime by the
# launch files to declare the parameter override launch arguments
install(DIRECTORY launch parameters parameters_override
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
