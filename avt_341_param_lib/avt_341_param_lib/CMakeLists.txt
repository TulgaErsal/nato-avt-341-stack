cmake_minimum_required(VERSION 3.20)
project(avt_341_param_lib)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(fmt REQUIRED)
find_package(rclcpp REQUIRED)

# --- C++ validator library ---
add_library(${PROJECT_NAME} src/parameter_validators.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  fmt::fmt
  rclcpp::rclcpp
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

# --- Windows: install fmt.dll alongside the binaries so it is found at runtime ---
if(WIN32)
  get_target_property(fmt_target_type fmt::fmt TYPE)
  if(fmt_target_type STREQUAL "SHARED_LIBRARY")
    install(FILES $<TARGET_FILE:fmt::fmt> DESTINATION bin)
  endif()
endif()

# --- Python package installation
ament_python_install_package(${PROJECT_NAME})

# --- CMake macros for downstream packages ---
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

# --- Tests ---
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(yaml_parse_error_test ${PROJECT_NAME}/test/YAML_parse_error_test.py)
  ament_add_pytest_test(float_codegen_test ${PROJECT_NAME}/test/float_codegen_test.py)
  ament_add_pytest_test(launch_params_test ${PROJECT_NAME}/test/launch_params_test.py)
  ament_add_pytest_test(launch_node_config_test ${PROJECT_NAME}/test/launch_node_config_test.py)
  ament_add_pytest_test(launch_expressions_test ${PROJECT_NAME}/test/launch_expressions_test.py)
  ament_add_pytest_test(parse_runtime_yaml_test ${PROJECT_NAME}/test/parse_runtime_yaml_test.py)
endif()

# --- Export ---
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(fmt rclcpp)

ament_package(
  CONFIG_EXTRAS "avt_341_param_lib-extras.cmake"
)
