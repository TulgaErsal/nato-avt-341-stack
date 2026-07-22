cmake_minimum_required(VERSION 3.20)
project(avt_341_param_lib)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(fmt REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# --- C++ validator library ---
add_library(${PROJECT_NAME} src/parameter_validators.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  fmt::fmt
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

# --- Python package installation
ament_python_install_package(${PROJECT_NAME})

# --- CMake macros for downstream packages ---
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

# --- Export ---
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(fmt rclcpp rclcpp_lifecycle)

ament_package(
  CONFIG_EXTRAS "avt_341_param_lib-extras.cmake"
)
