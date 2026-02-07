if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()

# Use Qt5 to match rviz_common installation
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(SRC_FILES mission_command_panel.cpp)
set(HEADER_FILES ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/ui/mission_command_panel.h)

# Create the library target
add_library(${PROJECT_NAME}_ui SHARED ${SRC_FILES} ${HEADER_FILES})
target_include_directories(${PROJECT_NAME}_ui PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link Qt5
target_link_libraries(${PROJECT_NAME}_ui Qt5::Widgets)

# Find and link ROS dependencies
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rclcpp REQUIRED)
find_package(avt_341_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Use ament_target_dependencies to handle ROS packages
ament_target_dependencies(${PROJECT_NAME}_ui
  rviz_common
  rviz_rendering
  rclcpp
  avt_341_msgs
  std_msgs
)

install(TARGETS ${PROJECT_NAME}_ui
  EXPORT export_${PROJECT_NAME}_ui
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME}_ui HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME}_ui)

