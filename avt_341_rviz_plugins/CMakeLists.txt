cmake_minimum_required(VERSION 3.5)

project(avt_341_rviz_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rclcpp REQUIRED)
find_package(avt_341_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Use Qt5 to match rviz_common installation
find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(SRC_FILES
    src/mission_command_panel.cpp
    src/autonomy_panel.cpp
    src/components/setup_component.cpp
    src/components/nav_state_component.cpp
    src/components/mission_component.cpp
    src/components/tracker_component.cpp
    src/components/compute_component.cpp
    src/components/entity_list_component.cpp
    src/primitives/icon_header.cpp
    src/primitives/icon_button.cpp
    src/primitives/message_label.cpp)

# Headers are not compiled; they are listed only for IDE visibility and so
# CMAKE_AUTOMOC runs moc on the Q_OBJECT classes. Glob recursively so new
# headers under include/ are picked up without editing this list.
# CONFIGURE_DEPENDS re-evaluates the glob at build time.
file(GLOB_RECURSE HEADER_FILES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h)

# Create the plugin library target
add_library(${PROJECT_NAME} SHARED ${SRC_FILES} ${HEADER_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link Qt5
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

# Link ROS dependencies
ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  rviz_common
  rviz_rendering
  rclcpp
  avt_341_msgs
  std_msgs
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES rviz_plugins.xml
  DESTINATION "share/${PROJECT_NAME}")

# RViz resolves a panel's icon by convention at
#   package://<package>/icons/classes/<display name>.{svg,png}
# where <display name> is the segment after the last '/' in the class "name"
# attribute in rviz_plugins.xml.
install(FILES resources/logo/logo_icon.png
  DESTINATION "share/${PROJECT_NAME}/icons/classes"
  RENAME "AVT-341 Autonomy Stack.png")

# Install image assets so they resolve at runtime via rviz_common::loadPixmap
# package:// URLs (e.g. the logo shown in the Setup tab header).
install(DIRECTORY resources
  DESTINATION "share/${PROJECT_NAME}")

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  pluginlib
  rviz_common
  rviz_rendering
  rclcpp
  avt_341_msgs
  std_msgs
)

# Export RViz Plugin
pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)

ament_package()
