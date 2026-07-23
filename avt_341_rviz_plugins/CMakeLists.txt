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
find_package(rviz_default_plugins REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(avt_341_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# The binary rviz_default_plugins distribution bakes its build machine's absolute
# Qt include path into the imported target's INTERFACE_INCLUDE_DIRECTORIES. That
# path does not exist here, which trips CMake's generate-time existence check.
# Drop any non-existent entries; the real Qt headers come from the locally found
# Qt5::Widgets target we link below.
if(TARGET rviz_default_plugins::rviz_default_plugins)
  get_target_property(_rdp_includes rviz_default_plugins::rviz_default_plugins INTERFACE_INCLUDE_DIRECTORIES)

  if(_rdp_includes)
    # Remove non-existing entries from the list of include directories.
    set(_rdp_includes_existing "")
    foreach(_dir IN LISTS _rdp_includes)
      if(EXISTS "${_dir}")
        list(APPEND _rdp_includes_existing "${_dir}")
      endif()
    endforeach()
    set_target_properties(rviz_default_plugins::rviz_default_plugins PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${_rdp_includes_existing}")
  endif()

endif()

# Use Qt5 to match rviz_common installation. Svg renders the package's SVG icons
# crisply at the display's DPI (see primitives/icon_utils). Gui provides QImage,
# used by the StaticMap display to decode map image files.
find_package(Qt5 REQUIRED COMPONENTS Widgets Svg Gui)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(SRC_FILES
    src/panel_plugins/mission_command_panel.cpp
    src/panel_plugins/autonomy_panel.cpp
    src/panel_plugins/map_markers_panel.cpp
    src/components/setup_component.cpp
    src/components/nav_state_component.cpp
    src/components/mission_component.cpp
    src/components/tracker_component.cpp
    src/components/compute_component.cpp
    src/components/entity_list_component.cpp
    src/components/topic_config_component.cpp
    src/components/vehicle_table_component.cpp
    src/display_plugins/nav_goal_display.cpp
    src/display_plugins/nav_goal_sequence_display.cpp
    src/display_plugins/nav_goal_properties.cpp
    src/display_plugins/map_marker_display.cpp
    src/display_plugins/map_marker_list_display.cpp
    src/display_plugins/map_marker_properties.cpp
    src/display_plugins/static_map_image_display.cpp
    src/tool_plugins/set_nav_goal_tool.cpp
    src/tool_plugins/set_formation_tool.cpp
    src/primitives/accordion_group.cpp
    src/primitives/vehicle_palette.cpp
    src/primitives/icon_header.cpp
    src/primitives/icon_button.cpp
    src/primitives/icon_utils.cpp
    src/primitives/message_label.cpp
    src/primitives/nav_goal_visual.cpp
    src/primitives/map_marker_visual.cpp
    src/primitives/flat_text.cpp
    src/primitives/map_image.cpp
    src/primitives/vector_field.cpp
    src/primitives/matrix_field.cpp)

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
target_link_libraries(${PROJECT_NAME} Qt5::Widgets Qt5::Svg Qt5::Gui)

# Link ROS dependencies
ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  rviz_common
  rviz_rendering
  rviz_default_plugins
  rclcpp
  ament_index_cpp
  avt_341_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
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

# Tool plugin icons
# ============================================================
install(FILES resources/plugin_icons/nav_goal.svg
        DESTINATION "share/${PROJECT_NAME}/icons/classes"
        RENAME "Set Nav Goal.svg")
install(FILES resources/plugin_icons/set_formation.svg
  DESTINATION "share/${PROJECT_NAME}/icons/classes"
  RENAME "Set Formation.svg")

# Panel plugin icons
# ============================================================
install(FILES resources/logo/logo_icon.png
        DESTINATION "share/${PROJECT_NAME}/icons/classes"
        RENAME "AVT-341 Autonomy Stack.png")
install(FILES resources/plugin_icons/map_markers.svg
  DESTINATION "share/${PROJECT_NAME}/icons/classes"
  RENAME "Map Markers.svg")
install(FILES resources/plugin_icons/mission_command.svg
  DESTINATION "share/${PROJECT_NAME}/icons/classes"
  RENAME "Mission Command.svg")

# Display plugin icons
# ============================================================
install(FILES resources/plugin_icons/nav_goal.svg
  DESTINATION "share/${PROJECT_NAME}/icons/classes"
  RENAME "NavGoal.svg")
install(FILES resources/plugin_icons/nav_goal_sequence.svg
  DESTINATION "share/${PROJECT_NAME}/icons/classes"
  RENAME "NavGoalSequence.svg")
install(FILES resources/plugin_icons/map_marker.svg
  DESTINATION "share/${PROJECT_NAME}/icons/classes"
  RENAME "MapMarker.svg")
install(FILES resources/plugin_icons/map_markers.svg
  DESTINATION "share/${PROJECT_NAME}/icons/classes"
  RENAME "MapMarkerList.svg")
install(FILES resources/plugin_icons/static_map_image.svg
  DESTINATION "share/${PROJECT_NAME}/icons/classes"
  RENAME "StaticMapImage.svg")

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
  rviz_default_plugins
  rclcpp
  ament_index_cpp
  avt_341_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

# Export RViz Plugin
pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)

ament_package()
