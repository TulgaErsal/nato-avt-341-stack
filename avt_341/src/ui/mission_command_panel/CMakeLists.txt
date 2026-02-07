message(STATUS "Building mission command panel plugin...")


find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rclcpp REQUIRED)
find_package(avt_341_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()

find_package(Qt6 QUIET COMPONENTS Widgets)
if(Qt6_FOUND)
  set(QT_VERSION 6)
else()
  find_package(Qt5 REQUIRED COMPONENTS Widgets)
  set(QT_VERSION 5)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(SRC_FILES mission_command_panel.cpp)
set(HEADER_FILES ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/ui/mission_command_panel.h)

add_library(${PROJECT_NAME}_ui SHARED ${SRC_FILES} ${HEADER_FILES})
target_include_directories(${PROJECT_NAME}_ui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(QT_VERSION EQUAL 6)
  target_link_libraries(${PROJECT_NAME}_ui
    Qt6::Widgets
    rviz_common::rviz_common
    rviz_rendering::rviz_rendering
    rclcpp::rclcpp
    avt_341_msgs::avt_341_msgs__rosidl_typesupport_cpp
    std_msgs::std_msgs__rosidl_typesupport_cpp
  )
else()
  target_link_libraries(${PROJECT_NAME}_ui
    Qt5::Widgets
    rviz_common::rviz_common
    rviz_rendering::rviz_rendering
    rclcpp::rclcpp
    avt_341_msgs::avt_341_msgs__rosidl_typesupport_cpp
    std_msgs::std_msgs__rosidl_typesupport_cpp
  )
endif()

install(TARGETS ${PROJECT_NAME}_ui
  EXPORT export_${PROJECT_NAME}_ui
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME}_ui HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME}_ui)

