message(STATUS "Building odometry to tf publisher node ...")

add_executable(odom_to_tf_publisher_node "odom_to_tf_publisher.cpp")
target_link_libraries(odom_to_tf_publisher_node ${PROJECT_NAME}_proxy)
install(
  TARGETS odom_to_tf_publisher_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")
