# AVT bot state publisher node
add_executable(avt_bot_state_publisher_node "../avt_bot_state_publisher.cpp")
target_link_libraries(avt_bot_state_publisher_node ${PROJECT_NAME}_proxy)
install(
  TARGETS avt_bot_state_publisher_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION "lib/${PROJECT_NAME}")
