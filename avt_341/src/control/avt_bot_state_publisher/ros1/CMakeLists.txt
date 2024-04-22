# AVT bot state publisher node
add_executable(avt_bot_state_publisher_node "../avt_bot_state_publisher.cpp")
target_link_libraries(avt_bot_state_publisher_node ${catkin_LIBRARIES})
install(
  TARGETS avt_bot_state_publisher_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
