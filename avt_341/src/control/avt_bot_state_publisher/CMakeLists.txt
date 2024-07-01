message(STATUS "Building AVT bot state publisher node ...")

if(USING_CATKIN)
  add_executable(avt_bot_state_publisher_node "avt_bot_state_publisher.cpp")
  target_link_libraries(avt_bot_state_publisher_node ${catkin_LIBRARIES})
  install(
    TARGETS avt_bot_state_publisher_node
    RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}")
endif() # if(USING_CATKIN)

if(USING_AMENT)
  add_executable(avt_bot_state_publisher_node "avt_bot_state_publisher.cpp")
  target_link_libraries(avt_bot_state_publisher_node ${PROJECT_NAME}_proxy)
  install(
    TARGETS avt_bot_state_publisher_node
    EXPORT export_${PROJECT_NAME}
    DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)
