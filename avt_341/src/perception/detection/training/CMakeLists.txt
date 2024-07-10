message(STATUS "Building detection training node ...")

if(USING_CATKIN)
  message(
    WARNING
      "Skipping detection training node build - not yet supported in ROS 1.")
endif() # if(USING_CATKIN)

if(USING_AMENT)
  install(
    PROGRAMS gym_executor.py
    RENAME ${PROJECT_NAME}_gym_node
    DESTINATION "lib/${PROJECT_NAME}")
endif() # if(USING_AMENT)
