message(STATUS "Building detection training node ...")

install(
  PROGRAMS gym_executor.py
  RENAME ${PROJECT_NAME}_gym_node
  DESTINATION "lib/${PROJECT_NAME}")
