message(STATUS "Building detection training node ...")

install(
  PROGRAMS gym_executor.py
  RENAME gym_node
  DESTINATION "lib/${PROJECT_NAME}")
