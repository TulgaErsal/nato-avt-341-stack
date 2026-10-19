message(STATUS "Building KRC speed control python node ...")

# Generated python parameter module krc_speed_control_parameters.py, installed
# into the avt_341_nav python package.
avt_341_generate_python_parameters(${PROJECT_SOURCE_DIR}/parameters/krc_speed_control.yaml)

install(
  PROGRAMS ${PROJECT_SOURCE_DIR}/avt_341_nav/control/krc_speed_control_node.py
  DESTINATION "lib/${PROJECT_NAME}")
