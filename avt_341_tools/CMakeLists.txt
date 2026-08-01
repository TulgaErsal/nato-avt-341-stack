cmake_minimum_required(VERSION 3.5)

project(avt_341_tools)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Compute-time plotting
# =====================
# The avt_341_tools python module goes to site-packages; its entry script is
# additionally installed under lib so it is reachable via ros2 run.
ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS "${PROJECT_NAME}/plot_compute_time/plot_compute_time.py"
  DESTINATION "lib/${PROJECT_NAME}")

# The remaining top-level folders (annotation, lidar_camera_calibration,
# preprocessing) are standalone development scripts run from the source tree
# and are deliberately not installed.

ament_package()
