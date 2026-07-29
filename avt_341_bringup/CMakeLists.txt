cmake_minimum_required(VERSION 3.5)

project(avt_341_bringup)

find_package(ament_cmake REQUIRED)

# Bagging helper scripts
# ======================
# vehicle_logging.py imports ros_param_dump as a sibling module, so both have
# to be installed into the same directory.
install(
  PROGRAMS "bagging/vehicle_logging.py" "bagging/ros_param_dump.py"
  DESTINATION "lib/${PROJECT_NAME}")

install(DIRECTORY "bagging/config" DESTINATION "share/${PROJECT_NAME}/bagging")

# Launch files and deployment data
# ================================
install(DIRECTORY env_data launch parameters rviz urdf
        DESTINATION "share/${PROJECT_NAME}")

ament_package()
