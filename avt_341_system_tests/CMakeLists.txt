cmake_minimum_required(VERSION 3.5)

project(avt_341_system_tests)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python package holding the test drivers
ament_python_install_package(${PROJECT_NAME})

# The test launch files spawn the drivers by file name
# ("ros2 run avt_341_system_tests <driver>.py"), so they are additionally
# installed as executables.
install(
  PROGRAMS "${PROJECT_NAME}/formation_distance_test_driver.py"
           "${PROJECT_NAME}/formation_distance_two_follower_test_driver.py"
           "${PROJECT_NAME}/global_path_test_driver.py"
           "${PROJECT_NAME}/mpc_terminal_heading_test_driver.py"
  DESTINATION "lib/${PROJECT_NAME}")

install(DIRECTORY launch rviz DESTINATION "share/${PROJECT_NAME}")

ament_package()
