message(STATUS "Building Unit Tests")

find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(${PROJECT_NAME}_unit_tests null_test.cpp)
