cmake_minimum_required(VERSION 3.16)
project(so101_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_fake_bus_bringup.py TIMEOUT 180)
endif()

ament_package()
