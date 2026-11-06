cmake_minimum_required(VERSION 3.16)
project(so101_hardware LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type -Werror=unused-variable)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nlohmann_json REQUIRED)

# --- ROS-independent core: protocol, serial port, calibration, simulator ----
add_library(so101_feetech STATIC
  src/feetech/protocol.cpp
  src/feetech/serial_port.cpp
  src/calibration.cpp
  src/joint_mapping.cpp
  src/sim/servo_bus_sim.cpp
)
target_include_directories(so101_feetech PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(so101_feetech PUBLIC nlohmann_json::nlohmann_json)
set_target_properties(so101_feetech PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- ros2_control plugin -----------------------------------------------------
add_library(${PROJECT_NAME} SHARED src/so101_system.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC
  so101_feetech
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(hardware_interface so101_hardware.xml)

# --- tools -------------------------------------------------------------------
add_executable(fake_servo_bus tools/fake_servo_bus.cpp)
target_link_libraries(fake_servo_bus so101_feetech)
add_executable(so101_bus_tool tools/so101_bus_tool.cpp)
target_link_libraries(so101_bus_tool so101_feetech)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS so101_feetech ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS fake_servo_bus so101_bus_tool DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # Formatting/style linters are left out on purpose; cppcheck and xmllint
  # catch real defects.
  set(ament_cmake_cppcheck_LANGUAGE c++)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol so101_feetech)

  ament_add_gtest(test_joint_mapping test/test_joint_mapping.cpp)
  target_link_libraries(test_joint_mapping so101_feetech)

  ament_add_gtest(test_so101_system test/test_so101_system.cpp)
  target_link_libraries(test_so101_system ${PROJECT_NAME})
  target_compile_definitions(test_so101_system PRIVATE
    EXAMPLE_CALIBRATION="${CMAKE_CURRENT_SOURCE_DIR}/config/example_calibration.json")

  ament_add_gtest(test_load_plugin test/test_load_plugin.cpp)
  target_link_libraries(test_load_plugin ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle nlohmann_json)
ament_package()
