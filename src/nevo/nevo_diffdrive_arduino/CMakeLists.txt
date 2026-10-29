cmake_minimum_required(VERSION 3.16)
project(nevo_diffdrive_arduino)

# Default to C++17 (required by ROS 2 Jazzy)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

# Library name differs from TIFA's diffdrive_arduino so both can be installed side by side
add_library(nevo_diffdrive_arduino SHARED src/diffdrive_arduino.cpp src/wheel.cpp src/arduino_comms.cpp)

target_include_directories(
  nevo_diffdrive_arduino
  PRIVATE
  include
)
ament_target_dependencies(
  nevo_diffdrive_arduino
  hardware_interface
  controller_manager
  serial
  rclcpp
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)

install(
  TARGETS nevo_diffdrive_arduino
  DESTINATION lib
)

ament_export_libraries(
  nevo_diffdrive_arduino
)
ament_export_dependencies(
  hardware_interface
  controller_manager
  serial
  rclcpp
  pluginlib
)

ament_package()
