cmake_minimum_required(VERSION 3.8)
project(nevo_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# cmd_vel freshness watchdog: twist_mux output -> diff_cont (TwistStamped)
add_executable(safety_fail_node src/safety_fail_node.cpp)
ament_target_dependencies(safety_fail_node rclcpp geometry_msgs)
install(TARGETS safety_fail_node DESTINATION lib/${PROJECT_NAME})

# Arduino Nano + MPU6050 IMU driver
install(PROGRAMS scripts/imu_publisher.py DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config description launch meshes maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
