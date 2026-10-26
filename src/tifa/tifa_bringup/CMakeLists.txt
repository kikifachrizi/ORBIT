cmake_minimum_required(VERSION 3.8)
project(tifa_bringup)

# ===============================
# Compiler options
# ===============================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===============================
# Dependencies
# ===============================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# ===============================
# Include directories
# ===============================
include_directories(include)

# ===============================
# Safety fail node
# ===============================
add_executable(safety_fail_node
  src/safety_fail_node.cpp
)
ament_target_dependencies(safety_fail_node
  rclcpp
  geometry_msgs
)

install(
  TARGETS safety_fail_node
  DESTINATION lib/${PROJECT_NAME}
)

# ===============================
# publish_initial_pose node
# ===============================
add_executable(publish_initial_pose
  src/publish_initial_pose.cpp
)

ament_target_dependencies(publish_initial_pose
  rclcpp
  geometry_msgs
  lifecycle_msgs
)

install(
  TARGETS publish_initial_pose
  DESTINATION lib/${PROJECT_NAME}
)

# ===============================
# Install other resources
# ===============================
install(
  DIRECTORY
    config
    description
    launch
    meshes
    maps
    worlds
  DESTINATION share/${PROJECT_NAME}
)

# ===============================
# Testing
# ===============================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
