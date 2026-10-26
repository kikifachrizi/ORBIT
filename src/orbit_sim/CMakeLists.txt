cmake_minimum_required(VERSION 3.8)
project(orbit_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    sites
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
