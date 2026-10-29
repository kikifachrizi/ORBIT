cmake_minimum_required(VERSION 3.8)
project(orbit_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    models
    sites
  DESTINATION share/${PROJECT_NAME}
)

# Nav graph RMF di-generate dari sites/<site>/<site>.building.yaml (rmf_traffic_editor)
# ke share/orbit_sim/sites/<site>/nav_graphs/0.yaml, supaya graph selalu sesuai file sumbernya.
file(GLOB building_files "${CMAKE_CURRENT_SOURCE_DIR}/sites/*/*.building.yaml")
foreach(building_file ${building_files})
  get_filename_component(site_dir ${building_file} DIRECTORY)
  get_filename_component(site ${site_dir} NAME)
  set(nav_graph_dir ${CMAKE_CURRENT_BINARY_DIR}/sites/${site}/nav_graphs)
  add_custom_command(
    OUTPUT ${nav_graph_dir}/0.yaml
    COMMAND ros2 run rmf_building_map_tools building_map_generator nav ${building_file} ${nav_graph_dir}
    DEPENDS ${building_file})
  add_custom_target(generate_${site}_nav_graphs ALL DEPENDS ${nav_graph_dir}/0.yaml)
  install(DIRECTORY ${nav_graph_dir} DESTINATION share/${PROJECT_NAME}/sites/${site})
endforeach()

ament_package()
