"""Node inti RMF untuk satu site: traffic schedule, blockade, building map, dispatcher, visualisasi.

    ros2 launch orbit_bringup rmf_core.launch.py building:=<path>.building.yaml map_name:=L1

Biasanya dipanggil oleh orbit.launch.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_time = {'use_sim_time': use_sim_time}
    headless = PythonExpression(["'false' if '", LaunchConfiguration('rviz'), "' == 'true' else 'true'"])

    args = [
        DeclareLaunchArgument('building', description='Path <site>.building.yaml'),
        DeclareLaunchArgument('map_name', default_value='L1', description='Level awal visualisasi'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true', description='Buka RViz RMF'),
    ]

    nodes = [
        Node(package='rmf_traffic_ros2', executable='rmf_traffic_schedule', output='both',
             name='rmf_traffic_schedule_primary', parameters=[sim_time]),
        Node(package='rmf_traffic_ros2', executable='rmf_traffic_blockade', output='both',
             parameters=[sim_time]),
        Node(package='rmf_building_map_tools', executable='building_map_server', output='both',
             arguments=[LaunchConfiguration('building')], parameters=[sim_time]),
        Node(package='rmf_task_ros2', executable='rmf_task_dispatcher', output='both',
             parameters=[sim_time, {'bidding_time_window': 2.0}]),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(
                get_package_share_directory('rmf_visualization'), 'visualization.launch.xml')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'map_name': LaunchConfiguration('map_name'),
                              'headless': headless}.items()),
    ]
    return LaunchDescription(args + nodes)
