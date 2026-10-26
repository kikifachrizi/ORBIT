"""Kompatibilitas: perintah lama robot fisik. Sama dengan robot.launch.py sim:=false.

    ros2 launch tifa_bringup launch_robot.launch.py map:=/path/peta.yaml

Pintu masuk utama sekarang robot.launch.py (namespace, sim, map, nav). Peta tidak lagi
dibaca dari ~/tifa_selected_map.txt; berikan lewat argumen map.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('tifa_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(pkg_share, 'maps', 'abie_022526.yaml'),
                              description='Peta Nav2 (yaml)'),
        DeclareLaunchArgument('namespace', default_value='',
                              description='Namespace robot. Kosong = sama dengan perilaku lama'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'robot.launch.py')),
            launch_arguments={'sim': 'false', 'map': LaunchConfiguration('map'),
                              'namespace': LaunchConfiguration('namespace')}.items()),
    ])
