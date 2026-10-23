import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("tifa_bringup")

    filter_config_path = PathJoinSubstitution([
        pkg_dir, "config", "lidar_filter.yaml"
    ])

    return LaunchDescription([
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[
                filter_config_path, 
                {'use_sim_time': False}  
            ],
            remappings=[
                ('scan', '/scan'),
                ('scan_filtered', '/scan_filtered')
            ]
        )
    ])