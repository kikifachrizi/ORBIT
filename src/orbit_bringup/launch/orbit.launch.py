"""ORBIT untuk satu site (ADR-004): inti RMF + satu orbit_adapter per fleet,
dan pada mode:=sim juga world Gazebo + robot.launch.py tiap robot.

    ros2 launch orbit_bringup orbit.launch.py site:=test_room mode:=sim
    ros2 launch orbit_bringup orbit.launch.py site:=test_room mode:=real   # robot fisik sudah jalan

Isi site dibaca dari config/sites/<site>/site.yaml. Data site lain dari orbit_sim/sites/<site>/:
<site>.building.yaml, nav_graphs/0.yaml, maps/<site>.yaml (peta Nav2 robot sim).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _launch_site(context):
    site = LaunchConfiguration('site').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('sim', 'real'):
        raise RuntimeError(f"mode harus 'sim' atau 'real', bukan {mode!r}")
    sim = mode == 'sim'
    sim_str = 'true' if sim else 'false'

    config_dir = os.path.join(get_package_share_directory('orbit_bringup'), 'config', 'sites', site)
    site_dir = os.path.join(get_package_share_directory('orbit_sim'), 'sites', site)
    with open(os.path.join(config_dir, 'site.yaml')) as f:
        site_cfg = yaml.safe_load(f)
    building = os.path.join(site_dir, f'{site}.building.yaml')
    nav_graph = os.path.join(site_dir, 'nav_graphs', '0.yaml')
    with open(nav_graph) as f:
        map_name = next(iter(yaml.safe_load(f)['levels']))

    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('orbit_bringup'), 'launch', 'rmf_core.launch.py')),
        launch_arguments={'building': building, 'map_name': map_name, 'use_sim_time': sim_str,
                          'rviz': LaunchConfiguration('rviz')}.items())]

    for fleet, fleet_cfg in site_cfg['fleets'].items():
        actions.append(Node(
            package='orbit_adapter', executable='fleet_adapter', name=f'{fleet}_orbit_adapter',
            output='both',
            arguments=['-c', os.path.join(config_dir, fleet_cfg['config']), '-n', nav_graph,
                       *(['--sim'] if sim else [])],
            parameters=[{'use_sim_time': sim}]))

    if sim:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('orbit_sim'), 'launch', 'world.launch.py')),
            launch_arguments={'site': site, 'gz_args': LaunchConfiguration('gz_args')}.items()))
        nav2_map = os.path.join(site_dir, 'maps', f'{site}.yaml')
        stagger = float(LaunchConfiguration('robot_stagger').perform(context))
        # Robot dinyalakan bergiliran supaya Nav2 tiap robot tidak start bersamaan (beban CPU)
        for i, (name, robot) in enumerate(site_cfg['sim']['robots'].items()):
            robot_launch = os.path.join(
                get_package_share_directory(robot['bringup']), 'launch', 'robot.launch.py')
            actions.append(TimerAction(period=5.0 + i * stagger, actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(robot_launch),
                    launch_arguments={'namespace': name, 'sim': 'true', 'nav': 'true',
                                      'map': nav2_map, 'x': str(robot['x']), 'y': str(robot['y']),
                                      'yaw': str(robot.get('yaw', 0.0))}.items())]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('site', default_value='test_room',
                              description='Nama site di config/sites/'),
        DeclareLaunchArgument('mode', default_value='sim', description='sim | real'),
        DeclareLaunchArgument('rviz', default_value='true', description='Buka RViz RMF'),
        DeclareLaunchArgument('gz_args', default_value='-r -v 1',
                              description="Argumen gz sim, mis. '-r -s --headless-rendering -v 1'"),
        DeclareLaunchArgument('robot_stagger', default_value='25.0',
                              description='Jeda (s) antar robot sim dinyalakan. Nav2 robot sebelumnya '
                                          'harus aktif dulu: startup bersamaan bisa membuat respons '
                                          'lifecycle Nav2 hilang dan lokalisasi menggantung'),
        OpaqueFunction(function=_launch_site),
    ])
