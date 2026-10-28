"""World satu site di Gazebo Harmonic + bridge /clock. Tidak menyalakan robot.

    ros2 launch orbit_sim world.launch.py                 # site test_room
    ros2 launch orbit_sim world.launch.py site:=telu_coffee   # atau site:=gedung_P
    ros2 launch orbit_sim world.launch.py site:=test_room gz_args:='-r -s --headless-rendering -v 1'

Robot di-spawn terpisah ke world yang sudah jalan, satu kali per robot:
    ros2 launch tifa_bringup robot.launch.py sim:=true namespace:=tifa1 x:=-2.5 y:=0.5

/clock di-bridge sekali di sini, bukan per robot: semua node use_sim_time membaca /clock global,
dan robot fisik tidak boleh punya bridge clock. Nanti dipanggil oleh orbit_bringup/sim.launch.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('orbit_sim')
    site = LaunchConfiguration('site')

    args = [
        DeclareLaunchArgument('site', default_value='test_room',
                              description='Nama folder di sites/'),
        DeclareLaunchArgument('world',
                              default_value=PathJoinSubstitution([pkg_share, 'sites', site, [site, '.sdf']]),
                              description='Override path world SDF'),
        DeclareLaunchArgument('gz_args', default_value='-r -v 1',
                              description="Argumen gz sim, mis. '-r -s --headless-rendering -v 1' untuk tanpa GUI"),
    ]

    # model:// dicari di models/ (perabot, dipakai bersama semua site) dan sites/<site>/models/
    # (bangunan site). Path mesh robot (package://tifa_bringup/...) dicari ros_gz_sim lewat
    # export <gazebo_ros gazebo_model_path> di package.xml tiap bringup robot.
    # Ditaruh di depan path lain supaya folder bernama sama di tempat lain tidak menimpanya.
    resource_paths = [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', [
        PathJoinSubstitution([pkg_share, 'sites', site, 'models']), os.pathsep,
        os.path.join(pkg_share, 'models'), os.pathsep,
        EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
    ])]
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [LaunchConfiguration('gz_args'), ' ', LaunchConfiguration('world')],
                          'on_exit_shutdown': 'true'}.items())
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    return LaunchDescription(args + resource_paths + [gz_sim, clock_bridge])
