"""Simulasi TIFA di Gazebo Harmonic dengan rantai cmd_vel yang SAMA dengan robot fisik.

Rantai kecepatan (identik dengan launch_robot.launch.py):

    teleop (TwistStamped) -> /diff_cont ----------------------+
    Nav2 -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_smoothed --+
                                     twist_mux -> /diff_cont/cmd_vel_raw
                                     -> safety_fail_node -> /diff_cont/cmd_vel
                                     -> diff_cont (gz_ros2_control di Gazebo)

Yang berbeda dari robot fisik hanya sumbernya:
    hardware : gz_ros2_control (sim_mode:=true)  vs diffdrive_arduino
    lidar    : sensor gpu_lidar + ros_gz_bridge    vs rplidar_ros
    clock    : /clock dari Gazebo (use_sim_time)   vs jam sistem

Teleop keyboard:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \\
        -p stamped:=true -p frame_id:=base_link -r cmd_vel:=/diff_cont

Dengan namespace (mis. namespace:=tifa1) semua topic robot pindah ke /tifa1/...,
termasuk sensor Gazebo dan controller_manager. Teleop: -r cmd_vel:=/tifa1/diff_cont
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def generate_launch_description():
    package_name = 'tifa_bringup'
    pkg_share = get_package_share_directory(package_name)

    use_sim_time = LaunchConfiguration('use_sim_time')
    world        = LaunchConfiguration('world')
    gz_args      = LaunchConfiguration('gz_args')
    namespace    = LaunchConfiguration('namespace')

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Pakai clock Gazebo (/clock)')
    declare_world = DeclareLaunchArgument(
        'world', default_value=os.path.join(pkg_share, 'worlds', 'test_room.sdf'),
        description='Path file world SDF. Default: ruang uji robot. World site ada di orbit_sim')
    declare_gz_args = DeclareLaunchArgument(
        'gz_args', default_value='-r -v 1',
        description="Argumen gz sim, mis. '-r -s --headless-rendering -v 1' untuk tanpa GUI")
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Namespace robot, mis. tifa1. Kosong = tanpa namespace')
    declare_x   = DeclareLaunchArgument('x',   default_value='0.0')
    declare_y   = DeclareLaunchArgument('y',   default_value='0.0')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')

    ekf_config          = os.path.join(pkg_share, 'config', 'ekf.yaml')
    lidar_filter_config = os.path.join(pkg_share, 'config', 'lidar_filter.yaml')
    twist_mux_config    = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    bridge_config       = os.path.join(pkg_share, 'config', 'bridge_gazebo.yaml')

    # Gazebo mencari mesh package://tifa_bringup/... di <share>/tifa_bringup/...
    gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_share, '..'))

    # =========================================================================
    # World: Gazebo + /clock (sekali per world, di luar namespace robot)
    # =========================================================================
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [gz_args, ' ', world], 'on_exit_shutdown': 'true'}.items()
    )
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    # =========================================================================
    # Robot (semua di bawah namespace)
    # =========================================================================
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'sim_mode': 'true',
                          'namespace': namespace}.items()
    )
    # Nama entity di Gazebo harus unik per robot: pakai namespace, atau 'tifa' bila kosong
    entity_name = PythonExpression(["'", namespace, "' or 'tifa'"])
    spawn_robot = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', entity_name,
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.05', '-Y', LaunchConfiguration('yaw')],
    )
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': bridge_config, 'use_sim_time': use_sim_time,
                     'expand_gz_topic_names': True}],
    )

    # controller_manager berjalan di dalam Gazebo (gz_ros2_control) setelah robot di-spawn.
    # Satu spawner untuk kedua controller (diaktifkan berurutan) dan langsung di dalam
    # group: aksi yang dipicu event handler berjalan di luar PushRosNamespace,
    # sehingga spawner akan mencari /controller_manager, bukan /<ns>/controller_manager.
    controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', 'diff_cont', '--controller-manager-timeout', '60'],
    )

    # =========================================================================
    # Rantai sensor & kecepatan — sama dengan launch_robot.launch.py
    # =========================================================================
    laser_filter_node = Node(
        package='laser_filters', executable='scan_to_scan_filter_chain',
        parameters=[lidar_filter_config, {'use_sim_time': use_sim_time}],
    )
    rf2o_node = Node(
        package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry', output='log',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[{
            'laser_scan_topic': 'scan_filtered', 'odom_topic': 'odom_rf2o',
            'publish_tf': False, 'base_frame_id': 'base_link',
            'odom_frame_id': 'odom', 'init_pose_from_topic': '', 'freq': 10.0,
            'use_sim_time': use_sim_time,
        }],
    )
    ekf_node = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='log',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}]
    )
    twist_mux = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux', output='log',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[twist_mux_config, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_out', 'diff_cont/cmd_vel_raw')]
    )
    safety_fail_node = Node(
        package='tifa_bringup', executable='safety_fail_node', name='safety_fail_node', output='log',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # tf tetap per robot (/tifa1/tf), pola multi-robot Nav2
    robot = GroupAction([
        PushRosNamespace(namespace),
        SetRemap('/tf', 'tf'), SetRemap('/tf_static', 'tf_static'),
        rsp, bridge, spawn_robot, controller_spawner,
        laser_filter_node, rf2o_node, ekf_node, twist_mux, safety_fail_node,
    ])

    return LaunchDescription([
        declare_sim_time, declare_world, declare_gz_args, declare_namespace,
        declare_x, declare_y, declare_yaw,
        gz_resource_path,

        gz_sim, clock_bridge,
        robot,
    ])
    