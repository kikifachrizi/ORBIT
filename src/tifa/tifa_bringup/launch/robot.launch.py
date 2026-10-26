"""Pintu masuk satu robot TIFA, sama untuk robot fisik dan simulasi (ADR-004).

Robot fisik (dijalankan di komputer robot):
    ros2 launch tifa_bringup robot.launch.py namespace:=tifa1 map:=/path/peta.yaml

Simulasi (world Gazebo sudah jalan, dari orbit_sim world.launch.py atau orbit_bringup):
    ros2 launch orbit_sim world.launch.py site:=test_room
    ros2 launch tifa_bringup robot.launch.py sim:=true namespace:=tifa1 x:=-2.5 y:=0.5 \
        map:=<share orbit_sim>/sites/test_room/maps/test_room.yaml

Semua node, topic, dan tf berada di bawah /<namespace>. Satu-satunya yang berbeda
antara fisik dan sim adalah hardware:
    fisik : ros2_control_node + diffdrive_arduino, rplidar_ros
    sim   : spawn ke Gazebo (controller_manager di dalam Gazebo), ros_gz_bridge

Urutan nyala memakai TimerAction, bukan event handler: aksi yang dipicu event handler
berjalan di luar PushRosNamespace sehingga node-nya kehilangan namespace.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def generate_launch_description():
    pkg_share = get_package_share_directory('tifa_bringup')
    launch_dir = os.path.join(pkg_share, 'launch')
    config = lambda name: os.path.join(pkg_share, 'config', name)

    namespace    = LaunchConfiguration('namespace')
    sim          = LaunchConfiguration('sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml     = LaunchConfiguration('map')
    nav          = LaunchConfiguration('nav')
    params_file  = LaunchConfiguration('params_file')
    x, y, yaw    = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('yaw')

    args = [
        DeclareLaunchArgument('namespace', default_value='',
                              description='Namespace robot, mis. tifa1. Kosong = tanpa namespace'),
        DeclareLaunchArgument('sim', default_value='false',
                              description='true = Gazebo (gz_ros2_control), false = robot fisik (diffdrive_arduino)'),
        DeclareLaunchArgument('use_sim_time', default_value=sim,
                              description='Default mengikuti sim'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg_share, 'maps', 'abie_022526.yaml'),
                              description='Peta Nav2 (yaml). Peta site nanti dari orbit_sim'),
        DeclareLaunchArgument('nav', default_value='true',
                              description='Jalankan AMCL + Nav2'),
        DeclareLaunchArgument('params_file', default_value=config('DWB_params.yaml'),
                              description='Parameter Nav2'),
        DeclareLaunchArgument('x', default_value='0.0', description='Pose awal: posisi spawn sim dan initial pose AMCL'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]
    sim_time = {'use_sim_time': use_sim_time}

    # =========================================================================
    # Deskripsi robot (namespace diteruskan ke xacro untuk topic sensor Gazebo)
    # =========================================================================
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rsp.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'sim_mode': sim,
                          'namespace': namespace}.items())

    # =========================================================================
    # Hardware: sim
    # =========================================================================
    # Nama entity di Gazebo harus unik per robot: pakai namespace, atau 'tifa' bila kosong
    entity_name = PythonExpression(["'", namespace, "' or 'tifa'"])
    spawn_robot = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', entity_name,
                   '-x', x, '-y', y, '-z', '0.05', '-Y', yaw],
        condition=IfCondition(sim))
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': config('bridge_gazebo.yaml'), 'use_sim_time': use_sim_time,
                     'expand_gz_topic_names': True}],
        condition=IfCondition(sim))

    # =========================================================================
    # Hardware: robot fisik
    # =========================================================================
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_c1_launch.py')),
        condition=UnlessCondition(sim))
    # Jazzy: controller_manager membaca URDF dari topic robot_description (dari rsp).
    # Jeda 5 s supaya serial Arduino siap sebelum hardware interface di-init.
    controller_manager = TimerAction(period=5.0, actions=[Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[config('robot_controllers.yaml')],
        condition=UnlessCondition(sim))])

    # Sama untuk keduanya. Spawner menunggu controller_manager (fisik: node di atas,
    # sim: di dalam Gazebo) lalu mengaktifkan kedua controller berurutan.
    controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', 'diff_cont', '--controller-manager-timeout', '60'])

    # =========================================================================
    # Rantai sensor dan kecepatan (identik fisik dan sim)
    # =========================================================================
    laser_filter = Node(
        package='laser_filters', executable='scan_to_scan_filter_chain',
        parameters=[config('lidar_filter.yaml'), sim_time])
    rf2o = Node(
        package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry', output='log',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[{
            'laser_scan_topic': 'scan_filtered', 'odom_topic': 'odom_rf2o',
            'publish_tf': False, 'base_frame_id': 'base_link',
            'odom_frame_id': 'odom', 'init_pose_from_topic': '', 'freq': 10.0,
        }, sim_time])
    ekf = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='log', parameters=[config('ekf.yaml'), sim_time])
    twist_mux = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux', output='log',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[config('twist_mux.yaml'), sim_time],
        remappings=[('cmd_vel_out', 'diff_cont/cmd_vel_raw')])  # TwistStamped -> safety_fail_node
    safety_fail = Node(
        package='tifa_bringup', executable='safety_fail_node', name='safety_fail_node',
        output='log', parameters=[sim_time])

    # =========================================================================
    # Lokalisasi + Nav2 (jeda sama dengan launch_robot lama: 5 s + 12/17/20 s)
    # =========================================================================
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization_launch.py')),
        launch_arguments={'namespace': namespace, 'map': map_yaml, 'use_sim_time': use_sim_time,
                          'params_file': params_file}.items())
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
        launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time,
                          'params_file': params_file}.items())
    initial_pose = Node(
        package='tifa_bringup', executable='publish_initial_pose', name='initial_pose_pub',
        output='log', parameters=[sim_time], arguments=[x, y, yaw])
    nav_stack = GroupAction(condition=IfCondition(nav), actions=[
        TimerAction(period=17.0, actions=[localization]),
        TimerAction(period=22.0, actions=[navigation]),
        TimerAction(period=25.0, actions=[initial_pose]),
    ])

    robot = GroupAction([
        PushRosNamespace(namespace),
        # tf per robot (/<ns>/tf), frame tetap base_link, odom, map: pola multi-robot Nav2
        SetRemap('/tf', 'tf'), SetRemap('/tf_static', 'tf_static'),
        rsp,
        spawn_robot, bridge,
        rplidar, controller_manager,
        controller_spawner,
        laser_filter, rf2o, ekf, twist_mux, safety_fail,
        nav_stack,
    ])

    return LaunchDescription(args + [robot])
