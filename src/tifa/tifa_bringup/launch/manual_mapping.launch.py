import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    package_name    = 'tifa_bringup'
    slam_toolbox_pkg = 'slam_toolbox'
    lidar_pkg       = 'rplidar_ros'

    robot_id       = LaunchConfiguration('robot_id')
    ui_id          = LaunchConfiguration('ui_id')
    ws_uri_primary = LaunchConfiguration('ws_uri_primary')

    declare_robot_id = DeclareLaunchArgument('robot_id',       default_value='TFRB1')
    declare_ui_id    = DeclareLaunchArgument('ui_id',          default_value='TFUI1')
    declare_ws_uri   = DeclareLaunchArgument('ws_uri_primary', default_value='wss://tifa-ws.forgixrobotic.com')

    # =========================================================================
    # PATHS
    # =========================================================================
    pkg_share           = get_package_share_directory(package_name)
    controller_params   = os.path.join(pkg_share, 'config', 'robot_controllers.yaml')
    ekf_config          = os.path.join(pkg_share, 'config', 'ekf.yaml')
    lidar_filter_config = os.path.join(pkg_share, 'config', 'lidar_filter.yaml')
    twist_mux_config    = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    # Fix: pakai absolute path via get_package_share_directory, bukan relative path
    slam_params_file    = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')


    # =========================================================================
    # LAYER 0 — langsung jalan saat launch
    # =========================================================================

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    rplidar_activate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(lidar_pkg), 'launch', 'rplidar_c1_launch.py')
        ),
    )

    ws_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tifa_ws_bridge'), 'launch', 'ws_bridge.launch.py')
        ),
        launch_arguments={
            'ws_uri_primary': ws_uri_primary,
            'robot_id': robot_id,
            'network_mode': 'auto'
        }.items()
    )

    diagnostic_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tifa_diagnostic_node'), 'launch', 'diagnostic.launch.py')
        ),
        launch_arguments={
            'mode': 'mapping',
            'robot_id': 'TFRB1',
            'grace_period_sec': '10',
            'global_timeout_sec': '60',
        }.items()
    )

    # =========================================================================
    # LAYER 1 — laser/rf2o langsung, controller_manager dengan 3s buffer
    # =========================================================================

    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[lidar_filter_config, {'use_sim_time': False}],
        remappings=[('scan', '/scan'), ('scan_filtered', '/scan_filtered')]
    )

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='log',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[{
            'laser_scan_topic':     '/scan_filtered',
            'odom_topic':           '/odom_rf2o',
            'publish_tf':           False,
            'base_frame_id':        'base_link',
            'odom_frame_id':        'odom',
            'init_pose_from_topic': '',
            'freq':                 10.0
        }],
    )

    # Jazzy: controller_manager membaca URDF dari topic /robot_description (dari rsp),
    # parameter robot_description tidak dipakai lagi
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_params]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    # =========================================================================
    # LAYER 2 — OnProcessStart(controller_manager): semua controller + ekf + twist_mux
    # =========================================================================

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont']
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='log',
        parameters=[ekf_config]
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='log',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[twist_mux_config],
        remappings=[('cmd_vel_out', '/diff_cont/cmd_vel_raw')]  # TwistStamped → safety_fail_node
    )

    layer2_on_controller_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[
                diff_drive_spawner,
                joint_state_broadcaster_spawner,
                ekf_node,
                twist_mux,
            ]
        )
    )

    # =========================================================================
    # LAYER 3 — OnProcessExit(diff_drive_spawner): SLAM + safety_fail
    # SLAM toolbox harus setelah diff_cont aktif karena butuh TF odom->base_link
    # yang di-publish EKF (yang sudah jalan di layer 2).
    # safety_fail butuh /diff_cont/cmd_vel_raw yang aktif setelah spawner.
    # =========================================================================

    slam_activate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(slam_toolbox_pkg), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,  # absolute path, bukan relative
            'use_sim_time': 'false'
        }.items()
    )

    safety_fail_node = Node(
        package='tifa_bringup',
        executable='safety_fail_node',
        name='safety_fail_node',
        output='log'
    )

    layer3_on_diff_cont_done = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_spawner,
            on_exit=[
                slam_activate,
                safety_fail_node,
            ]
        )
    )

    # =========================================================================
    # LAYER 4 — robot_api, sensor_manager, slam_supervisor, position_sensor
    # Jalan setelah diff_cont done + 8s buffer untuk SLAM toolbox init.
    # slam_supervisor dan position_sensor butuh SLAM topic aktif.
    # robot_api dan sensor_manager tidak butuh SLAM, tapi kita grup bareng
    # supaya semua jalan setelah sistem stabil.
    # =========================================================================

    robot_api = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tifa_robot_api'), 'launch', 'robot_api.launch.py')
        ),
        launch_arguments={
            'robot_id': robot_id,
            'ui_id': ui_id
        }.items()
    )

    sensor_manager = Node(
        package='tifa_robot_api',
        executable='sensor_manager_node',
        name='sensor_manager_node'
    )

    slam_supervisor = Node(
        package='tifa_robot_api',
        executable='slam_supervisor_node',
        name='slam_supervisor_node'
    )

    position_sensor_node = Node(
        package='tifa_robot_api',
        executable='position_sensor_node',
        name='position_sensor_node'
    )

    layer4_after_slam_init = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_spawner,
            on_exit=[
                TimerAction(period=8.0, actions=[
                    robot_api,
                    sensor_manager,
                    slam_supervisor,
                    position_sensor_node,
                ])
            ]
        )
    )

    # =========================================================================
    # RETURN
    # =========================================================================
    return LaunchDescription([
        # Args
        declare_robot_id,
        declare_ui_id,
        declare_ws_uri,

        # Layer 0 — immediate
        rsp,
        rplidar_activate,
        ws_bridge,
        diagnostic_node,

        # Layer 1 — laser/rf2o langsung, controller_manager 3s timer
        laser_filter_node,
        rf2o_node,
        delayed_controller_manager,

        # Layer 2 — OnProcessStart(controller_manager)
        layer2_on_controller_start,

        # Layer 3 — OnProcessExit(diff_drive_spawner)
        layer3_on_diff_cont_done,

        # Layer 4 — OnProcessExit(diff_drive_spawner) + 8s timer
        layer4_after_slam_init,
    ])