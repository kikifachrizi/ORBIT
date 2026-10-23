import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction,
    RegisterEventHandler, DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node


def get_selected_map():
    config_file = os.path.expanduser("~/tifa_selected_map.txt")
    default_map = "/home/tifav1/tifav2_ws/src/tifa_bringup/maps/abie_022526.yaml"
    if os.path.exists(config_file):
        with open(config_file, 'r') as f:
            map_path = f.read().strip()
        if map_path and os.path.exists(map_path):
            return map_path
    return default_map


def generate_launch_description():
    package_name = 'tifa_bringup'
    lidar_pkg    = 'rplidar_ros'

    robot_id       = LaunchConfiguration('robot_id')
    ui_id          = LaunchConfiguration('ui_id')
    ws_uri_primary = LaunchConfiguration('ws_uri_primary')

    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='TFRB1')
    declare_ui_id    = DeclareLaunchArgument('ui_id',    default_value='TFUI1')
    declare_ws_uri   = DeclareLaunchArgument('ws_uri_primary', default_value='wss://tifa-ws.forgixrobotic.com')
    declare_map_arg  = DeclareLaunchArgument(
        name='map',
        default_value=get_selected_map(),
        description='Full path to map yaml file'
    )
    map_file = LaunchConfiguration('map')

    pkg_share           = get_package_share_directory(package_name)
    controller_params   = os.path.join(pkg_share, 'config', 'robot_controllers.yaml')
    ekf_config          = os.path.join(pkg_share, 'config', 'ekf.yaml')
    lidar_filter_config = os.path.join(pkg_share, 'config', 'lidar_filter.yaml')
    twist_mux_config    = os.path.join(pkg_share, 'config', 'twist_mux.yaml')


    # =========================================================================
    # LAYER 0 — immediate
    # =========================================================================
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'rsp.launch.py')),
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
        launch_arguments={'ws_uri_primary': ws_uri_primary, 'robot_id': robot_id, 'network_mode': 'auto'}.items()
    )
    diagnostic_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tifa_diagnostic_node'), 'launch', 'diagnostic.launch.py')
        ),
        launch_arguments={'mode': 'navigation', 'robot_id': 'TFRB1', 'grace_period_sec': '10', 'global_timeout_sec': '120'}.items()
    )

    # =========================================================================
    # LAYER 1 — laser/rf2o immediate, controller_manager +5s (NAIK dari 3s)
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
            'laser_scan_topic': '/scan_filtered', 'odom_topic': '/odom_rf2o',
            'publish_tf': False, 'base_frame_id': 'base_link',
            'odom_frame_id': 'odom', 'init_pose_from_topic': '', 'freq': 10.0
        }],
    )
    # Jazzy: controller_manager membaca URDF dari topic /robot_description (dari rsp),
    # parameter robot_description tidak dipakai lagi
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_params]
    )

    # FIX: 3s → 5s agar hardware interface punya waktu cukup untuk init
    delayed_controller_manager = TimerAction(period=5.0, actions=[controller_manager])

    # =========================================================================
    # LAYER 2 — OnProcessStart(controller_manager)
    # =========================================================================
    diff_drive_spawner = Node(
        package='controller_manager', executable='spawner', arguments=['diff_cont']
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster']
    )
    ekf_node = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='log', parameters=[ekf_config]
    )
    twist_mux = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux', output='log',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[twist_mux_config],
        remappings=[('cmd_vel_out', '/diff_cont/cmd_vel_raw')]  # absolute path, TwistStamped → safety_fail_node
    )

    layer2_on_controller_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner, joint_state_broadcaster_spawner, ekf_node, twist_mux]
        )
    )

    # =========================================================================
    # LAYER 3 — TimerAction dari OnProcessStart(controller_manager)
    # FIX: tidak lagi menggunakan OnProcessExit(diff_drive_spawner) yang race condition
    # =========================================================================
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'localization_launch.py')),
        launch_arguments={'map': map_file}.items()
    )
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'false', 'map_subscribe_transient_local': 'true'}.items()
    )
    safety_fail_node = Node(
        package='tifa_bringup', executable='safety_fail_node', name='safety_fail_node', output='log'
    )

    layer3_timed = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[
                TimerAction(period=12.0, actions=[localization_launch]),
                TimerAction(period=17.0, actions=[navigation_launch]),
                TimerAction(period=14.0, actions=[safety_fail_node]),
            ]
        )
    )

    # =========================================================================
    # LAYER 4 — TimerAction dari OnProcessStart(controller_manager)
    # =========================================================================
    initial_pose_node = Node(
        package='tifa_bringup', executable='publish_initial_pose',
        name='initial_pose_pub', output='log',
        parameters=[{'use_sim_time': False}], arguments=['0.0', '0.0', '0.0']
    )
    robot_api = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tifa_robot_api'), 'launch', 'robot_api.launch.py')
        ),
        launch_arguments={'robot_id': robot_id, 'ui_id': ui_id}.items()
    )
    sensor_manager = Node(
        package='tifa_robot_api', executable='sensor_manager_node', name='sensor_manager_node'
    )

    layer4_timed = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[
                TimerAction(period=20.0, actions=[initial_pose_node]),
                TimerAction(period=22.0, actions=[robot_api]),
                TimerAction(period=25.0, actions=[sensor_manager]),
            ]
        )
    )

    # =========================================================================
    # RETURN
    # =========================================================================
    return LaunchDescription([
        declare_map_arg, declare_robot_id, declare_ui_id, declare_ws_uri,

        # Layer 0
        rsp, rplidar_activate, ws_bridge, diagnostic_node,

        # Layer 1
        laser_filter_node, rf2o_node, delayed_controller_manager,

        # Layer 2
        layer2_on_controller_start,

        # Layer 3 — FIX: timer-based, bukan OnProcessExit
        layer3_timed,

        # Layer 4
        layer4_timed,
    ])