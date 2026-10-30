"""Entry point for one NEVO robot, the same for the real robot and simulation (ADR-004).

Real robot (run on the robot's Pi, with nevo.service stopped):
    ros2 launch nevo_bringup robot.launch.py namespace:=nevo1 map:=/path/map.yaml

Simulation (Gazebo world already running, from orbit_sim world.launch.py or orbit_bringup):
    ros2 launch orbit_sim world.launch.py site:=test_room
    ros2 launch nevo_bringup robot.launch.py sim:=true namespace:=nevo1 x:=-2.5 y:=0.5 \
        map:=<share orbit_sim>/sites/test_room/maps/test_room.yaml

All nodes, topics and tf live under /<namespace>. Only the hardware differs:
    real : ros2_control_node + nevo_diffdrive_arduino, rplidar_ros (A1), IMU (Arduino Nano
           + MPU6050), Kinect (kinect_ros2, camera:=true)
    sim  : spawn into Gazebo (controller_manager inside Gazebo), ros_gz_bridge
Same for both: laser filter, rf2o, EKF, twist_mux -> safety_fail_node -> diff_cont, AMCL + Nav2.

Start order uses TimerAction, not event handlers: actions fired by an event handler run
outside PushRosNamespace, so their nodes lose the namespace.
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, SetLaunchConfiguration, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetRemap
import yaml

# Nav2 params whose value is a topic name. Costmap layers resolve a relative topic inside
# the costmap's own namespace (/<ns>/local_costmap/scan_filtered: nobody publishes there),
# so these are made absolute under the robot namespace instead.
NAV_TOPIC_KEYS = ('topic', 'scan_topic', 'odom_topic')


def nav2_params(context):
    """Pick the Nav2 params and BT for sim/real and put their topics under /<namespace>."""
    ns = LaunchConfiguration('namespace').perform(context).strip('/')
    sim = LaunchConfiguration('sim').perform(context) == 'true'
    pkg_share = get_package_share_directory('nevo_bringup')
    src = LaunchConfiguration('params_file').perform(context) or os.path.join(
        pkg_share, 'config', 'nav2_sim.yaml' if sim else 'nav2_real.yaml')
    # Real: NEVO's tree ends with AMCL re-localization. Sim: Nav2's stock tree.
    bt = os.path.join(pkg_share, 'config', 'navigate_to_pose_w_relocalization.xml') if not sim else \
        os.path.join(get_package_share_directory('nav2_bt_navigator'), 'behavior_trees',
                     'navigate_to_pose_w_replanning_and_recovery.xml')

    with open(src) as f:
        params = yaml.safe_load(f)
    prefix = f'/{ns}' if ns else ''

    def walk(node):
        if isinstance(node, dict):
            for key, value in node.items():
                if key in NAV_TOPIC_KEYS and isinstance(value, str):
                    node[key] = f'{prefix}/{value.lstrip("/")}'
                else:
                    walk(value)
        elif isinstance(node, list):
            for item in node:
                walk(item)

    walk(params)
    out = tempfile.NamedTemporaryFile('w', prefix=f'{ns or "nevo"}_', suffix='_nav2.yaml', delete=False)
    yaml.safe_dump(params, out, sort_keys=False)
    out.close()
    return [SetLaunchConfiguration('nav2_params_ns', out.name),
            SetLaunchConfiguration('nav_to_pose_bt', bt)]


def generate_launch_description():
    pkg_share = get_package_share_directory('nevo_bringup')
    launch_dir = os.path.join(pkg_share, 'launch')
    config = lambda name: os.path.join(pkg_share, 'config', name)

    namespace    = LaunchConfiguration('namespace')
    sim          = LaunchConfiguration('sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml     = LaunchConfiguration('map')
    nav          = LaunchConfiguration('nav')
    camera       = LaunchConfiguration('camera')
    x, y, yaw    = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('yaw')

    args = [
        DeclareLaunchArgument('namespace', default_value='',
                              description='Robot namespace, e.g. nevo1. Empty = no namespace'),
        DeclareLaunchArgument('sim', default_value='false',
                              description='true = Gazebo (gz_ros2_control), false = real robot (nevo_diffdrive_arduino)'),
        DeclareLaunchArgument('use_sim_time', default_value=sim,
                              description='Defaults to sim'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg_share, 'maps', 'abie_lab_save.yaml'),
                              description='Nav2 map (yaml). Site maps come from orbit_sim'),
        DeclareLaunchArgument('nav', default_value='true',
                              description='Run AMCL + Nav2'),
        DeclareLaunchArgument('params_file', default_value='',
                              description='Nav2 params. Empty = config/nav2_sim.yaml (sim) or nav2_real.yaml'),
        DeclareLaunchArgument('camera', default_value=PythonExpression(["'false' if '", sim, "' == 'true' else 'true'"]),
                              description='Kinect (real) / depth camera (sim). Default: on for real, off for sim'),
        DeclareLaunchArgument('x', default_value='0.0', description='Start pose: sim spawn position and AMCL initial pose'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=nav2_params),
    ]
    sim_time = {'use_sim_time': use_sim_time}
    real_camera = IfCondition(PythonExpression(["'", sim, "' != 'true' and '", camera, "' == 'true'"]))
    sim_camera = IfCondition(PythonExpression(["'", sim, "' == 'true' and '", camera, "' == 'true'"]))

    # =========================================================================
    # Robot description (namespace and camera go to xacro for the Gazebo tags)
    # =========================================================================
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rsp.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'sim_mode': sim,
                          'namespace': namespace, 'camera': camera}.items())

    # =========================================================================
    # Hardware: sim
    # =========================================================================
    # The Gazebo entity name must be unique per robot: the namespace, or 'nevo' if empty
    entity_name = PythonExpression(["'", namespace, "' or 'nevo'"])
    spawn_robot = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', entity_name,
                   '-x', x, '-y', y, '-z', '0.02', '-Y', yaw],
        condition=IfCondition(sim))
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': config('bridge_gazebo.yaml'), 'use_sim_time': use_sim_time,
                     'expand_gz_topic_names': True}],
        condition=IfCondition(sim))
    camera_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='camera_bridge', output='screen',
        parameters=[{'config_file': config('bridge_gazebo_camera.yaml'), 'use_sim_time': use_sim_time,
                     'expand_gz_topic_names': True}],
        condition=sim_camera)

    # =========================================================================
    # Hardware: real robot
    # =========================================================================
    # TIFA's vendored rplidar_ros (same 2.1.4); NEVO has an A1 on the ttyLIDAR udev link
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')),
        launch_arguments={'serial_port': '/dev/ttyLIDAR', 'frame_id': 'laser_frame'}.items(),
        condition=UnlessCondition(sim))
    # Jazzy: controller_manager reads the URDF from the robot_description topic (from rsp).
    # 5 s delay so the Arduino serial is ready before the hardware interface is initialised.
    controller_manager = TimerAction(period=5.0, actions=[Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[config('robot_controllers.yaml')],
        condition=UnlessCondition(sim))])
    # Arduino Nano + MPU6050. Calibration from the NEVO repo (bag imu_cal_1027, 3+3 turns vs
    # lidar ICP); gyro_cov z 1e-5 lets the EKF take yaw rate from the IMU, not the wheels.
    imu = Node(
        package='nevo_bringup', executable='imu_publisher.py', name='imu_publisher', output='log',
        parameters=[{'serial_port': '/dev/serial/by-id/usb-1a86_USB2.0-Ser_-if00-port0',
                     'tilt_correction': 'fixed',
                     'mount_up': [0.0962, -0.0458, 0.9943],
                     'gyro_scale_z': 1.0023,
                     'gyro_cov': [1e-4, 1e-4, 1e-5]}],
        condition=UnlessCondition(sim))
    # Kinect -> points (depth_image_proc in the same process) for the STVL layer in nav2_real
    kinect = TimerAction(period=5.0, actions=[
        Node(package='kinect_ros2', executable='kinect_ros2_node', output='log',
             parameters=[{'full_depth_every_n': 3}], condition=real_camera),
        Node(package='tf2_ros', executable='static_transform_publisher', name='kinect_tf_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'camera_link_optical', 'kinect_depth'],
             condition=real_camera),
    ])

    # Same for both. The spawner waits for controller_manager (real: node above, sim: inside
    # Gazebo) and then activates both controllers in order.
    controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', 'diff_cont', '--controller-manager-timeout', '60',
                   '--service-call-timeout', '30'])

    # =========================================================================
    # Sensor and velocity chain (identical real and sim)
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
            # lower = more weight in the EKF: vyaw below the wheels' 0.001 so lidar wins on
            # rotation, vx equal to the wheels (rf2o vx is weaker in long corridors)
            'twist_cov_vx': 0.001, 'twist_cov_vyaw': 0.0005,
        }, sim_time])
    # Started 5 s after the rest: in sim the EKF output went NaN at startup in 4 of 9 runs
    # (random robot, clean wheel/rf2o input), never once the clock and odometry were flowing.
    ekf = TimerAction(period=5.0, actions=[Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='log', parameters=[config('ekf.yaml'), sim_time])])
    twist_mux = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux', output='log',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[config('twist_mux.yaml'), sim_time],
        remappings=[('cmd_vel_out', 'diff_cont/cmd_vel_unstamped_raw')])  # Twist -> safety_fail_node
    # 500 ms freshness watchdog; publishes TwistStamped to diff_cont/cmd_vel
    safety_fail = Node(
        package='nevo_bringup', executable='safety_fail_node', name='safety_fail_node',
        output='log', parameters=[sim_time])

    # =========================================================================
    # Localization + Nav2. AMCL is activated (and given the x/y/yaw initial pose) by the
    # lifecycle script in localization_launch.py.
    # =========================================================================
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization_launch.py')),
        launch_arguments={'namespace': namespace, 'map': map_yaml, 'use_sim_time': use_sim_time,
                          'params_file': LaunchConfiguration('nav2_params_ns'),
                          'initial_pose_x': x, 'initial_pose_y': y, 'initial_pose_yaw': yaw}.items())
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
        launch_arguments={'namespace': namespace, 'use_sim_time': use_sim_time,
                          'map_subscribe_transient_local': 'true',
                          'params_file': LaunchConfiguration('nav2_params_ns'),
                          'nav_to_pose_bt': LaunchConfiguration('nav_to_pose_bt')}.items())
    nav_stack = GroupAction(condition=IfCondition(nav), actions=[
        TimerAction(period=20.0, actions=[localization]),
        TimerAction(period=25.0, actions=[navigation]),
    ])

    robot = GroupAction([
        PushRosNamespace(namespace),
        # tf per robot (/<ns>/tf), frames stay base_link, odom, map: the Nav2 multi-robot pattern
        SetRemap('/tf', 'tf'), SetRemap('/tf_static', 'tf_static'),
        rsp,
        spawn_robot, bridge, camera_bridge,
        rplidar, controller_manager, imu, kinect,
        controller_spawner,
        laser_filter, rf2o, ekf, twist_mux, safety_fail,
        nav_stack,
    ])

    return LaunchDescription(args + [robot])
