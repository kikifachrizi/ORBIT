"""Fleet adapter ORBIT: satu proses per fleet, satu backend per robot (ADR-002).

    ros2 run orbit_adapter fleet_adapter -c fleet_tifa.yaml -n nav_graphs/0.yaml [--sim]

Config fleet mengikuti format EasyFullControl (rmf_fleet: limits, profile, robots, ...).
Key tambahan ORBIT per robot di rmf_fleet.robots.<nama>:
    backend      nav2 (default)
    namespace    namespace ROS robot (default: nama robot)
    map_frame, base_frame, battery_topic, pose_timeout   opsional, lihat backends/nav2.py
"""
import argparse
import sys
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
import rmf_adapter as adpt
import rmf_adapter.easy_full_control as rmf_easy
import yaml

from .backends import make_backend
from .robot_adapter import RobotAdapter


def main(argv=sys.argv):
    adpt.init_rclcpp()
    rclpy.init(args=argv)

    parser = argparse.ArgumentParser(prog='fleet_adapter')
    parser.add_argument('-c', '--config_file', required=True, help='Config fleet (yaml)')
    parser.add_argument('-n', '--nav_graph', required=True, help='Nav graph RMF (yaml)')
    parser.add_argument('-s', '--server_uri', default='', help='URI websocket API server (opsional)')
    parser.add_argument('--sim', action='store_true', help='Pakai sim time')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(argv)[1:])

    with open(args.config_file) as f:
        config = yaml.safe_load(f)
    with open(args.nav_graph) as f:
        map_name = next(iter(yaml.safe_load(f)['levels']))

    fleet_config = rmf_easy.FleetConfiguration.from_config_files(
        args.config_file, args.nav_graph, args.server_uri or None)
    if fleet_config is None:
        print(f'Config fleet tidak valid: {args.config_file}', file=sys.stderr)
        return 1
    fleet_name = fleet_config.fleet_name

    node = rclpy.create_node(
        f'{fleet_name}_orbit_adapter',
        parameter_overrides=[Parameter('use_sim_time', value=args.sim)])
    logger = node.get_logger()

    adapter = adpt.Adapter.make(f'{fleet_name}_fleet_adapter')
    if args.sim:
        adapter.node.use_sim_time()
    adapter.start()
    time.sleep(1.0)
    fleet_handle = adapter.add_easy_fleet(fleet_config)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    robots = []
    for name, robot_yaml in config['rmf_fleet']['robots'].items():
        robot_yaml = robot_yaml or {}
        robot_config = fleet_config.get_known_robot_configuration(name)
        if robot_config is None:
            robot_config = rmf_easy.RobotConfiguration([robot_yaml.get('charger', '')])
        backend = make_backend(robot_yaml.get('backend', 'nav2'), name, robot_yaml, args.sim)
        for n in backend.nodes():
            executor.add_node(n)
        robots.append(RobotAdapter(name, backend, fleet_handle, robot_config, map_name, logger))
        logger.info(f'{name}: backend {robot_yaml.get("backend", "nav2")}, menunggu pose')

    period = 1.0 / float(config['rmf_fleet'].get('robot_state_update_frequency', 2.0))

    def update_all():
        for robot in robots:
            try:
                robot.update()
            except Exception as e:  # satu robot bermasalah tidak boleh menghentikan fleet
                logger.error(f'{robot.name}: update gagal: {e}')

    node.create_timer(period, update_all)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        adapter.stop()
        executor.shutdown()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
