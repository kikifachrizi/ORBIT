"""Open-RMF fleet adapter for NEVO (EasyFullControl → Nav2).

RMF hands out one waypoint at a time. Two ways to drive there (`lane_following`
in the config):
  stage 3a (false): a NavigateToPose goal; Nav2's planner picks the path freely.
  stage 3b (true):  the lane itself (previous waypoint → this one) is sent as a
                    path to FollowPath with the RPP controller, so NEVO stays on
                    the drawn lane and turns on the spot at waypoints.
One fleet, several robots: each robot runs in its own namespace
(orbit_nevo_bringup), so the adapter uses /<ns>/navigate_to_pose or
/<ns>/follow_path and reads TF map → base_link from /<ns>/tf.

Coordinates: traffic-editor's reference_image frame has its origin at the floor
plan's top-left pixel with y up (pixels × scale, y flipped). The robot map's
origin is its bottom-left pixel. With the drawing scale equal to the map
resolution, robot = rmf + (origin_x, origin_y + height_px × resolution).
"""

import argparse
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.time import Time
import rmf_adapter
from rmf_adapter import Adapter
import rmf_adapter.easy_full_control as rmf_easy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath, NavigateToPose
from tf2_ros import Buffer, TransformListener
import yaml


def read_pgm_height(path):
    with open(path, 'rb') as f:
        tokens = []
        while len(tokens) < 3:
            line = f.readline()
            if line.startswith(b'#'):
                continue
            tokens += line.split()
    return int(tokens[2])


def rmf_to_robot_offset(map_yaml_path):
    with open(map_yaml_path) as f:
        m = yaml.safe_load(f)
    image = os.path.join(os.path.dirname(map_yaml_path), m['image'])
    height = read_pgm_height(image)
    ox, oy = m['origin'][0], m['origin'][1]
    return ox, oy + height * m['resolution'], m['resolution']


def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NevoAdapter:
    """One NEVO robot. `node` is this robot's own node (TF remapped to /<ns>/tf)."""

    def __init__(self, node, fleet_handle, fleet_config, cfg, name, ns, offset):
        self.node = node
        self.fleet_handle = fleet_handle
        self.cfg = cfg
        self.name = name
        self.robot_config = fleet_config.get_known_robot_configuration(name)
        assert self.robot_config, f'robot {name} is not in rmf_fleet.robots of the config'
        self.dx, self.dy = offset
        self.update_handle = None
        self.execution = None
        self.goal_handle = None
        self.cmd_id = 0
        self.finished_cmd = None
        self.next_add_attempt = 0.0
        self.last_waypoint = None  # robot-frame (x, y) of the last waypoint reached = start of the next lane
        self.lane_following = cfg.get('lane_following', False)
        self.lock = threading.Lock()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.nav = ActionClient(node, NavigateToPose, f'/{ns}/navigate_to_pose')
        self.follow = ActionClient(node, FollowPath, f'/{ns}/follow_path')
        node.create_timer(cfg['update_period'], self.update)

    # ---- RMF → robot ---------------------------------------------------
    def navigate(self, destination, execution):
        with self.lock:
            self.cmd_id += 1
            cmd_id = self.cmd_id
            self.execution = execution
            self.finished_cmd = None
        x, y, yaw = destination.position
        rx, ry = x + self.dx, y + self.dy
        name = destination.name or '(unnamed)'
        self.node.get_logger().info(
            f'navigate cmd {cmd_id}: waypoint {name} rmf=({x:.2f}, {y:.2f}) '
            f'→ robot=({rx:.2f}, {ry:.2f}, {math.degrees(yaw):.0f}°)')

        if self.lane_following:
            self._follow_lane(cmd_id, rx, ry, yaw)
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.cfg['map_frame']
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = rx
        goal.pose.pose.position.y = ry
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = yaw_to_quat(yaw)

        if not self.nav.wait_for_server(timeout_sec=2.0):
            self._server_unavailable(cmd_id, self.nav)
            return
        future = self.nav.send_goal_async(goal)
        future.add_done_callback(lambda f: self._goal_response(f, cmd_id))

    def _follow_lane(self, cmd_id, rx, ry, yaw):
        """Stage 3b: drive the straight lane from the last waypoint to (rx, ry) with RPP."""
        p = self.pose()
        if p is None:
            self.node.get_logger().error(f'cmd {cmd_id}: no pose yet, cannot build the lane')
            self._request_replan()
            return
        # Start the path at the previous waypoint (the lane's real start) when the robot
        # is still near it, so RPP pulls the robot back onto the drawn line. Otherwise
        # (first command, mid-lane replan) start from where the robot is.
        sx, sy = p[0], p[1]
        if self.last_waypoint and math.hypot(p[0] - self.last_waypoint[0], p[1] - self.last_waypoint[1]) < 0.6:
            sx, sy = self.last_waypoint
        length = math.hypot(rx - sx, ry - sy)
        if length < 0.10 and math.hypot(rx - p[0], ry - p[1]) < 0.25:
            # already on this waypoint (RMF often starts a task at the robot's own waypoint)
            self.node.get_logger().info(f'cmd {cmd_id}: already at the waypoint')
            with self.lock:
                self.finished_cmd = cmd_id
                self.last_waypoint = (rx, ry)
            return

        heading = math.atan2(ry - sy, rx - sx)
        step = self.cfg.get('lane_path_resolution', 0.05)
        n = max(1, math.ceil(length / step))
        stamp = self.node.get_clock().now().to_msg()
        goal = FollowPath.Goal()
        goal.path.header.frame_id = self.cfg['map_frame']
        goal.path.header.stamp = stamp
        for i in range(n + 1):
            ps = PoseStamped()
            ps.header = goal.path.header
            ps.pose.position.x = sx + (rx - sx) * i / n
            ps.pose.position.y = sy + (ry - sy) * i / n
            # along the lane, then RMF's heading at the waypoint (RPP turns on the spot to it)
            ps.pose.orientation.z, ps.pose.orientation.w = yaw_to_quat(yaw if i == n else heading)
            goal.path.poses.append(ps)
        goal.controller_id = self.cfg.get('lane_controller', 'LaneFollow')
        goal.goal_checker_id = self.cfg.get('lane_goal_checker', 'general_goal_checker')
        goal.progress_checker_id = self.cfg.get('lane_progress_checker', '')  # '' = the server's only one

        if not self.follow.wait_for_server(timeout_sec=2.0):
            self._server_unavailable(cmd_id, self.follow)
            return
        future = self.follow.send_goal_async(goal)
        future.add_done_callback(lambda f: self._goal_response(f, cmd_id, (rx, ry)))

    def _server_unavailable(self, cmd_id, client):
        # Without a replan the command never finishes and RMF keeps the robot "busy" forever
        # (e.g. Nav2 still starting on the robot, or the network link to it dropped).
        delay = self.cfg.get('replan_delay', 5.0)
        self.node.get_logger().error(
            f'cmd {cmd_id}: {client._action_name} server not available, asking RMF to replan in {delay:.0f} s')
        self._request_replan(delay, cmd_id)

    def _goal_response(self, future, cmd_id, target=None):
        handle = future.result()
        if not handle.accepted:
            self.node.get_logger().warn(f'cmd {cmd_id}: Nav2 rejected the goal')
            self._request_replan()
            return
        with self.lock:
            if cmd_id != self.cmd_id:  # superseded by a newer navigate()
                handle.cancel_goal_async()
                return
            self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self._result(f, cmd_id, target))

    def _result(self, future, cmd_id, target=None):
        status = future.result().status
        with self.lock:
            current = cmd_id == self.cmd_id
        if not current:
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info(f'cmd {cmd_id}: arrived')
            with self.lock:
                self.finished_cmd = cmd_id
                if target is not None:
                    self.last_waypoint = target
        elif status == GoalStatus.STATUS_CANCELED:
            self.node.get_logger().info(f'cmd {cmd_id}: canceled')
        else:
            delay = self.cfg.get('replan_delay', 5.0)
            self.node.get_logger().warn(
                f'cmd {cmd_id}: Nav2 failed (status {status}), asking RMF to replan in {delay:.0f} s')
            self._request_replan(delay, cmd_id)

    def stop(self, activity):
        with self.lock:
            if self.execution is None or not self.execution.identifier.is_same(activity):
                return
            self.execution = None
            handle = self.goal_handle
            self.goal_handle = None
        self.node.get_logger().info('stop: canceling Nav2 goal')
        with self.lock:
            self.last_waypoint = None  # robot may stop mid-lane
        if handle is not None:
            handle.cancel_goal_async()

    def execute_action(self, category, description, execution):
        self.node.get_logger().warn(f'action [{category}] not supported, finishing it immediately')
        execution.finished()

    def _request_replan(self, delay=0.0, cmd_id=None):
        # After a Nav2 failure (e.g. RPP stopped at an obstacle on the lane) RMF usually sends the
        # same lane again, which fails again at once: without a pause that loops ~2x per second and
        # the robot never gets a quiet moment to drive on once the obstacle is gone.
        if self.update_handle is None:
            return
        if delay <= 0.0:
            self.update_handle.more().replan()
            return

        def later():
            with self.lock:
                stale = cmd_id is not None and cmd_id != self.cmd_id  # RMF already sent something new
            if not stale and self.update_handle is not None:
                self.update_handle.more().replan()
        threading.Timer(delay, later).start()

    # ---- robot → RMF ---------------------------------------------------
    def pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.cfg['map_frame'], self.cfg['base_frame'], Time(), timeout=Duration(seconds=0.05))
        except Exception:
            return None
        tr = t.transform
        return tr.translation.x, tr.translation.y, quat_to_yaw(tr.rotation)

    def update(self):
        p = self.pose()
        if p is None:
            return
        rx, ry, yaw = p
        state = rmf_easy.RobotState(self.cfg['rmf_map_name'], [rx - self.dx, ry - self.dy, yaw],
                                    self.cfg['battery_soc'])
        if self.update_handle is None:
            # add_robot fails if the robot is too far from the graph or its charger
            # waypoint is missing; retry every few seconds instead of giving up.
            now = time.monotonic()
            if now < self.next_add_attempt:
                return
            self.next_add_attempt = now + 5.0
            callbacks = rmf_easy.RobotCallbacks(
                lambda dest, ex: self.navigate(dest, ex),
                lambda act: self.stop(act),
                lambda cat, desc, ex: self.execute_action(cat, desc, ex))
            handle = self.fleet_handle.add_robot(self.name, state, self.robot_config, callbacks)
            if handle is None:
                self.node.get_logger().warn(
                    f'RMF did not accept {self.name} at rmf=({rx - self.dx:.2f}, {ry - self.dy:.2f}); '
                    'is it on the graph and does its charger waypoint exist? retrying in 5 s')
                return
            self.update_handle = handle
            self.node.get_logger().info(
                f'registered {self.name} with RMF at rmf=({rx - self.dx:.2f}, {ry - self.dy:.2f})')
            return

        with self.lock:
            if self.execution is not None and self.finished_cmd == self.cmd_id:
                self.execution.finished()
                self.execution = None
                self.goal_handle = None
            activity = self.execution.identifier if self.execution is not None else None
        self.update_handle.update(state, activity)


def main(argv=sys.argv):
    rclpy.init(args=argv)
    rmf_adapter.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(prog='fleet_adapter')
    parser.add_argument('-c', '--config_file', required=True)
    parser.add_argument('-n', '--nav_graph', required=True)
    parser.add_argument('-m', '--map_yaml', required=True, help='robot map yaml used as the floor plan')
    parser.add_argument('-sim', '--use_sim_time', action='store_true',
                        help='use /clock (also on when the node parameter use_sim_time is true)')
    parser.add_argument('-L', '--lane_following', choices=['config', 'true', 'false'], default='config',
                        help='stage 3b lane following (FollowPath + RPP); "config" = use nevo.lane_following')
    args = parser.parse_args(args_without_ros[1:])

    fleet_config = rmf_easy.FleetConfiguration.from_config_files(args.config_file, args.nav_graph)
    assert fleet_config, f'failed to parse {args.config_file}'
    with open(args.config_file) as f:
        cfg = yaml.safe_load(f)['nevo']
    if args.lane_following != 'config':
        cfg['lane_following'] = args.lane_following == 'true'

    node = rclpy.node.Node(f'{fleet_config.fleet_name}_command_handle')
    # Sim time from -sim or from the launch param (real robot: false, so no wait for /clock)
    use_sim_time = args.use_sim_time or node.get_parameter('use_sim_time').value
    adapter = Adapter.make(f'{fleet_config.fleet_name}_fleet_adapter')
    assert adapter, 'Unable to start the fleet adapter. Is rmf_traffic_schedule running?'
    if use_sim_time:
        node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        adapter.node.use_sim_time()
    adapter.start()

    node.declare_parameter('server_uri', '')
    server_uri = node.get_parameter('server_uri').get_parameter_value().string_value
    fleet_config.server_uri = server_uri or None
    fleet_handle = adapter.add_easy_fleet(fleet_config)

    dx, dy, res = rmf_to_robot_offset(args.map_yaml)
    node.get_logger().info(
        'driving mode: ' + ('stage 3b lane following (FollowPath + ' + cfg.get('lane_controller', 'LaneFollow') + ')'
                            if cfg.get('lane_following') else 'stage 3a NavigateToPose (Nav2 plans freely)'))
    node.get_logger().info(
        f'RMF → robot offset ({dx:.3f}, {dy:.3f}); assumes traffic-editor scale = {res} m/px')

    # One node per robot so its TF listener can be remapped to /<ns>/tf.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    robot_nodes, robots = [], []
    for name, rcfg in cfg['robots'].items():
        ns = rcfg['namespace']
        rnode = rclpy.node.Node(
            f'{fleet_config.fleet_name}_{name}_handle',
            cli_args=['--ros-args', '-r', f'/tf:=/{ns}/tf', '-r', f'/tf_static:=/{ns}/tf_static'],
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)])
        robots.append(NevoAdapter(rnode, fleet_handle, fleet_config, cfg, name, ns, (dx, dy)))
        robot_nodes.append(rnode)
        executor.add_node(rnode)
        node.get_logger().info(f'{name}: /{ns}/navigate_to_pose, TF from /{ns}/tf')

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    for n in robot_nodes + [node]:
        n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
