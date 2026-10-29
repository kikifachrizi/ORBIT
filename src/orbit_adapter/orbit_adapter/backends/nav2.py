"""Backend nav2: robot dengan Nav2 di bawah /<namespace> (TIFA, NEVO).

Kontrak robot (ADR-004): action navigate_to_pose, tf map -> base_link di /<namespace>/tf,
opsional sensor_msgs/BatteryState. Semua nama relatif terhadap namespace robot.
"""
import math
import threading

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from tf2_ros import Buffer, TransformListener

from .base import NavStatus, RobotBackend


def _yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Nav2Backend(RobotBackend):

    def __init__(self, name, config, use_sim_time):
        self.name = name
        ns = config.get('namespace', name)
        self.map_frame = config.get('map_frame', 'map')
        self.base_frame = config.get('base_frame', 'base_link')
        self.pose_timeout = float(config.get('pose_timeout', 2.0))

        # Node per robot: tf robot ada di /<ns>/tf (bukan /tf global), jadi listener di-remap.
        self.node = rclpy.create_node(
            f'orbit_nav2_{name}', namespace=ns, use_global_arguments=False,
            cli_args=['--ros-args', '-r', f'/tf:=/{ns}/tf', '-r', f'/tf_static:=/{ns}/tf_static'],
            parameter_overrides=[Parameter('use_sim_time', value=use_sim_time)])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.nav_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

        self._battery = 1.0
        battery_topic = config.get('battery_topic')
        if battery_topic:
            self.node.create_subscription(BatteryState, battery_topic, self._on_battery, 10)

        self._lock = threading.Lock()
        self._goal_seq = 0
        self._goal_handle = None
        self._status = NavStatus.IDLE

    def nodes(self):
        return [self.node]

    def pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:
            return None
        # Waktu tf dibandingkan dengan jam node (sim time di simulasi)
        age = (self.node.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
        if age > self.pose_timeout:
            return None
        tr = t.transform.translation
        return (tr.x, tr.y, _yaw_from_quaternion(t.transform.rotation))

    def battery_soc(self):
        return self._battery

    def _on_battery(self, msg):
        if not math.isnan(msg.percentage):
            self._battery = max(0.0, min(1.0, msg.percentage))

    def navigate(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        with self._lock:
            self._goal_seq += 1
            seq = self._goal_seq
            self._status = NavStatus.ACTIVE
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error('navigate_to_pose tidak tersedia')
            self._set_status(seq, NavStatus.FAILED)
            return
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_goal_response(seq, f))

    def _on_goal_response(self, seq, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.node.get_logger().warning('goal Nav2 ditolak')
            self._set_status(seq, NavStatus.FAILED)
            return
        with self._lock:
            if seq != self._goal_seq:
                # Sudah ada goal baru atau stop sebelum goal ini diterima
                handle.cancel_goal_async()
                return
            self._goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self._on_result(seq, f))

    def _on_result(self, seq, future):
        status = future.result().status
        self._set_status(
            seq, NavStatus.SUCCEEDED if status == GoalStatus.STATUS_SUCCEEDED else NavStatus.FAILED)

    def _set_status(self, seq, status):
        with self._lock:
            if seq == self._goal_seq:
                self._status = status
                if status != NavStatus.ACTIVE:
                    self._goal_handle = None

    def stop(self):
        with self._lock:
            self._goal_seq += 1
            handle, self._goal_handle = self._goal_handle, None
            self._status = NavStatus.IDLE
        if handle is not None:
            handle.cancel_goal_async()

    def nav_status(self):
        with self._lock:
            return self._status
