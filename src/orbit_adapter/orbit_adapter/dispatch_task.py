"""Kirim task ke RMF lewat /task_api_requests (pengganti rmf_demos_tasks).

    ros2 run orbit_adapter dispatch_task patrol north east --rounds 2
    ros2 run orbit_adapter dispatch_task go_to south --fleet tifa --robot tifa2

Tanpa --robot, RMF memilih robot (dispatch_task_request). Dengan --fleet dan --robot,
task langsung ke robot itu (robot_task_request). Keluar 0 bila RMF menerima task.
"""
import argparse
import json
import sys
import uuid

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rmf_task_msgs.msg import ApiRequest, ApiResponse


def build_request(args, now_millis):
    if args.kind == 'patrol':
        task = {'category': 'patrol',
                'description': {'places': args.places, 'rounds': args.rounds}}
    else:
        activity = {'category': 'go_to_place', 'description': {'waypoint': args.place}}
        task = {'category': 'compose',
                'description': {'category': 'go_to_place', 'phases': [{'activity': activity}]}}
    task['unix_millis_earliest_start_time'] = now_millis

    if args.robot:
        return {'type': 'robot_task_request', 'robot': args.robot, 'fleet': args.fleet,
                'request': task}
    return {'type': 'dispatch_task_request', 'request': task}


def main(argv=sys.argv):
    rclpy.init(args=argv)
    parser = argparse.ArgumentParser(prog='dispatch_task')
    parser.add_argument('--fleet', default='tifa')
    parser.add_argument('--robot', default='', help='Kosong = RMF memilih robot')
    parser.add_argument('--sim', action='store_true', help='Pakai sim time untuk start time')
    parser.add_argument('--timeout', type=float, default=15.0, help='Batas tunggu balasan RMF (s)')
    sub = parser.add_subparsers(dest='kind', required=True)
    patrol = sub.add_parser('patrol')
    patrol.add_argument('places', nargs='+')
    patrol.add_argument('--rounds', type=int, default=1)
    go_to = sub.add_parser('go_to')
    go_to.add_argument('place')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(argv)[1:])

    node = rclpy.create_node('orbit_dispatch_task',
                             parameter_overrides=[Parameter('use_sim_time', value=args.sim)])
    if args.sim:
        # Tunggu /clock supaya start time tidak 0
        while rclpy.ok() and node.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(node, timeout_sec=0.1)

    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(ApiRequest, 'task_api_requests', qos)

    msg = ApiRequest()
    msg.request_id = f'{args.kind}_{uuid.uuid4()}'
    msg.json_msg = json.dumps(build_request(args, node.get_clock().now().nanoseconds // 1_000_000))
    print(f'request_id: {msg.request_id}\n{msg.json_msg}')

    # Publisher transient-local dibiarkan hidup sampai RMF membalas di task_api_responses:
    # di graph besar (RMF + beberapa stack Nav2) fleet adapter bisa ter-match beberapa detik
    # kemudian, dan request hilang bila proses ini keluar lebih dulu.
    responses = []
    node.create_subscription(
        ApiResponse, 'task_api_responses',
        lambda r: responses.append(r) if r.request_id == msg.request_id else None, qos)
    pub.publish(msg)
    for _ in range(int(args.timeout * 10)):
        rclpy.spin_once(node, timeout_sec=0.1)
        if any(r.type == ApiResponse.TYPE_RESPONDING for r in responses):
            break

    reply = next((r for r in responses if r.type == ApiResponse.TYPE_RESPONDING), None)
    node.destroy_node()
    rclpy.try_shutdown()
    if reply is None:
        print(f'gagal: tidak ada balasan RMF dalam {args.timeout:.0f} s', file=sys.stderr)
        return 1
    result = json.loads(reply.json_msg)
    if not result.get('success', False):
        print(f'ditolak RMF: {result.get("errors", result)}', file=sys.stderr)
        return 1
    state = result.get('state', {})
    print(f'diterima: {state.get("booking", {}).get("id")} -> '
          f'{state.get("assigned_to", {}).get("name", "(dipilih RMF)")}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
