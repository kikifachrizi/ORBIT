"""Contract test RobotAdapter dengan FakeBackend: tanpa ROS, Gazebo, atau RMF core (ADR-002)."""
import logging

from orbit_adapter.backends.base import NavStatus, RobotBackend
from orbit_adapter.robot_adapter import RobotAdapter


class FakeBackend(RobotBackend):

    def __init__(self):
        self.current_pose = None
        self.status = NavStatus.IDLE
        self.goals = []
        self.stops = 0

    def pose(self):
        return self.current_pose

    def battery_soc(self):
        return 0.8

    def navigate(self, x, y, yaw):
        self.goals.append((x, y, yaw))
        self.status = NavStatus.ACTIVE

    def stop(self):
        self.stops += 1
        self.status = NavStatus.IDLE

    def nav_status(self):
        return self.status


class FakeIdentifier:

    def is_same(self, other):
        return other is self


class FakeExecution:

    def __init__(self):
        self.identifier = FakeIdentifier()
        self.finished_count = 0

    def finished(self):
        self.finished_count += 1


class FakeDestination:

    def __init__(self, x, y, yaw, name='north'):
        self.position = [x, y, yaw]
        self.name = name


class FakeUpdateHandle:

    def __init__(self):
        self.updates = []

    def update(self, state, activity):
        self.updates.append((list(state.position), activity))


class FakeFleetHandle:

    def __init__(self):
        self.added = []
        self.handle = FakeUpdateHandle()

    def add_robot(self, name, state, config, callbacks):
        self.added.append((name, list(state.position), callbacks))
        return self.handle


def make_robot(**kwargs):
    backend = FakeBackend()
    fleet = FakeFleetHandle()
    robot = RobotAdapter('tifa1', backend, fleet, robot_config=None, map_name='L1',
                         logger=logging.getLogger('test'), **kwargs)
    return robot, backend, fleet


def registered_robot(**kwargs):
    robot, backend, fleet = make_robot(**kwargs)
    backend.current_pose = (1.0, 2.0, 0.5)
    robot.update()
    return robot, backend, fleet


def test_registers_only_after_first_pose():
    robot, backend, fleet = make_robot()
    robot.update()
    assert fleet.added == []
    backend.current_pose = (1.0, 2.0, 0.5)
    robot.update()
    assert [a[0] for a in fleet.added] == ['tifa1']
    assert fleet.added[0][1] == [1.0, 2.0, 0.5]


def test_navigate_success_finishes_execution():
    robot, backend, fleet = registered_robot()
    execution = FakeExecution()
    robot._navigate(FakeDestination(3.0, 4.0, 1.0), execution)
    assert backend.goals == [(3.0, 4.0, 1.0)]
    robot.update()
    assert fleet.handle.updates[-1][1] is execution.identifier
    backend.status = NavStatus.SUCCEEDED
    robot.update()
    assert execution.finished_count == 1
    assert fleet.handle.updates[-1][1] is None


def test_failed_goal_is_retried_then_given_up():
    robot, backend, _ = registered_robot(max_retries=2, retry_delay=0.0)
    execution = FakeExecution()
    robot._navigate(FakeDestination(3.0, 4.0, 1.0), execution)
    for _ in range(2):
        backend.status = NavStatus.FAILED
        robot.update()  # jadwalkan retry
        robot.update()  # kirim ulang goal
    assert len(backend.goals) == 3
    backend.status = NavStatus.FAILED
    robot.update()
    assert execution.finished_count == 1


def test_stop_cancels_only_matching_activity():
    robot, backend, _ = registered_robot()
    execution = FakeExecution()
    robot._navigate(FakeDestination(3.0, 4.0, 1.0), execution)
    robot._stop(FakeIdentifier())
    assert backend.stops == 0
    robot._stop(execution.identifier)
    assert backend.stops == 1


def test_stale_pose_pauses_updates():
    robot, backend, fleet = registered_robot()
    n = len(fleet.handle.updates)
    backend.current_pose = None
    robot.update()
    assert len(fleet.handle.updates) == n
