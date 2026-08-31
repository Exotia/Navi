"""goal_relay on a real ROS graph, against a real (but scripted)
NavigateToPose action server, on a throwaway domain.

  bash -c 'source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=94 \
    python3 -m pytest rover/src/navi_autonomy/test/test_goal_relay.py -q'

94 rather than 92: test_sim_bridge.py already pins 92 as its SIM_DOMAIN,
and 91 is test_autonomy_graph.py's. Never domain 0, where the rover and
the simulation live.

/nav_request and /mode_status are delivered by calling GoalRelay's own
subscription callbacks directly - the navi_supervisor/test_mode_supervisor.py
convention - rather than round-tripping plain std_msgs/String through
pub/sub, which buys nothing here and only adds discovery latency. What
must cross the real ROS graph, and does, is the one thing no fake port can
stand in for: the NavigateToPose action between GoalRelay's real
ActionClientNav2Goals and the fake server below.
"""

import json
import math
import os

os.environ.setdefault("ROS_DOMAIN_ID", "94")   # throwaway; never the rover's

import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from navi_autonomy import nav_run as rules
from navi_autonomy.goal_relay import GoalRelay
from navi_autonomy.task_control import RecordingTaskControl


class Clock:
    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t


class _Yield:
    """Suspend the fake server's coroutine until the next spin_once(): a
    scripted goal sits "in flight" without blocking the executor thread
    that GoalRelay and this server share - a plain blocking wait here would
    freeze GoalRelay's own timers along with it."""

    def __await__(self):
        yield


class FakeNav2Server(Node):
    """A NavigateToPose action server whose outcome per goal is scripted
    from the test thread via succeed()/abort() - or genuinely cancelled,
    when Nav2Goals.cancel() reaches here as a real cancel request."""

    def __init__(self):
        super().__init__("fake_nav2_server")
        self.received_goals = []      # NavigateToPose.Goal, arrival order
        self.cancel_count = 0
        self._pending = []            # goal handles awaiting a scripted outcome
        self._server = ActionServer(
            self, NavigateToPose, "navigate_to_pose",
            execute_callback=self._execute, cancel_callback=self._on_cancel)

    def _on_cancel(self, cancel_request):
        self.cancel_count += 1
        return CancelResponse.ACCEPT

    async def _execute(self, goal_handle):
        self.received_goals.append(goal_handle.request)
        self._pending.append(goal_handle)
        try:
            while goal_handle in self._pending:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self._pending.remove(goal_handle)
                    break
                await _Yield()
        finally:
            if goal_handle in self._pending:
                self._pending.remove(goal_handle)
        return NavigateToPose.Result()

    def succeed(self, index=0):
        self._pending.pop(index).succeed()

    def abort(self, index=0):
        self._pending.pop(index).abort()


def spin(executor, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def graph_factory():
    """Each call builds a fresh {GoalRelay, FakeNav2Server} pair on one
    executor and returns it; every one built is torn down when the test
    ends. A factory rather than a single fixture value because a few tests
    need control over the RecordingTaskControl's initial coordinator
    state, which has to be set before GoalRelay's first tick."""
    made = []

    def _make(state="Autonomous"):
        clock = Clock()
        task = RecordingTaskControl(state=state)
        server = FakeNav2Server()
        relay = GoalRelay(clock=clock, task_control=task)
        statuses, summaries = [], []
        # Read off the publishers directly, the test_mode_supervisor.py
        # convention: the ROS graph cache for a plain String topic is not
        # worth the discovery latency when the object is right here.
        relay._nav_status_pub.publish = lambda msg: statuses.append(json.loads(msg.data))
        relay._nav_path_summary_pub.publish = lambda msg: summaries.append(json.loads(msg.data))
        executor = SingleThreadedExecutor()
        executor.add_node(server)
        executor.add_node(relay)
        spin(executor, 1.0)     # discovery: the action client must find the server
        bundle = (executor, relay, server, task, clock, statuses, summaries)
        made.append(bundle)
        return bundle

    yield _make
    for executor, relay, server, task, clock, statuses, summaries in made:
        executor.remove_node(relay)
        executor.remove_node(server)
        relay.destroy_node()
        server.destroy_node()


def _string(payload):
    m = String()
    m.data = payload if isinstance(payload, str) else json.dumps(payload)
    return m


def mode(name):
    return _string({"mode": name})


def go(waypoints=((3.0, -1.5), (8.0, -1.5)), run_id="gs-1"):
    return _string({"action": "go", "run_id": run_id, "frame_id": "map",
                    "waypoints": [{"x": x, "y": y, "yaw": None} for x, y in waypoints]})


def test_go_publishes_a_refusal_when_the_mode_is_not_autonomous(graph_factory):
    executor, relay, server, task, clock, statuses, summaries = graph_factory()
    relay._on_mode_status(mode("manual"))
    relay._on_nav_request(go())
    assert statuses[-1]["state"] == "refused"
    assert "autonomous" in statuses[-1]["error"]
    spin(executor, 0.5)
    assert server.received_goals == []


def test_go_in_autonomous_reaches_the_fake_action_server_with_the_first_waypoint(graph_factory):
    executor, relay, server, task, clock, statuses, summaries = graph_factory()
    relay._on_mode_status(mode("autonomous"))
    relay._on_nav_request(go())
    spin(executor, 2.0)
    assert len(server.received_goals) == 1
    goal = server.received_goals[0]
    assert goal.pose.pose.position.x == pytest.approx(3.0)
    assert goal.pose.pose.position.y == pytest.approx(-1.5)
    assert goal.pose.header.frame_id == "map"


def test_the_coordinator_is_told_before_nav2_is(graph_factory):
    executor, relay, server, task, clock, statuses, summaries = graph_factory(state=None)
    relay._on_mode_status(mode("autonomous"))
    relay._on_nav_request(go())
    assert task.calls[0] == ("start_task", ((3.0, -1.5), (8.0, -1.5)))
    spin(executor, 0.5)
    assert server.received_goals == [], "goal sent before the coordinator armed"
    task.state = "Autonomous"
    spin(executor, 2.0)
    assert len(server.received_goals) == 1


def test_go_with_an_unarmed_coordinator_aborts_at_the_timeout(graph_factory):
    executor, relay, server, task, clock, statuses, summaries = graph_factory(state="Idle")
    relay._on_mode_status(mode("autonomous"))
    relay._on_nav_request(go())
    clock.t = rules.ARM_TIMEOUT_S + 0.1
    spin(executor, 1.0)
    assert statuses[-1]["state"] == "aborted"
    assert "coordinator" in statuses[-1]["error"]
    assert server.received_goals == []


def test_each_waypoint_reached_is_notified_and_the_last_one_is_the_destination(graph_factory):
    executor, relay, server, task, clock, statuses, summaries = graph_factory()
    relay._on_mode_status(mode("autonomous"))
    relay._on_nav_request(go())
    spin(executor, 2.0)
    assert len(server.received_goals) == 1

    server.succeed(0)
    spin(executor, 2.0)
    assert ("waypoint", 0) in task.calls
    assert ("destination", None) not in task.calls
    assert len(server.received_goals) == 2

    server.succeed(0)
    spin(executor, 2.0)
    assert ("waypoint", 1) in task.calls
    assert ("destination", None) in task.calls
    assert statuses[-1]["state"] == "succeeded"


def test_a_pause_survives_the_cancelled_goals_own_result(graph_factory):
    executor, relay, server, task, clock, statuses, summaries = graph_factory()
    relay._on_mode_status(mode("autonomous"))
    relay._on_nav_request(go())
    spin(executor, 2.0)
    assert len(server.received_goals) == 1

    relay._on_nav_request(_string({"action": "pause", "run_id": "gs-1"}))
    # The fake server's cancel_callback fires, its execute() coroutine sees
    # is_cancel_requested and calls canceled(), and the action's result
    # (STATUS_CANCELED) reaches ActionClientNav2Goals - all inside this spin.
    spin(executor, 2.0)

    assert statuses[-1]["state"] == "paused"
    assert ("pause", None) in task.calls
    assert ("abort", None) not in task.calls, \
        "the cancelled goal's own result reported itself as a failure"


def test_abort_cancels_the_goal_and_aborts_the_task(graph_factory):
    executor, relay, server, task, clock, statuses, summaries = graph_factory()
    relay._on_mode_status(mode("autonomous"))
    relay._on_nav_request(go())
    spin(executor, 2.0)
    assert len(server.received_goals) == 1

    relay._on_nav_request(_string({"action": "abort", "run_id": "gs-1"}))
    spin(executor, 2.0)
    assert server.cancel_count >= 1
    assert task.calls.count(("abort", None)) == 1

    spin(executor, 1.0)     # the cancelled goal's own result lands here too
    assert task.calls.count(("abort", None)) == 1


def test_a_bumped_stop_seq_cancels_the_goal_and_pauses_the_run(graph_factory):
    # SP11 task 8 / SP8 C1: navi_rpc_server bumps stop_seq on /navi_rpc/status
    # for F6, F7(false) and a failed run alike; goal_relay is the one that
    # cancels the Nav2 goal, which is what actually stops /autonomy_twist.
    executor, relay, server, task, clock, statuses, summaries = graph_factory()
    relay._on_mode_status(mode("autonomous"))
    relay._on_nav_request(go())
    spin(executor, 2.0)
    assert len(server.received_goals) == 1

    relay._on_navi_rpc_status(_string({"stop_seq": 0}))    # baseline, no-op
    spin(executor, 0.5)
    assert statuses[-1]["state"] == "running"

    relay._on_navi_rpc_status(_string({"stop_seq": 1}))
    spin(executor, 2.0)
    assert statuses[-1]["state"] == "paused"
    assert server.cancel_count >= 1


def test_goal_relay_binds_naviRpcTaskControl_to_the_real_wire(ros):
    """8.5's binding test: with goal_relay wired the way main() wires it (no
    task_control override), a Go produces {"action": "navi_task"} on
    /drive_command with the operator's waypoints, and a completed run
    produces notifyTaskFinished's payload - waypoint_reached then
    destination_reached - on /navi_rpc/progress. This is the one test that
    checks the real NaviRpcTaskControl reaches the real topics rather than
    RecordingTaskControl standing in for it.
    """
    from navi_autonomy.task_control import NaviRpcTaskControl

    class WireListener(Node):
        def __init__(self):
            super().__init__("wire_listener")
            self.drive_commands = []
            self.progress = []
            self.create_subscription(
                String, "/drive_command",
                lambda m: self.drive_commands.append(json.loads(m.data)), 10)
            self.create_subscription(
                String, "/navi_rpc/progress",
                lambda m: self.progress.append(json.loads(m.data)), 10)
            self.status_pub = self.create_publisher(String, "/drive_status", 1)

    clock = Clock()
    server = FakeNav2Server()
    listener = WireListener()
    relay = GoalRelay(clock=clock)     # no task_control override: main()'s own wiring
    assert isinstance(relay._task, NaviRpcTaskControl)

    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(listener)
    executor.add_node(relay)
    try:
        spin(executor, 1.0)     # discovery

        relay._on_mode_status(mode("autonomous"))
        relay._on_nav_request(go())
        spin(executor, 1.0)
        assert {"action": "navi_task",
                "waypoints": [[3.0, -1.5, 0.0], [8.0, -1.5, 0.0]]} \
            in listener.drive_commands

        # Arm the run over the real wire, same as bema_bridge would report it.
        listener.status_pub.publish(_string({"coordinator_state": 5}))
        spin(executor, 2.0)
        assert len(server.received_goals) == 1

        server.succeed(0)
        spin(executor, 2.0)
        assert {"event": "waypoint_reached", "index": 0, "reason": None} \
            in listener.progress

        server.succeed(0)
        spin(executor, 2.0)
        assert {"event": "destination_reached", "index": None, "reason": None} \
            in listener.progress
    finally:
        executor.remove_node(relay)
        executor.remove_node(server)
        executor.remove_node(listener)
        relay.destroy_node()
        server.destroy_node()
        listener.destroy_node()


def test_nav_status_is_published_on_every_state_change_and_at_2_hz(graph_factory):
    executor, relay, server, task, clock, statuses, summaries = graph_factory()
    before = len(statuses)
    spin(executor, 1.1)      # nothing happens: the 2 Hz baseline still ticks
    assert len(statuses) - before >= 2

    relay._on_mode_status(mode("manual"))
    relay._on_nav_request(go())
    # Published by the callback itself, not by the next status tick.
    assert statuses[-1]["state"] == "refused"


def test_a_plan_is_republished_decimated_on_nav_path_summary(graph_factory):
    executor, relay, server, task, clock, statuses, summaries = graph_factory()
    relay._on_mode_status(mode("autonomous"))
    relay._on_nav_request(go())
    spin(executor, 2.0)

    path = Path()
    path.header.frame_id = "map"
    for i in range(400):
        pose = PoseStamped()
        pose.pose.position.x = i * 0.05
        pose.pose.position.y = math.sin(i * 0.05)
        path.poses.append(pose)
    relay._on_plan(path)
    spin(executor, 1.0)

    assert summaries, "no /nav_path_summary published"
    summary = summaries[-1]
    assert len(summary["points"]) <= 60
    assert summary["source_points"] == 400


def test_a_finished_run_publishes_an_empty_summary_that_clears_the_drawing(graph_factory):
    executor, relay, server, task, clock, statuses, summaries = graph_factory()
    relay._on_mode_status(mode("autonomous"))
    relay._on_nav_request(go(waypoints=((3.0, -1.5),)))
    spin(executor, 2.0)

    path = Path()
    for x, y in ((0.0, 0.0), (1.0, 0.0), (3.0, -1.5)):
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = x, y
        path.poses.append(pose)
    relay._on_plan(path)
    spin(executor, 1.0)
    assert summaries[-1]["source_points"] == 3

    server.succeed(0)
    spin(executor, 2.0)
    assert statuses[-1]["state"] == "succeeded"
    assert summaries[-1]["points"] == []
    assert summaries[-1]["source_points"] == 0


def test_an_unreadable_nav_request_is_logged_and_the_node_survives(graph_factory):
    executor, relay, server, task, clock, statuses, summaries = graph_factory()
    relay._on_nav_request(_string("{{{"))     # no exception = the node survives
    relay._on_mode_status(mode("autonomous"))
    relay._on_nav_request(go())
    spin(executor, 2.0)
    assert len(server.received_goals) == 1
