import json
import os

os.environ.setdefault("ROS_DOMAIN_ID", "91")   # throwaway; never the rover's

import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String

from fake_coordinator import FakeCoordinator, RpcError
from navi_supervisor.navi_rpc_server import NaviRpcServer


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    clock = Clock()
    server = NaviRpcServer(clock=clock, parameter_overrides=[
        Parameter("bind_host", Parameter.Type.STRING, "127.0.0.1"),
        Parameter("port", Parameter.Type.INTEGER, 0)])
    paths, statuses, commands = [], [], []
    server._targets_pub.publish = lambda msg: paths.append(msg)
    server._status_pub.publish = lambda msg: statuses.append(json.loads(msg.data))
    server._command_pub.publish = lambda msg: commands.append(json.loads(msg.data))
    client = FakeCoordinator("127.0.0.1", server.port)
    yield server, clock, client, paths, statuses, commands
    client.close()
    server.destroy_node()


def _string(payload):
    msg = String()
    msg.data = json.dumps(payload)
    return msg


def test_the_server_binds_loopback_on_an_ephemeral_port(node):
    server, clock, client, paths, statuses, commands = node
    assert server.port != 0
    assert server.port != 21021


def test_set_targets_publishes_a_latched_path_in_the_map_frame(node):
    server, clock, client, paths, statuses, commands = node
    client.access()
    client.call("F3", [[1.0, 2.0, 0.0], [3.0, 4.0, 3.141592653589793]])
    server._action_tick()
    path = paths[-1]
    assert path.header.frame_id == "map"
    assert len(path.poses) == 2
    assert path.poses[0].header.frame_id == "map"
    assert path.poses[0].pose.position.x == pytest.approx(1.0)
    assert path.poses[0].pose.position.y == pytest.approx(2.0)
    assert path.poses[0].pose.orientation.z == pytest.approx(0.0)
    assert path.poses[0].pose.orientation.w == pytest.approx(1.0)
    assert path.poses[1].pose.orientation.z == pytest.approx(1.0)
    assert path.poses[1].pose.orientation.w == pytest.approx(0.0, abs=1e-9)


def test_start_navigation_shows_up_in_the_status_and_starts_nothing(node):
    server, clock, client, paths, statuses, commands = node
    client.access()
    client.call("F3", [[1.0, 2.0, 0.0]])
    client.call("F4")
    server._action_tick()
    server._status_tick()
    assert statuses[-1]["navigation_requested"] is True
    assert statuses[-1]["start_seq"] == 1
    assert statuses[-1]["listening"].startswith("127.0.0.1:")
    assert commands == []            # no run is commanded from here


def test_a_reached_waypoint_becomes_the_coordinator_tag(node):
    server, clock, client, paths, statuses, commands = node
    client.access()
    client.call("F3", [[1.0, 2.0, 0.0], [3.0, 4.0, 0.0]])
    client.call("F4")
    server._action_tick()
    server._on_progress(_string({"event": "waypoint_reached", "index": 0}))
    assert commands[-1] == {"action": "task_finished", "tag": 0x31}
    assert client.call("F5") is True


def test_the_destination_becomes_the_destination_tag(node):
    server, clock, client, paths, statuses, commands = node
    client.access()
    client.call("F3", [[1.0, 2.0, 0.0]])
    client.call("F4")
    server._action_tick()
    server._on_progress(_string({"event": "destination_reached", "index": 0}))
    assert commands[-1] == {"action": "task_finished", "tag": 0x32}
    server._status_tick()
    assert statuses[-1]["last_point_reached"] is True


def test_a_failure_stops_and_reports_and_invents_no_tag(node):
    server, clock, client, paths, statuses, commands = node
    client.access()
    client.call("F3", [[1.0, 2.0, 0.0]])
    client.call("F4")
    server._action_tick()
    server._on_progress(_string({"event": "failed", "reason": "no valid path"}))
    assert commands[-1] == {"action": "stop"}
    assert all(c.get("action") != "task_finished" for c in commands)
    server._status_tick()
    assert statuses[-1]["last_error"] == "no valid path"


def test_stop_navigation_stops_the_chassis_and_bumps_the_stop_seq(node):
    server, clock, client, paths, statuses, commands = node
    server._on_mode_status(_string({"mode": "autonomous"}))
    client.access()
    client.call("F3", [[1.0, 2.0, 0.0]])
    client.call("F4")
    server._action_tick()
    client.call("F6")
    server._action_tick()
    server._status_tick()
    assert commands[-1] == {"action": "stop"}
    assert statuses[-1]["stop_seq"] == 1
    assert statuses[-1]["stop_requested"] is True
    assert statuses[-1]["navigation_requested"] is False


def test_the_server_never_publishes_a_mode_request(node):
    server, clock, client, paths, statuses, commands = node
    # The regression pin for the pause-becomes-abort trap. CoordinatorImpl::
    # pause() calls F6 while it is still in Autonomous, so a /mode_request
    # from here would come back through the supervisor as COORDINATOR_ABORT
    # and the run would be gone; from estop it would clear the operator's own
    # latch. The supervisor owns mode, alone (SP5).
    assert "/mode_request" not in [p.topic_name for p in server.publishers]
    for mode in ("autonomous", "estop", "manual"):
        server._on_mode_status(_string({"mode": mode}))
        client.access()
        client.call("F3", [[1.0, 2.0, 0.0]])
        client.call("F6")
        server._action_tick()
        assert commands[-1] == {"action": "stop"}


def test_an_unreadable_progress_message_does_not_kill_the_node(node):
    server, clock, client, paths, statuses, commands = node
    bad = String()
    bad.data = "{not json"
    server._on_progress(bad)
    server._on_progress(_string(["not", "a", "dict"]))
    client.access()
    assert client.call("F0") is None


def test_an_unreadable_mode_status_leaves_the_mode_unknown(node):
    server, clock, client, paths, statuses, commands = node
    bad = String()
    bad.data = "nonsense"
    server._on_mode_status(bad)
    client.access()
    client.call("F3", [[1.0, 2.0, 0.0]])
    client.call("F6")
    server._action_tick()
    server._status_tick()
    assert statuses[-1]["supervisor_mode"] is None
    assert commands[-1] == {"action": "stop"}   # the stop goes out regardless


def test_a_hostile_client_leaves_the_node_serving(node):
    server, clock, client, paths, statuses, commands = node
    with pytest.raises(RpcError):
        client.call("F99")
    with pytest.raises(RpcError):
        client.call("F3", "not-an-array")
    client.access()
    assert client.call("F3", [[1.0, 2.0, 0.0]]) is None
    server._status_tick()
    assert statuses[-1]["target_count"] == 1
