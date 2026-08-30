import json
import os

os.environ.setdefault("ROS_DOMAIN_ID", "91")   # throwaway; never the rover's

import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from navi_supervisor.mode_supervisor import ModeSupervisor
from navi_supervisor.nav2_control import NullNav2Control


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
    nav2 = NullNav2Control()
    supervisor = ModeSupervisor(clock=clock, nav2_control=nav2)
    twists, commands, statuses = [], [], []
    supervisor._twist_pub.publish = lambda msg: twists.append(
        (msg.linear.x, msg.linear.y, msg.angular.z))
    supervisor._command_pub.publish = lambda msg: commands.append(
        json.loads(msg.data)["action"])
    supervisor._status_pub.publish = lambda msg: statuses.append(json.loads(msg.data))
    yield supervisor, clock, nav2, twists, commands, statuses
    supervisor.destroy_node()


def _twist(x, y, wz):
    t = Twist()
    t.linear.x, t.linear.y, t.angular.z = x, y, wz
    return t


def _string(payload):
    m = String()
    m.data = json.dumps(payload)
    return m


def test_a_manual_twist_is_republished_on_rover_twist(node):
    supervisor, clock, nav2, twists, commands, statuses = node
    supervisor._on_manual_twist(_twist(0.05, 0.0, 0.1))
    supervisor._publish_tick()
    assert twists[-1] == pytest.approx((0.05, 0.0, 0.1))


def test_the_deadman_zeroes_and_sends_exactly_one_chassis_stop(node):
    supervisor, clock, nav2, twists, commands, statuses = node
    supervisor._on_manual_twist(_twist(0.05, 0.0, 0.0))
    supervisor._publish_tick()
    clock.t = 2.0
    supervisor._publish_tick()
    supervisor._publish_tick()
    assert twists[-1] == pytest.approx((0.0, 0.0, 0.0))
    assert commands == ["stop"]


def test_an_estop_request_stops_and_latches(node):
    supervisor, clock, nav2, twists, commands, statuses = node
    supervisor._on_manual_twist(_twist(0.05, 0.0, 0.0))
    supervisor._on_estop_request(_string({"reason": "ground station STOP"}))
    supervisor._publish_tick()
    assert twists[-1] == pytest.approx((0.0, 0.0, 0.0))
    assert "stop" in commands
    assert statuses[-1]["mode"] == "estop"
    assert statuses[-1]["reason"] == "ground station STOP"


def test_an_unparseable_estop_request_still_stops(node):
    supervisor, clock, nav2, twists, commands, statuses = node
    supervisor._on_manual_twist(_twist(0.05, 0.0, 0.0))
    bad = String()
    bad.data = "{not json"
    supervisor._on_estop_request(bad)
    supervisor._publish_tick()
    assert twists[-1] == pytest.approx((0.0, 0.0, 0.0))
    assert statuses[-1]["mode"] == "estop"


def test_a_mode_request_back_to_manual_clears_the_latch(node):
    supervisor, clock, nav2, twists, commands, statuses = node
    supervisor._on_estop_request(_string({"reason": "STOP"}))
    supervisor._on_mode_request(_string({"mode": "manual"}))
    assert statuses[-1]["mode"] == "manual"
    assert statuses[-1]["estop_latched"] is False


def test_an_unreadable_mode_request_changes_nothing(node):
    supervisor, clock, nav2, twists, commands, statuses = node
    bad = String()
    bad.data = "{not json"
    supervisor._on_mode_request(bad)
    supervisor._status_tick()
    assert statuses[-1]["mode"] == "manual"


def test_a_takeover_cancels_nav2_and_drives_the_coordinator(node):
    supervisor, clock, nav2, twists, commands, statuses = node
    supervisor._on_mode_request(_string({"mode": "autonomous"}))
    supervisor._on_autonomy_twist(_twist(0.3, 0.0, 0.0))
    supervisor._publish_tick()
    assert twists[-1] == pytest.approx((0.3, 0.0, 0.0))

    supervisor._on_manual_twist(_twist(0.05, 0.0, 0.0))
    assert nav2.calls == [("cancel_goal", "operator takeover"),
                          ("deactivate", "operator takeover")]
    assert commands == ["abort", "manual"]
    supervisor._publish_tick()
    assert twists[-1] == pytest.approx((0.05, 0.0, 0.0))
    # published by the callback itself, not by a status tick: the ground
    # station's gate is driven by this topic.
    assert statuses[-1]["mode"] == "manual"


def test_localisation_loss_halts_autonomy_with_the_reason_in_the_status(node):
    supervisor, clock, nav2, twists, commands, statuses = node
    supervisor._on_mode_request(_string({"mode": "autonomous"}))
    supervisor._on_autonomy_twist(_twist(0.3, 0.0, 0.0))
    supervisor._publish_tick()
    supervisor._on_localization_status(_string({"state": "SEARCHING"}))
    supervisor._publish_tick()
    assert twists[-1] == pytest.approx((0.0, 0.0, 0.0))
    assert nav2.calls[0] == ("cancel_goal", "localisation SEARCHING")
    assert "stop" in commands
    # no _status_tick() here: the callback publishes the change itself
    assert statuses[-1]["mode"] == "manual"
    assert statuses[-1]["reason"] == "localisation SEARCHING"
    assert statuses[-1]["localization_state"] == "SEARCHING"


def test_an_unreadable_localisation_status_does_not_kill_the_node(node):
    supervisor, clock, nav2, twists, commands, statuses = node
    bad = String()
    bad.data = "{not json"
    supervisor._on_localization_status(bad)
    supervisor._publish_tick()      # no exception = pass


def test_the_status_tick_publishes_json_with_every_field(node):
    supervisor, clock, nav2, twists, commands, statuses = node
    supervisor._status_tick()
    status = statuses[-1]
    for key in ("mode", "reason", "source", "deadman_active", "estop_latched",
                "localization_state", "source_age_s"):
        assert key in status


def test_the_node_publishes_rover_twist_and_never_manual_twist(node):
    # Read off the publisher itself rather than the ROS graph: the graph
    # cache is populated by discovery and a test that asks it immediately
    # is a flake waiting to happen.
    supervisor, clock, nav2, twists, commands, statuses = node
    assert supervisor._twist_pub.topic_name == "/rover_twist"
    assert supervisor._status_pub.topic_name == "/mode_status"
    assert supervisor._command_pub.topic_name == "/drive_command"
