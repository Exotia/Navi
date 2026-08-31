"""The node. A real graph on a throwaway domain - never domain 0, and
/manual_twist is never created here."""
import json
import os

os.environ["ROS_DOMAIN_ID"] = "92"

import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from navi_shaper.twist_shaper import TwistShaperNode


class FakeClock:
    """The node's clock, under the test's control.

    Wall time cannot be used here. The shaper's hold windows are seconds long
    and the node measures dt from this clock, so a test that relies on how long
    `spin_once` happens to take is a test that fails on a loaded machine. The
    node takes `clock` for exactly this reason.
    """

    def __init__(self):
        self.t = 0.0

    def advance(self, dt):
        self.t += dt

    def __call__(self):
        return self.t


@pytest.fixture
def graph():
    rclpy.init()
    clock = FakeClock()
    node = TwistShaperNode(clock=clock)
    probe = rclpy.create_node("sp10_probe")
    received = []
    status = []
    probe.create_subscription(Twist, "/chassis_twist", lambda m: received.append(m), 10)
    probe.create_subscription(String, "/ik_feasibility",
                              lambda m: status.append(json.loads(m.data)), 10)
    pub = probe.create_publisher(Twist, "/rover_twist", 10)

    def send(vx, vy, wz, dt=0.05, spins=8):
        clock.advance(dt)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        pub.publish(msg)
        for _ in range(spins):
            rclpy.spin_once(node, timeout_sec=0.02)
            rclpy.spin_once(probe, timeout_sec=0.02)

    def settle(vx, vy, wz):
        """Two messages with a long gap between them.

        The second arrives with dt far above max_dt_s, which the shaper reads
        as "the chassis has long since finished moving" and treats as settled -
        the same path a dropped link takes. Cheaper and more deterministic than
        pumping 72 messages through a real graph to time a hold out.
        """
        send(vx, vy, wz)
        send(vx, vy, wz, dt=10.0)

    yield node, probe, send, settle, received, status
    node.destroy_node()
    probe.destroy_node()
    rclpy.shutdown()


def test_it_does_not_publish_rover_twist(graph):
    """The single-writer rule. The shaper subscribes; it must never publish."""
    node, _, _, _, _, _ = graph
    names = [n for n, _ in node.get_publisher_names_and_types_by_node(
        node.get_name(), node.get_namespace())]
    assert "/rover_twist" not in names
    assert "/chassis_twist" in names


def test_a_settled_command_is_relayed_unchanged(graph):
    _, _, _, settle, received, _ = graph
    settle(0.05, 0.0, 0.1)
    assert received
    assert received[-1].linear.x == pytest.approx(0.05)
    assert received[-1].angular.z == pytest.approx(0.1)


def test_zeros_are_relayed_immediately_and_exactly(graph):
    _, _, send, settle, received, _ = graph
    settle(0.05, 0.0, 0.0)
    before = len(received)
    send(0.0, 0.0, 0.0)
    assert len(received) > before, "a stop must produce an output message"
    assert (received[-1].linear.x, received[-1].linear.y, received[-1].angular.z) == (0.0, 0.0, 0.0)


def test_one_output_per_input_no_resampling(graph):
    _, _, send, _, received, _ = graph
    received.clear()
    for _ in range(10):
        send(0.05, 0.0, 0.1)
    assert len(received) == 10


def test_a_geometry_jump_is_shaped_not_clipped(graph):
    _, _, send, settle, received, _ = graph
    settle(0.05, 0.0, 0.0)
    received.clear()
    send(0.0, 0.0, 0.1)
    out = received[-1]
    assert out.angular.z != 0.0, "shaped, not clipped - the command still gets through"
    assert abs(out.angular.z) < 0.1, "but scaled down while the wheels catch up"
    assert out.angular.z > 0.0, "and never with a flipped sign"
    assert out.angular.z == pytest.approx(0.1 * 0.106, abs=0.002)


def test_feasibility_status_is_json_with_the_agreed_keys(graph):
    node, probe, send, settle, _, status = graph
    settle(0.05, 0.0, 0.0)
    send(0.0, 0.0, 0.1)
    node._publish_status()
    for _ in range(10):
        rclpy.spin_once(probe, timeout_sec=0.02)
    assert status
    s = status[-1]
    assert set(s) >= {"gain", "feasible", "limited_by", "also_limited_by",
                      "fidelity_err_rad", "delta_beta_rad",
                      "hold_remaining_s", "icr_x", "icr_y", "shaped_count",
                      "straight_bias_rad_s"}
    assert 0.0 <= s["gain"] <= 1.0
    assert isinstance(s["feasible"], bool)
    assert s["limited_by"] == "slew"
    assert s["shaped_count"] >= 1


def test_parameters_are_declared_and_change_behaviour(graph):
    node, _, send, settle, received, _ = graph
    node.set_parameters([rclpy.parameter.Parameter(
        "backstop_max_vx", rclpy.Parameter.Type.DOUBLE, 0.01)])
    received.clear()
    settle(0.05, 0.0, 0.0)
    # 0.01 / 0.05 = a backstop gain of 0.2, and the backstop is a hard ceiling
    # that deliberately overrides the fidelity guard - so this settles at 0.01
    # m/s even though a gain of 0.2 carries about 0.2365 rad of geometry error,
    # well past the 0.10 rad default tolerance. That is the documented
    # precedence, not a bug; the error is reported, not hidden.
    assert abs(received[-1].linear.x) <= 0.01 + 1e-9


def test_an_overriding_backstop_reports_its_true_geometry_error(graph):
    """The backstop wins over the fidelity guard, and says so honestly."""
    node, probe, send, settle, _, status = graph
    node.set_parameters([rclpy.parameter.Parameter(
        "backstop_max_vx", rclpy.Parameter.Type.DOUBLE, 0.01)])
    settle(0.05, 0.0, 0.0)
    node._publish_status()
    for _ in range(10):
        rclpy.spin_once(probe, timeout_sec=0.02)
    s = status[-1]
    assert s["limited_by"] == "backstop"
    # Not clipped to the tolerance, and not silently zero: the payload carries
    # the error the output really has, so an out-of-tolerance backstop is
    # visible on /ik_feasibility rather than only in the rover's tracks.
    assert s["fidelity_err_rad"] > 0.10
