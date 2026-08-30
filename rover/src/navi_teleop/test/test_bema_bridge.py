import json
import os

os.environ.setdefault("ROS_DOMAIN_ID", "93")   # throwaway; never the rover's

import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from navi_teleop.bema_bridge import BemaBridge
from navi_teleop.bema_session import BemaSession
from navi_teleop.msgpack_rpc import RpcClient
from fake_bema_server import FakeBemaServer


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
def server():
    s = FakeBemaServer()
    s.start()
    yield s
    s.stop()


@pytest.fixture
def bridge(server):
    clock = Clock()

    def session_factory(host, bema_port, coordinator_port, clk):
        def client_factory(h, port, timeout_s=1.0):
            return RpcClient("127.0.0.1", port, timeout_s)
        sess = BemaSession("127.0.0.1", server.bema_port, server.coordinator_port,
                           clock=clock, client_factory=client_factory)
        sess.connect()
        return sess

    node = BemaBridge(session_factory=session_factory, clock=clock,
                      parameter_overrides=[
                          rclpy.parameter.Parameter("deadman_s", value=1.0)])
    node._clock_ref = clock
    yield node, server, clock
    node.destroy_node()


def _twist(x, y, wz):
    t = Twist()
    t.linear.x, t.linear.y, t.angular.z = x, y, wz
    return t


def test_a_fresh_twist_is_forwarded_as_drive(bridge):
    node, server, clock = bridge
    node._on_twist(_twist(0.2, 0.0, 1.0))    # angular.z 1 rad/s -> -57.3 deg/s
    node._drive_tick()
    f1 = [a for tag, m, a in server.calls if m == "F1"][-1]
    assert f1[0] == pytest.approx(0.2)
    assert f1[2] == pytest.approx(-57.2958, abs=1e-3)


def test_deadman_stops_after_the_timeout(bridge):
    node, server, clock = bridge
    node._on_twist(_twist(0.2, 0.0, 0.0))
    node._drive_tick()
    clock.t = 1.5                            # > deadman_s past the twist
    node._drive_tick()
    assert node._deadman_active is True
    seq = [(m, a) for tag, m, a in server.calls if m in ("F1", "F2")]
    assert ("F2", []) in [(m, a) for m, a in seq]
    last_f1 = [a for m, a in seq if m == "F1"][-1]
    assert last_f1 == [0.0, 0.0, 0.0]


def test_stop_command_is_dispatched(bridge):
    node, server, clock = bridge
    node._on_command(_string({"action": "stop"}))
    assert ("bema", "F2", []) in server.calls


def test_init_command_calls_f0(bridge):
    node, server, clock = bridge
    node._on_command(_string({"action": "init"}))
    assert ("bema", "F0", []) in server.calls


def test_manual_command_calls_the_coordinator(bridge):
    node, server, clock = bridge
    node._on_command(_string({"action": "manual"}))
    assert ("coord", "F6", []) in server.calls


def test_status_tick_publishes_json(bridge):
    node, server, clock = bridge
    published = []
    node._status_pub.publish = lambda msg: published.append(msg.data)
    node._on_twist(_twist(0.1, 0.0, 0.0))
    node._drive_tick()
    node._status_tick()
    status = json.loads(published[-1])
    assert status["connected"] is True
    assert "twist_age_s" in status and "deadman_active" in status


def test_a_malformed_command_does_not_raise(bridge):
    node, server, clock = bridge
    node._on_command(_string({"action": "no_such_action"}))
    node._on_command(_bad_string("{not json"))
    # no exception = pass


def test_status_tick_handles_session_error(bridge):
    node, server, clock = bridge
    # Monkeypatch session.status() to raise
    node._session.status = lambda: (_ for _ in ()).throw(RuntimeError("boom"))
    # Should not raise; error is logged instead
    node._status_tick()


def test_stop_command_latches_over_a_recent_fresh_twist(bridge):
    node, server, clock = bridge
    node._on_twist(_twist(0.4, 0.0, 0.0))
    node._drive_tick()
    node._on_command(_string({"action": "stop"}))
    node._drive_tick()                     # twist was "recent" but latched off
    seq = [(m, a) for tag, m, a in server.calls if m in ("F1", "F2")]
    last_f1 = [a for m, a in seq if m == "F1"][-1]
    assert last_f1 == [0.0, 0.0, 0.0]
    assert node._twist_at is None

    # a fresh twist un-latches it
    node._on_twist(_twist(0.4, 0.0, 0.0))
    node._drive_tick()
    last_f1 = [a for tag, m, a in server.calls if m == "F1"][-1]
    assert last_f1[0] == pytest.approx(0.4)


def test_unknown_action_does_not_become_the_last_action(bridge):
    node, server, clock = bridge
    node._on_command(_string({"action": "manual"}))
    assert node._last_action == "manual"
    node._on_command(_string({"action": "no_such_action"}))
    assert node._last_action == "manual"       # unchanged, not overwritten


def test_status_tick_survives_an_odd_f9_value(bridge):
    node, server, clock = bridge
    server.state = b"weird"                # non-int F9 result
    node._drive_tick()                     # first tick: forces a heartbeat
    published = []
    node._status_pub.publish = lambda msg: published.append(msg.data)
    node._status_tick()
    status = json.loads(published[-1])     # raises if not valid JSON
    assert isinstance(status["coordinator_state"], str)


def _string(payload):
    m = String()
    m.data = json.dumps(payload)
    return m


def _bad_string(raw):
    m = String()
    m.data = raw
    return m
