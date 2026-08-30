import pytest

from navi_teleop.bema_session import BemaSession
from navi_teleop.msgpack_rpc import RpcClient
from fake_bema_server import FakeBemaServer


class Clock:
    def __init__(self):
        self.t = 0.0
    def __call__(self):
        return self.t


@pytest.fixture
def server():
    s = FakeBemaServer()
    s.start()
    yield s
    s.stop()


def _session(server, clock):
    def factory(host, port, timeout_s=1.0):
        return RpcClient("127.0.0.1", port, timeout_s)
    # host is unused because the fake binds loopback; ports select the server
    sess = BemaSession("127.0.0.1", server.bema_port, server.coordinator_port,
                       clock=clock, client_factory=factory)
    sess.connect()
    return sess


def test_connect_takes_the_lease(server):
    clock = Clock()
    sess = _session(server, clock)
    assert sess.status()["lease"] is True
    assert ("bema", "__sam__request", []) in server.calls


def test_tick_sends_drive_and_paces_ping_and_heartbeat(server):
    clock = Clock()
    sess = _session(server, clock)
    sess.set_command(0.1, 0.0, 5.0)
    sess.tick(clock.t)                       # t=0: drive + first ping + first hb
    clock.t = 0.2
    sess.tick(clock.t)                       # 0.2s: drive only
    clock.t = 0.6
    sess.tick(clock.t)                       # 0.6s: drive + ping (>=0.5)
    methods = [m for tag, m, a in server.calls if tag == "bema"]
    assert methods.count("F1") == 3
    assert methods.count("__sam__ping") == 2   # t=0 and t=0.6
    coord = [m for tag, m, a in server.calls if tag == "coord"]
    assert coord.count("F10") == 1             # heartbeat once in [0,0.6)


def test_drive_carries_the_command_verbatim(server):
    clock = Clock()
    sess = _session(server, clock)
    sess.set_command(0.25, -0.1, -12.0)
    sess.tick(clock.t)
    f1 = [a for tag, m, a in server.calls if m == "F1"][0]
    assert f1 == [pytest.approx(0.25), pytest.approx(-0.1), pytest.approx(-12.0)]


def test_stop_sends_zero_then_stopmovement(server):
    clock = Clock()
    sess = _session(server, clock)
    sess.stop()
    seq = [(m, a) for tag, m, a in server.calls if tag == "bema"
           and m in ("F1", "F2")]
    assert seq[0] == ("F1", [0.0, 0.0, 0.0])
    assert seq[1] == ("F2", [])


def test_reconnect_after_the_link_drops(server):
    clock = Clock()
    sess = _session(server, clock)
    sess.tick(clock.t)
    # drop the server; next tick must mark down and schedule a reconnect
    server.stop()
    clock.t = 1.0
    sess.tick(clock.t)
    assert sess.status()["connected"] is False
    # bring a new server up on the same ports is not possible here; assert
    # the session backed off rather than raising
    assert sess.status()["reconnect_in_s"] >= 1.0


def test_start_manual_calls_the_coordinator(server):
    clock = Clock()
    sess = _session(server, clock)
    sess.start_manual()
    assert ("coord", "F6", []) in server.calls
    assert server.state == 3
