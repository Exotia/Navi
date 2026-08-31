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
    server.calls.clear()   # connect() now sends its own lease/stop calls
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
    server.calls.clear()   # connect() now sends its own zero-command stop
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


def test_abort_takes_the_coordinator_lease_then_calls_f7(server):
    # Coordinator F7 is abort, and like F6 it sits behind checkAccess() on
    # the coordinator's own lease - the same trap start_manual fell into.
    clock = Clock()
    sess = _session(server, clock)
    server.calls.clear()
    server.set_state(5)                          # Autonomous, so 1 proves F7
    sess.abort()
    assert ("coord", "__sam__request", []) in server.calls
    assert ("coord", "F7", []) in server.calls
    assert server.state == 1                     # back to Idle


def test_abort_without_a_coordinator_is_a_no_op(server):
    clock = Clock()
    sess = _session(server, clock)
    sess._coord = None
    sess.abort()          # no exception = pass


def test_close_after_link_drop_does_not_raise(server):
    clock = Clock()
    sess = _session(server, clock)
    server.stop()
    sess.close()  # should not raise AttributeError


def test_reconnect_starts_with_a_stop(server):
    """Safety rule 1: the link drops at speed with the gamepad still
    streaming a nonzero command, the socket reconnects, and the old
    F1(0.5,...) must not resume - the reconnect has to send a stop on the
    freshly-opened bema client before anything else can reuse it, and clear
    the retained command so nothing revives it on its own."""
    clock = Clock()
    ports = {"bema": server.bema_port, "coord": server.coordinator_port}
    calls_made = []

    def factory(host, port, timeout_s=1.0):
        calls_made.append(port)
        # connect() asks for bema first, then coord, exactly once each per
        # call - alternate on that fixed order so a later connect() can be
        # redirected at a freshly-started server on different ports (a new
        # FakeBemaServer can't reuse the old one's ports).
        target = ports["bema"] if len(calls_made) % 2 == 1 else ports["coord"]
        return RpcClient("127.0.0.1", target, timeout_s)

    sess = BemaSession("127.0.0.1", server.bema_port, server.coordinator_port,
                       clock=clock, client_factory=factory)
    sess.connect()
    sess.set_command(0.5, 0.0, 0.0)
    sess.tick(clock.t)
    assert sess.status()["connected"] is True

    server.stop()
    clock.t = 1.0
    sess.tick(clock.t)                     # marks down, schedules a retry
    assert sess.status()["connected"] is False

    server2 = FakeBemaServer()
    server2.start()
    try:
        ports["bema"] = server2.bema_port
        ports["coord"] = server2.coordinator_port
        sess.connect()                     # simulate the scheduled reconnect

        bema_calls = [(m, a) for tag, m, a in server2.calls if tag == "bema"]
        names = [m for m, a in bema_calls]
        assert names[0] == "__sam__request"
        assert bema_calls[1] == ("F1", [0.0, 0.0, 0.0])
        assert bema_calls[2] == ("F2", [])

        clock.t = 1.1
        sess.tick(clock.t)                 # command was cleared by connect()
        f1_calls = [a for tag, m, a in server2.calls
                    if tag == "bema" and m == "F1"]
        assert f1_calls[-1] == [0.0, 0.0, 0.0]

        sess.set_command(0.3, 0.0, 0.0)
        clock.t = 1.2
        sess.tick(clock.t)
        f1_calls = [a for tag, m, a in server2.calls
                    if tag == "bema" and m == "F1"]
        assert f1_calls[-1] == [pytest.approx(0.3), pytest.approx(0.0),
                                pytest.approx(0.0)]
    finally:
        server2.stop()


def test_f9_non_int_state_is_stringified(server):
    clock = Clock()
    sess = _session(server, clock)
    server.state = [1, 2, 3]       # F9 returning something other than an int
    sess.tick(clock.t)             # forces the first heartbeat -> F9 call
    assert isinstance(sess.status()["coordinator_state"], str)


def test_manual_state_makes_the_session_enable_movement(server):
    # The coordinator cannot push setMovementEnabled itself while this
    # session holds the exclusive lease (its polite request is refused), so
    # on seeing Manual the session must send F7 true - once, not per tick.
    clock = Clock()
    sess = _session(server, clock)
    server.set_state(3)                      # Manual
    server.calls.clear()
    sess.tick(clock.t)                       # heartbeat reads state 3
    clock.t = 1.1
    sess.tick(clock.t)                       # next heartbeat, still Manual
    f7 = [(m, a) for tag, m, a in server.calls if tag == "bema" and m == "F7"]
    assert f7 == [("F7", [True])]
    assert server.movement_enabled is True


def test_movement_enable_not_sent_while_idle(server):
    clock = Clock()
    sess = _session(server, clock)
    server.set_state(1)                      # Idle
    server.calls.clear()
    sess.tick(clock.t)
    assert not [m for tag, m, a in server.calls if m == "F7"]


def test_movement_enable_resent_after_lease_reacquired(server):
    clock = Clock()
    sess = _session(server, clock)
    server.set_state(3)
    sess.tick(clock.t)                       # F7 goes out once
    server.lease_held = False                # coordinator force-took the lease
    server.calls.clear()
    clock.t = 0.6
    sess.tick(clock.t)                       # ping false -> re-request -> re-enable
    clock.t = 1.2
    sess.tick(clock.t)
    f7 = [(m, a) for tag, m, a in server.calls if m == "F7"]
    assert f7 == [("F7", [True])]


def test_an_rpc_refusal_drops_the_lease_not_the_link(server):
    # An application-level refusal (RpcError, e.g. error 1 from checkAccess)
    # on an otherwise healthy socket must NOT be treated like a dead link:
    # the coordinator can force-take the BEMA lease at any moment to
    # disable movement, and the session's own re-request can be refused
    # too. That must drop the lease and let the next tick's re-request
    # recover it - not tear down both sockets, drop the F10 heartbeat, and
    # back off for 1-5s (which drops the coordinator to Disconnected).
    clock = Clock()
    sess = _session(server, clock)
    sess.set_command(0.1, 0.0, 0.0)
    sess.tick(clock.t)                       # baseline: lease held, F1 flows
    assert sess.status()["lease"] is True

    server.lease_held = False                # coordinator force-took the BEMA lease
    server.refuse_requests = True            # our own re-request is refused too
    clock.t = 0.6                            # >= PING_INTERVAL_S: ping notices
    sess.tick(clock.t)

    status = sess.status()
    assert status["connected"] is True       # no teardown of the sockets
    assert status["lease"] is False
    assert status["last_error"] is not None
    assert "refused" in status["last_error"].lower() or "false" in status["last_error"].lower()
    assert status["reconnect_in_s"] is None  # no backoff scheduled

    server.refuse_requests = False           # the coordinator lets go again
    clock.t = 1.6                            # past the next ping interval
    sess.tick(clock.t)

    assert sess.status()["lease"] is True
    f1_calls = [a for tag, m, a in server.calls if tag == "bema" and m == "F1"]
    assert f1_calls[-1] == [pytest.approx(0.1), pytest.approx(0.0), pytest.approx(0.0)]


def test_start_manual_takes_the_coordinator_lease_first(server):
    # CoordinatorProxy guards F6 behind checkAccess() on the coordinator's
    # OWN lease (separate from BEMA's). Without requesting it, F6 answers
    # error 1, the session marks itself down, and Manual never engages -
    # the "RPC error for a second on pressing Manual" bug.
    clock = Clock()
    sess = _session(server, clock)
    server.calls.clear()
    sess.start_manual()
    coord = [(m, a) for tag, m, a in server.calls if tag == "coord"]
    assert ("__sam__request", []) in coord
    assert coord.index(("__sam__request", [])) < coord.index(("F6", []))
    assert server.state == 3                      # startManual actually ran
    assert sess.status()["connected"] is True     # and nothing marked down


def test_notify_task_finished_calls_the_coordinators_f8(server):
    clock = Clock()
    sess = _session(server, clock)
    server.calls.clear()
    sess.notify_task_finished(0x31)
    assert ("coord", "F8", [0x31]) in server.calls


def test_notify_task_finished_needs_no_coordinator_lease(server):
    clock = Clock()
    sess = _session(server, clock)
    server.coord_lease_held = False
    server.calls.clear()
    sess.notify_task_finished(0x32)
    assert ("coord", "F8", [0x32]) in server.calls
    assert not any(m == "__sam__request" for tag, m, a in server.calls
                   if tag == "coord")
    assert sess.status()["last_error"] is None


def test_start_navi_task_takes_the_coordinator_lease_and_calls_f0(server):
    clock = Clock()
    sess = _session(server, clock)
    server.calls.clear()
    sess.start_navi_task([(1.0, 2.0, 0.0), (3.0, 4.0, 1.5)])
    coord = [(m, a) for tag, m, a in server.calls if tag == "coord"]
    # The lease first, then the guarded call - the same order start_manual
    # and abort use, because CoordinatorProxy guards F0 the same way.
    assert coord[0][0] == "__sam__request"
    assert ("F0", [[[1.0, 2.0, 0.0], [3.0, 4.0, 1.5]]]) in coord


def test_start_navi_task_packs_every_waypoint_as_three_floats(server):
    clock = Clock()
    sess = _session(server, clock)
    server.calls.clear()
    sess.start_navi_task([(1, 2, 0)])          # ints on the way in
    sent = [a for tag, m, a in server.calls if tag == "coord" and m == "F0"]
    # F0 takes ONE positional arg (the waypoint array), same as the two-
    # waypoint case above: a = [rows], so sent = [[rows]] one level deeper
    # than the row itself.
    assert sent == [[[[1.0, 2.0, 0.0]]]]
    assert all(isinstance(c, float) for c in sent[0][0][0])


def test_pause_and_resume_are_the_coordinators_f4_and_f5(server):
    clock = Clock()
    sess = _session(server, clock)
    server.calls.clear()
    sess.pause_task()
    sess.resume_task()
    coord = [m for tag, m, a in server.calls if tag == "coord"]
    assert "F4" in coord and "F5" in coord
    assert coord.count("__sam__request") == 2   # both are guarded
    assert sess.status()["last_error"] is None


def test_autonomous_state_makes_the_session_enable_movement_too(server):
    # The Autonomous twin of the Manual case above, found live on the rack:
    # PrepareAutonomous force-disables movement, and once Autonomous arrives
    # the coordinator's polite enable is refused for the lease this session
    # holds - so the session must send F7 true itself, exactly as in Manual.
    clock = Clock()
    sess = _session(server, clock)
    server.set_state(5)                      # Autonomous
    server.calls.clear()
    sess.tick(clock.t)
    clock.t = 1.1
    sess.tick(clock.t)                       # still Autonomous: F7 once, not twice
    f7 = [(m, a) for tag, m, a in server.calls if tag == "bema" and m == "F7"]
    assert f7 == [("F7", [True])]
    assert server.movement_enabled is True
