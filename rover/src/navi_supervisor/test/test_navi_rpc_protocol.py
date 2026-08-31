"""The wire and the lease, against the client the primary actually runs.

Every server here binds 127.0.0.1 on port 0 - never the .18 alias, never
:21021, never anything that needs root.
"""

import time

import msgpack
import pytest

from fake_coordinator import FakeCoordinator, RpcError
from navi_supervisor.navi_rpc_protocol import (ACCESS_DENIED_ERROR,
                                               LEASE_TIMEOUT_S, Lease,
                                               RpcRefusal, RpcServer)


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class Recorder:
    """A toy method table: enough surface to test the transport with."""

    def __init__(self):
        self.calls = []

    def table(self):
        return {
            "F0": self._noop,
            "F4": self._noop,
            "F5": self._true,
            "F9": self._refuse,
            "echo": self._echo,
            "boom": self._boom,
        }

    def _noop(self):
        self.calls.append("noop")

    def _true(self):
        self.calls.append("true")
        return True

    def _echo(self, value):
        self.calls.append(("echo", value))
        return value

    def _refuse(self, index):
        raise RpcRefusal("F9 (takeSnapshot) is not served by navi_rpc_server")

    def _boom(self):
        raise RuntimeError("handler exploded")


@pytest.fixture
def served():
    clock = Clock()
    recorder = Recorder()
    server = RpcServer(recorder.table(), guarded=("F0", "F4", "F5"),
                       host="127.0.0.1", port=0, clock=clock)
    server.start()
    clients = []

    def connect():
        client = FakeCoordinator("127.0.0.1", server.port)
        clients.append(client)
        return client

    yield server, clock, recorder, connect
    for client in clients:
        client.close()
    server.stop()


# --- the lease, as a unit ------------------------------------------------
def test_a_free_lease_is_granted_and_a_taken_one_is_refused():
    clock = Clock()
    lease = Lease()
    assert lease.request(1, clock.t) is True
    assert lease.request(2, clock.t) is False
    assert lease.request(1, clock.t) is True          # already ours


def test_the_lease_expires_after_four_seconds_of_silence():
    clock = Clock()
    lease = Lease()
    lease.request(1, clock.t)
    clock.t = LEASE_TIMEOUT_S - 0.01
    assert lease.holds(1, clock.t) is True
    clock.t = LEASE_TIMEOUT_S + 0.01
    assert lease.holds(1, clock.t) is False
    assert lease.request(2, clock.t) is True


def test_a_ping_refreshes_the_lease_and_only_for_the_holder():
    clock = Clock()
    lease = Lease()
    lease.request(1, clock.t)
    clock.t = 3.0
    assert lease.ping(1, clock.t) is True
    assert lease.ping(2, clock.t) is False
    clock.t = 6.0                                     # 3 s after the ping
    assert lease.holds(1, clock.t) is True


def test_force_takes_the_lease_from_the_holder():
    clock = Clock()
    lease = Lease()
    lease.request(1, clock.t)
    lease.force(2, clock.t)
    assert lease.holds(2, clock.t) is True
    assert lease.holds(1, clock.t) is False


def test_release_only_works_for_the_holder():
    clock = Clock()
    lease = Lease()
    lease.request(1, clock.t)
    assert lease.release(2, clock.t) is False
    assert lease.release(1, clock.t) is True
    assert lease.request(2, clock.t) is True


def test_a_dropped_connection_frees_the_lease():
    clock = Clock()
    lease = Lease()
    lease.request(1, clock.t)
    lease.drop(1)
    assert lease.request(2, clock.t) is True


# --- the wire ------------------------------------------------------------
def test_the_sam_dance_then_a_guarded_call(served):
    server, clock, recorder, connect = served
    client = connect()
    assert client.access() is True
    assert client.call("F0") is None
    assert client.ping() is True
    assert client.release() is True
    assert recorder.calls == ["noop"]


def test_a_guarded_call_without_the_lease_is_error_one(served):
    server, clock, recorder, connect = served
    client = connect()
    with pytest.raises(RpcError) as excinfo:
        client.call("F0")
    assert excinfo.value.error == ACCESS_DENIED_ERROR
    assert recorder.calls == []


def test_a_second_session_is_refused_the_lease_but_still_served(served):
    server, clock, recorder, connect = served
    first, second = connect(), connect()
    assert first.access() is True
    assert second.access() is False
    with pytest.raises(RpcError):
        second.call("F4")
    assert first.call("F4") is None                   # the holder is unaffected


def test_a_guarded_call_refreshes_the_lease_like_notify_client_activity(served):
    server, clock, recorder, connect = served
    client = connect()
    client.access()
    clock.t = 3.0
    client.call("F4")                                 # activity, not a ping
    clock.t = 6.0
    assert client.call("F4") is None                  # still ours


def test_closing_the_connection_frees_the_lease_for_the_next_session(served):
    server, clock, recorder, connect = served
    first = connect()
    assert first.access() is True
    first.close()
    # Real sleeps, not fake-clock ones: this is the serve thread noticing the
    # FIN, and the fake clock never advances far enough to expire the lease
    # (4 s), so a pass here can only mean the close freed it.
    for _ in range(50):
        second = FakeCoordinator("127.0.0.1", server.port)
        granted = second.access()
        second.close()
        if granted:
            return
        time.sleep(0.02)
    pytest.fail("the lease was never freed by the closed connection")


def test_an_unguarded_method_needs_no_lease(served):
    server, clock, recorder, connect = served
    client = connect()
    assert client.call("echo", 7) == 7


def test_a_bool_result_stays_a_bool_on_the_wire(served):
    server, clock, recorder, connect = served
    client = connect()
    client.access()
    result = client.call("F5")
    assert result is True                             # not 1


def test_a_notification_is_run_and_not_answered(served):
    server, clock, recorder, connect = served
    client = connect()
    client.notify("echo", 3)
    assert client.call("echo", 4) == 4                # the next answer is not the notify's
    assert ("echo", 3) in recorder.calls


# --- hostile input -------------------------------------------------------
def test_an_unknown_method_is_an_error_not_a_crash(served):
    server, clock, recorder, connect = served
    client = connect()
    with pytest.raises(RpcError) as excinfo:
        client.call("F42")
    assert "unknown method" in str(excinfo.value.error)
    assert client.call("echo", 1) == 1


def test_the_camera_methods_are_unknown_methods(served):
    server, clock, recorder, connect = served
    client = connect()
    with pytest.raises(RpcError):
        client.call("F10", 0.0, 0.0, 0.0, 0.0)


def test_a_wrong_argument_count_is_an_error_not_a_crash(served):
    server, clock, recorder, connect = served
    client = connect()
    with pytest.raises(RpcError) as excinfo:
        client.call("echo")                            # echo takes one
    assert "bad arguments" in str(excinfo.value.error)
    with pytest.raises(RpcError):
        client.call("echo", 1, 2, 3)
    assert client.call("echo", 5) == 5


def test_a_refusal_carries_its_message_and_leaves_the_link_up(served):
    server, clock, recorder, connect = served
    client = connect()
    with pytest.raises(RpcError) as excinfo:
        client.call("F9", 0)
    assert "not served" in str(excinfo.value.error)
    assert client.call("echo", 2) == 2


def test_a_handler_that_raises_is_an_error_not_a_dead_thread(served):
    server, clock, recorder, connect = served
    client = connect()
    with pytest.raises(RpcError) as excinfo:
        client.call("boom")
    assert "boom" in str(excinfo.value.error)
    assert client.call("echo", 8) == 8


def test_arguments_that_are_not_an_array_are_an_error(served):
    server, clock, recorder, connect = served
    client = connect()
    client.send_raw(msgpack.packb([0, 99, "echo", "not-an-array"],
                                  use_bin_type=True))
    frame = client.read_frame()
    assert frame[0] == 1 and frame[1] == 99
    assert frame[2] is not None


def test_a_frame_that_is_not_a_request_closes_only_that_connection(served):
    server, clock, recorder, connect = served
    victim, survivor = connect(), connect()
    victim.send_raw(msgpack.packb({"not": "a frame"}, use_bin_type=True))
    assert victim.closed_by_server() is True
    assert survivor.call("echo", 6) == 6


def test_garbage_bytes_close_only_that_connection(served):
    server, clock, recorder, connect = served
    victim, survivor = connect(), connect()
    victim.send_raw(b"\xc1\xc1\xc1\xc1 not msgpack at all \xc1")
    assert victim.closed_by_server() is True
    assert survivor.call("echo", 6) == 6


def test_an_unknown_sam_method_is_an_unknown_method(served):
    server, clock, recorder, connect = served
    client = connect()
    with pytest.raises(RpcError):
        client.call("__sam__whatever")
    assert client.access() is True


def test_a_frame_split_across_two_packets_is_answered(served):
    # The single most load-bearing behaviour of _drain: next(unpacker)
    # raising StopIteration on a partial frame, and resuming on the next
    # feed(). TCP splits frames whenever it feels like it.
    server, clock, recorder, connect = served
    client = connect()
    frame = msgpack.packb([0, 77, "echo", [42]], use_bin_type=True)
    client.send_raw(frame[:3]); time.sleep(0.05); client.send_raw(frame[3:])
    assert client.read_frame() == [1, 77, None, 42]


def test_an_oversized_stream_costs_only_that_connection(served):
    # _MAX_BUFFER_BYTES, and the unpacker.feed -> BufferFull -> close path.
    server, clock, recorder, connect = served
    victim, survivor = connect(), connect()
    try:
        victim.send_raw(b"\x91" * (2 * 1024 * 1024))   # 2 MB, never a complete frame
    except OSError:
        pass
    assert victim.closed_by_server() is True
    assert survivor.call("echo", 6) == 6


def test_a_half_closed_socket_still_gets_its_answer(served):
    # The buffered frame drains and the reply goes out before recv() returns
    # b"". Correct today; pinned so it stays correct.
    server, clock, recorder, connect = served
    client = connect()
    client.send_raw(msgpack.packb([0, 88, "echo", [9]], use_bin_type=True))
    client.half_close()
    assert client.read_frame() == [1, 88, None, 9]
