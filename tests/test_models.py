import pytest
from ground_station.models import DriveState, NodeRegistry


def test_ingest_stores_latest_sample():
    state = DriveState()
    state.ingest(0.4, -0.05, 0.1, now=10.0)

    assert state.latest.linear_x == 0.4
    assert state.latest.linear_y == -0.05
    assert state.latest.angular_z == 0.1
    assert state.latest.received_at == 10.0


def test_rate_hz_is_zero_with_fewer_than_two_samples():
    state = DriveState()
    assert state.rate_hz == 0.0
    state.ingest(0.0, 0.0, 0.0, now=10.0)
    assert state.rate_hz == 0.0


def test_rate_hz_computes_from_recent_samples():
    state = DriveState(rate_window_seconds=2.0)
    # 5 samples spaced 0.1s apart -> 10 Hz
    for i in range(5):
        state.ingest(0.0, 0.0, 0.0, now=10.0 + i * 0.1)

    assert state.rate_hz == pytest.approx(10.0, rel=0.05)


def test_rate_hz_drops_samples_outside_window():
    state = DriveState(rate_window_seconds=1.0)
    state.ingest(0.0, 0.0, 0.0, now=0.0)
    state.ingest(0.0, 0.0, 0.0, now=5.0)
    state.ingest(0.0, 0.0, 0.0, now=5.5)

    # only the last two samples (5.0, 5.5) are within the 1s window
    assert state.rate_hz == pytest.approx(2.0, rel=0.05)


def test_seconds_since_last_none_before_any_sample():
    state = DriveState()
    assert state.seconds_since_last(now=10.0) is None


def test_seconds_since_last_computes_elapsed_time():
    state = DriveState()
    state.ingest(0.0, 0.0, 0.0, now=10.0)
    assert state.seconds_since_last(now=10.5) == pytest.approx(0.5)


def test_update_marks_present_nodes_alive():
    registry = NodeRegistry()
    registry.update(["/cmd_vel_bridge", "/rosbridge_websocket"], now=10.0)

    names = [n.name for n in registry.snapshot()]
    assert names == ["/cmd_vel_bridge", "/rosbridge_websocket"]
    assert all(n.alive for n in registry.snapshot())


def test_update_marks_missing_nodes_stale_after_timeout():
    registry = NodeRegistry(stale_after_seconds=1.0)
    registry.update(["/cmd_vel_bridge"], now=0.0)
    # node no longer reported present, and enough time has passed
    registry.update([], now=2.0)

    status = registry.snapshot()[0]
    assert status.name == "/cmd_vel_bridge"
    assert status.alive is False


def test_update_keeps_node_alive_within_stale_window():
    registry = NodeRegistry(stale_after_seconds=5.0)
    registry.update(["/cmd_vel_bridge"], now=0.0)
    registry.update([], now=1.0)  # missing from this poll, but within window

    assert registry.snapshot()[0].alive is True


def test_snapshot_is_sorted_by_name():
    registry = NodeRegistry()
    registry.update(["/zzz_node", "/aaa_node"], now=0.0)

    names = [n.name for n in registry.snapshot()]
    assert names == ["/aaa_node", "/zzz_node"]
