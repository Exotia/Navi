import pytest
from ground_station.models import DriveCommandTracker, NodeRegistry


def test_ingest_stores_latest_sample():
    state = DriveCommandTracker()
    state.ingest(0.4, -0.05, 0.1, now=10.0)

    assert state.latest.linear_x == 0.4
    assert state.latest.linear_y == -0.05
    assert state.latest.angular_z == 0.1
    assert state.latest.received_at == 10.0


def test_rate_hz_is_zero_with_fewer_than_two_samples():
    state = DriveCommandTracker()
    assert state.rate_hz == 0.0
    state.ingest(0.0, 0.0, 0.0, now=10.0)
    assert state.rate_hz == 0.0


def test_rate_hz_computes_from_recent_samples():
    state = DriveCommandTracker(rate_window_seconds=2.0)
    # 5 samples spaced 0.1s apart -> 10 Hz
    for i in range(5):
        state.ingest(0.0, 0.0, 0.0, now=10.0 + i * 0.1)

    assert state.rate_hz == pytest.approx(10.0, rel=0.05)


def test_rate_hz_drops_samples_outside_window():
    state = DriveCommandTracker(rate_window_seconds=1.0)
    state.ingest(0.0, 0.0, 0.0, now=0.0)
    state.ingest(0.0, 0.0, 0.0, now=5.0)
    state.ingest(0.0, 0.0, 0.0, now=5.5)

    # only the last two samples (5.0, 5.5) are within the 1s window
    assert state.rate_hz == pytest.approx(2.0, rel=0.05)


def test_seconds_since_last_none_before_any_sample():
    state = DriveCommandTracker()
    assert state.seconds_since_last(now=10.0) is None


def test_seconds_since_last_computes_elapsed_time():
    state = DriveCommandTracker()
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


import math

from ground_station.models import pose_readout_from_odometry, yaw_from_quaternion


def test_yaw_of_the_identity_quaternion_is_zero():
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == 0.0


def test_yaw_of_a_quarter_turn_about_z():
    yaw = yaw_from_quaternion(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))

    assert yaw == pytest.approx(math.pi / 2)


def test_yaw_is_signed_so_a_right_turn_reads_negative():
    yaw = yaw_from_quaternion(0.0, 0.0, math.sin(-math.pi / 6), math.cos(-math.pi / 6))

    assert yaw == pytest.approx(-math.pi / 3)


def test_pose_readout_pulls_x_y_and_yaw_out_of_an_odometry_message():
    message = {
        "header": {"frame_id": "map"},
        "child_frame_id": "base_footprint",
        "pose": {"pose": {
            "position": {"x": 3.25, "y": -1.5, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0,
                            "z": math.sin(math.pi / 4), "w": math.cos(math.pi / 4)},
        }},
    }

    readout = pose_readout_from_odometry(message)

    assert readout["x"] == pytest.approx(3.25)
    assert readout["y"] == pytest.approx(-1.5)
    assert readout["yaw"] == pytest.approx(math.pi / 2)


def test_pose_readout_of_a_truncated_message_reads_as_the_origin_not_a_crash():
    # rosbridge hands over whatever JSON arrived. A malformed message must
    # not take the GUI thread down mid-drive; a readout at the origin is
    # visibly wrong and recoverable, an exception in a Qt slot is not.
    assert pose_readout_from_odometry({}) == {"x": 0.0, "y": 0.0, "yaw": 0.0}


import json

from ground_station.models import MapState, map_command_json, parse_map_status


def test_map_status_parses_every_field_with_defaults():
    state = parse_map_status('{"resolution": 0.05, "cells_seen": 12, "extent_m": [3.5, 2.0],'
                             ' "tiles": 2, "loaded": null, "maps": ["b", "a"], "last_command": null}')
    assert state == MapState(cells_seen=12, extent_m=(3.5, 2.0), tiles=2, loaded=None,
                             maps=["b", "a"], last_command=None)
    assert parse_map_status('{}') == MapState(cells_seen=0, extent_m=(0.0, 0.0), tiles=0,
                                              loaded=None, maps=[], last_command=None)


def test_map_status_that_is_not_json_is_none():
    assert parse_map_status('nope') is None
    assert parse_map_status('[1,2]') is None


def test_map_status_falls_back_to_defaults_on_type_malformed_fields():
    # rosbridge hands the payload over as whatever JSON arrived; a field of
    # the wrong shape must fall back to its default, not blow up the poll.
    state = parse_map_status(json.dumps({
        "extent_m": "not a list",
        "cells_seen": "twelve",
        "tiles": [1, 2],
        "maps": {"a": 1},
        "loaded": None,
        "last_command": None,
    }))
    assert state == MapState(cells_seen=0, extent_m=(0.0, 0.0), tiles=0,
                             loaded=None, maps=[], last_command=None)

    # A one-element extent is malformed the same way a non-list one is.
    short_extent = parse_map_status(json.dumps({"extent_m": [3.5]}))
    assert short_extent.extent_m == (0.0, 0.0)

    # A non-numeric entry inside an otherwise well-typed extent list.
    bad_entry = parse_map_status(json.dumps({"extent_m": ["nope", 2.0]}))
    assert bad_entry.extent_m == (0.0, 0.0)

    # maps as a plain string (iterable, but not a list of names) must not
    # be exploded into one-character map names.
    string_maps = parse_map_status(json.dumps({"maps": "yard"}))
    assert string_maps.maps == []


def test_map_commands_are_the_json_the_rover_reads():
    assert json.loads(map_command_json("save", "yard")) == {"action": "save", "name": "yard"}
    assert json.loads(map_command_json("clear")) == {"action": "clear"}


from ground_station.models import (DriveState, parse_drive_status,
                                    drive_command_json)


def test_parse_drive_status_reads_the_fields():
    payload = json.dumps({"connected": True, "lease": True,
                          "coordinator_state": 3, "deadman_active": False,
                          "twist_age_s": 0.1, "last_action": "manual",
                          "last_error": None})
    state = parse_drive_status(payload)
    assert state.connected is True and state.lease is True
    assert state.coordinator_state == "Manual"
    assert state.deadman_active is False and state.twist_age_s == 0.1


def test_parse_drive_status_tolerates_garbage():
    assert parse_drive_status("{not json") is None
    assert parse_drive_status(json.dumps([1, 2])) is None
    # wrong field types fall back, they do not raise
    state = parse_drive_status(json.dumps({"connected": "yes",
                                           "coordinator_state": "nonsense"}))
    assert state.connected is False
    assert state.coordinator_state is None


def test_drive_command_json_round_trips():
    assert json.loads(drive_command_json("stop")) == {"action": "stop"}
