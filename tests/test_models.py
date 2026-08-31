import json

import pytest
from ground_station.models import (DriveCommandTracker, ModeState, NodeRegistry,
                                   estop_request_json, is_stick_deflected,
                                   may_publish_manual_twist,
                                   may_publish_takeover_twist,
                                   mode_request_json, parse_mode_status)


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


def test_parse_drive_status_rejects_json_booleans_in_numeric_fields():
    # JSON booleans (True/False) must not pass int/float type guards
    # (Python bool is a subclass of int, so bare isinstance(x, int) lets True/False through)
    state = parse_drive_status(json.dumps({"twist_age_s": True, "coordinator_state": True}))
    assert state.twist_age_s is None
    assert state.coordinator_state is None


def test_parse_mode_status_reads_every_field():
    state = parse_mode_status(json.dumps({
        "mode": "autonomous", "reason": "mode request",
        "source": "/autonomy_twist", "deadman_active": False,
        "estop_latched": False, "localization_state": "OK",
        "source_age_s": 0.12}))
    assert state == ModeState(mode="autonomous", reason="mode request",
                              source="/autonomy_twist", deadman_active=False,
                              estop_latched=False, localization_state="OK",
                              source_age_s=0.12)


def test_parse_mode_status_defaults_every_missing_field():
    state = parse_mode_status("{}")
    assert state.mode == ""
    assert state.reason == ""
    assert state.source is None
    assert state.deadman_active is False
    assert state.estop_latched is False
    assert state.localization_state is None
    assert state.source_age_s is None


def test_parse_mode_status_returns_none_for_rubbish():
    assert parse_mode_status("{not json") is None
    assert parse_mode_status("[1, 2]") is None


def test_parse_mode_status_ignores_fields_of_the_wrong_type():
    state = parse_mode_status(json.dumps({
        "mode": "manual", "source": 7, "source_age_s": "soon",
        "localization_state": ["OK"]}))
    assert state.source is None
    assert state.source_age_s is None
    assert state.localization_state is None


def test_manual_twist_is_published_in_manual_and_semi_auto_only():
    def at(mode):
        return parse_mode_status(json.dumps({"mode": mode}))
    assert may_publish_manual_twist(at("manual")) is True
    assert may_publish_manual_twist(at("semi_auto")) is True
    assert may_publish_manual_twist(at("autonomous")) is False
    assert may_publish_manual_twist(at("estop")) is False


def test_an_unknown_mode_is_treated_as_not_driving():
    assert may_publish_manual_twist(parse_mode_status('{"mode": "turbo"}')) is False


def test_no_mode_status_at_all_still_publishes():
    # No supervisor on that rover: publishing is what it needs, and a rover
    # that has one answers within half a second of the subscription.
    assert may_publish_manual_twist(None) is True


def test_a_deflected_stick_may_still_be_published_in_autonomous_only():
    def at(mode):
        return parse_mode_status(json.dumps({"mode": mode}))
    assert may_publish_takeover_twist(at("autonomous")) is True
    assert may_publish_takeover_twist(at("estop")) is False
    assert may_publish_takeover_twist(at("manual")) is False
    assert may_publish_takeover_twist(parse_mode_status('{"mode": "turbo"}')) is False
    assert may_publish_takeover_twist(None) is False


def test_a_centred_stick_is_not_deflected_but_the_smallest_output_is():
    # gamepad_input.DEADZONE (0.1) has already been applied per axis before
    # scaling, so a centred stick is exactly zero and the smallest thing
    # the reader can emit is DEADZONE * MAX_LINEAR_SPEED = 0.005.
    assert is_stick_deflected((0.0, 0.0, 0.0)) is False
    assert is_stick_deflected((0.005, 0.0, 0.0)) is True
    assert is_stick_deflected((0.0, 0.0, 0.01)) is True


def test_mode_and_estop_request_json():
    assert json.loads(mode_request_json("manual")) == {"mode": "manual"}
    assert json.loads(estop_request_json("ground station STOP")) == {
        "reason": "ground station STOP"}


from ground_station.models import (NavStatus, PathSummary, ViewTransform, Waypoint,
                                   WaypointList, nav_request_json, new_run_id,
                                   parse_nav_status, parse_path_summary,
                                   parse_waypoint_text)


def test_nav_request_go_carries_every_waypoint_and_the_run_id():
    payload = json.loads(nav_request_json(
        "go", [Waypoint(3.0, -1.5), Waypoint(8.0, -1.5, yaw=0.0)], run_id="gs-1"))
    assert payload == {"action": "go", "run_id": "gs-1", "frame_id": "map",
                       "waypoints": [{"x": 3.0, "y": -1.5, "yaw": None},
                                     {"x": 8.0, "y": -1.5, "yaw": 0.0}]}


def test_nav_request_pause_carries_the_run_id_and_no_waypoints():
    payload = json.loads(nav_request_json("pause", run_id="gs-1"))
    assert payload == {"action": "pause", "run_id": "gs-1", "frame_id": "map",
                       "waypoints": []}


def test_run_id_is_derived_from_the_clock_so_it_is_reproducible_in_a_test():
    assert new_run_id(1756633200.123) == "gs-1756633200123"


def test_parse_nav_status_reads_every_field():
    status = parse_nav_status(json.dumps({
        "state": "running", "run_id": "gs-1", "waypoint_index": 1,
        "waypoint_count": 3, "distance_remaining_m": 12.4, "eta_s": 248.0,
        "error": "", "mode": "autonomous", "coordinator_state": "Autonomous",
        "stamp_s": 1830.5}))
    assert status == NavStatus(state="running", run_id="gs-1", waypoint_index=1,
                               waypoint_count=3, distance_remaining_m=12.4,
                               eta_s=248.0, error="", mode="autonomous",
                               coordinator_state="Autonomous")


def test_parse_nav_status_defaults_every_missing_field_rather_than_raising():
    status = parse_nav_status(json.dumps({"state": "idle"}))
    assert status.waypoint_index is None and status.waypoint_count == 0
    assert status.distance_remaining_m is None and status.eta_s is None
    assert status.error == "" and status.mode is None


def test_parse_nav_status_rejects_a_state_this_build_does_not_know():
    # An unknown state is shown verbatim, not guessed at - the rover is the
    # authority on its own states, exactly as the mode chip treats modes.
    assert parse_nav_status(json.dumps({"state": "reticulating"})).state == "reticulating"


def test_parse_nav_status_survives_rubbish():
    assert parse_nav_status("not json") is None
    assert parse_nav_status(json.dumps([1, 2, 3])) is None


def test_parse_path_summary_reads_points_and_waypoints():
    summary = parse_path_summary(json.dumps({
        "run_id": "gs-1", "frame_id": "map", "points": [[0.0, 0.0], [1.0, 2.0]],
        "waypoints": [[1.0, 2.0]], "length_m": 2.236, "source_points": 941}))
    assert summary == PathSummary(run_id="gs-1", frame_id="map",
                                  points=[(0.0, 0.0), (1.0, 2.0)],
                                  waypoints=[(1.0, 2.0)], length_m=2.236,
                                  source_points=941)


def test_parse_path_summary_drops_malformed_points_instead_of_raising():
    summary = parse_path_summary(json.dumps(
        {"points": [[0.0, 0.0], [1.0], "x", [2.0, 3.0]]}))
    assert summary.points == [(0.0, 0.0), (2.0, 3.0)]


def test_an_empty_point_list_is_a_summary_that_clears_the_drawing():
    assert parse_path_summary(json.dumps({"points": []})).points == []


def test_parse_waypoint_text_accepts_plain_numbers():
    waypoint, error = parse_waypoint_text("3.0", "-1.5")
    assert waypoint == Waypoint(3.0, -1.5, None) and error == ""


def test_parse_waypoint_text_refuses_non_numbers_with_a_reason():
    waypoint, error = parse_waypoint_text("three", "-1.5")
    assert waypoint is None and "x" in error


def test_parse_waypoint_text_refuses_a_point_further_than_the_map_can_reach():
    waypoint, error = parse_waypoint_text("500", "0")
    assert waypoint is None and "60" in error


def test_waypoint_list_adds_removes_reorders_and_clears():
    waypoints = WaypointList()
    waypoints.add(Waypoint(1.0, 1.0))
    waypoints.add(Waypoint(2.0, 2.0))
    waypoints.add(Waypoint(3.0, 3.0))
    waypoints.move_up(2)
    assert [w.x for w in waypoints.items] == [1.0, 3.0, 2.0]
    waypoints.remove(0)
    assert [w.x for w in waypoints.items] == [3.0, 2.0]
    waypoints.clear()
    assert len(waypoints) == 0


def test_waypoint_list_ignores_a_reorder_off_either_end():
    waypoints = WaypointList()
    waypoints.add(Waypoint(1.0, 1.0))
    waypoints.move_up(0)
    waypoints.move_down(0)
    assert [w.x for w in waypoints.items] == [1.0]


def test_view_transform_round_trips_a_click_back_to_the_world():
    view = ViewTransform(centre_x=0.0, centre_y=0.0, metres_per_pixel=0.05,
                         width_px=400, height_px=300)
    assert view.to_pixel(0.0, 0.0) == (200.0, 150.0)
    x, y = view.to_world(*view.to_pixel(3.0, -1.5))
    assert abs(x - 3.0) < 1e-9 and abs(y + 1.5) < 1e-9


def test_view_transform_puts_world_x_up_the_screen_and_world_y_to_the_left():
    view = ViewTransform(0.0, 0.0, 0.05, 400, 300)
    ahead_px = view.to_pixel(1.0, 0.0)
    left_px = view.to_pixel(0.0, 1.0)
    assert ahead_px[1] < 150.0        # +x is up: smaller pixel y
    assert left_px[0] < 200.0         # +y is left: smaller pixel x


def test_view_transform_zoom_keeps_the_centre_still():
    view = ViewTransform(4.0, -2.0, 0.05, 400, 300).zoomed(2.0)
    assert view.metres_per_pixel == 0.1
    assert view.to_pixel(4.0, -2.0) == (200.0, 150.0)
