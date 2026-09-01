"""End-to-end integration test for the site-anchor plan (Task 11,
docs/superpowers/plans/2026-09-01-site-anchor.md §6).

This drives the WHOLE ground-station chain - MainWindow, the SITE drawer,
NavRow, and ros_client - against the same fakes tests/test_main_window.py
already uses (FakeRos/FakeTopic via make_window's factory), with no rover,
no ROS and no camera. Where the plan calls for it, data is fed over the
fake wire as JSON text through the real ground_station.models parsers
(parse_sightings, parse_probe_result), not as pre-built dataclasses, so the
wire format itself is exercised end to end.

The one deliberate rule (§6, Task 11): the final "did the coordinate land
in the right place" assertions re-derive the site->map arithmetic BY HAND
in this file, rather than calling ground_station.site_frame.site_to_map -
so a bug shared between the production code and its own test cannot cancel
out. The only exception is the canvas-click round trip (scenario 5's third
waypoint and §3.9's invariant), where the thing under test IS that
map_to_site and site_to_map compose back to the identity - there is no
"independent" arithmetic for an invariant that says the two are exact
inverses of one another.
"""

import json
import math
import random

import pytest

from ground_station.landmark_table import load_landmark_table
from ground_station.models import (Waypoint, nav_request_json, new_run_id,
                                   parse_probe_result, parse_sightings)
from ground_station.ui import main_window
from tests.test_main_window import (FakeTopic, SITE_EXAMPLE_PATH,
                                     _mode_status, _nav_status,
                                     connected_window, published)

# --- hand-rolled site->map arithmetic, independent of site_frame.py --------
#
# solve_site_to_map, site_to_map, map_to_site and reexpress_at_lock_pose are
# all exercised through the production code paths below; these helpers
# re-derive the same rigid-transform maths from the plan's §3.2 formula
# (p_map = R(yaw) . p_site + (x, y)) so the final assertions are a real
# check against ground truth, not a restatement of the code under test.


def _rotate(x: float, y: float, yaw: float) -> tuple:
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y, s * x + c * y


def _wrap(angle: float) -> float:
    wrapped = (angle + math.pi) % (2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def _forward(gt: tuple, x: float, y: float) -> tuple:
    """site -> map under ground-truth (x0, y0, yaw0), by hand."""
    x0, y0, yaw0 = gt
    rx, ry = _rotate(x, y, yaw0)
    return rx + x0, ry + y0


def _forward_yaw(gt: tuple, yaw: float) -> float:
    return _wrap(gt[2] + yaw)


# --- fake-wire helpers ------------------------------------------------------


def _feed_json_wire(topic_name: str, payload: dict) -> None:
    """Feed `payload` through the FakeTopic named `topic_name` exactly as
    rosbridge would: a std_msgs/String carrying a JSON-encoded `data`
    field. This is what routes through ros_client's real
    subscribe_landmark_sightings / subscribe_probe_result callbacks, which
    call parse_sightings / parse_probe_result themselves - so feeding a
    dict here, not a parsed dataclass, is what exercises the wire format."""
    topic = next(t for t in FakeTopic.instances if t.name == topic_name)
    topic.callback({"data": json.dumps(payload)})


def _feed_pose(x: float, y: float, yaw: float) -> None:
    """Feed a raw nav_msgs/Odometry-shaped message through the fake
    /localization/pose topic. Unlike the JSON-string topics above, this one
    is not std_msgs/String - ros_client.subscribe_localization_pose hands
    the raw message straight to pose_readout_from_odometry."""
    topic = next(t for t in FakeTopic.instances if t.name == "/localization/pose")
    topic.callback({"pose": {"pose": {
        "position": {"x": x, "y": y, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0,
                        "z": math.sin(yaw / 2.0), "w": math.cos(yaw / 2.0)}}}})


def _sighting_entry(landmark_id: str, x: float, y: float, quality: str = "good") -> dict:
    return {"id": landmark_id, "x": x, "y": y, "z": 0.417,
            "n": 60, "spread_m": 0.02, "range_m": 5.0,
            "last_seen_s": 0.1, "quality": quality}


def _sightings_payload(entries: list, phase: str = "running") -> dict:
    return {"stamp_s": 100.0, "phase": phase, "frame_id": "map",
            "dictionary": "DICT_5X5_100", "image_size": [1280, 720],
            "detector_ok": True, "error": None, "sightings": entries}


def _probe_payload(request_id: str, label: str, x: float, y: float,
                   valid_fraction: float = 0.9) -> dict:
    return {"request_id": request_id, "ok": True, "label": label,
            "x": x, "y": y, "z": 0.417, "range_m": 5.0,
            "samples": 40, "valid_fraction": valid_fraction,
            "stamp_s": 10.0, "frame_id": "map", "error": None}


def _load_table(window) -> "LandmarkTable":
    """Load docs/site/landmarks.example.json through the same signal path
    the operator's file dialog uses (SiteCard.table_load_requested ->
    MainWindow._on_site_table_load_requested), and hand back the parsed
    table too, for the ground-truth site coordinates."""
    window.dashboard_page.site_card.table_load_requested.emit(str(SITE_EXAMPLE_PATH))
    return load_landmark_table(SITE_EXAMPLE_PATH)


def _enter_autonomy(window) -> None:
    _mode_status(window, "autonomous")
    _nav_status(window, state="idle")


GT = (10.0, -4.0, 0.3)  # an arbitrary, nontrivial site->map ground truth


# --- scenarios 1-6: the whole chain, solve, lock, typed + canvas waypoints -


def test_full_chain_solves_locks_and_converts_go_with_typed_and_canvas_waypoints(qtbot):
    window = connected_window(qtbot)
    table = _load_table(window)
    assert window.dashboard_page.site_card.table is not None

    entries = [_sighting_entry(lm.id, *_forward(GT, lm.x, lm.y)) for lm in table.landmarks]
    _feed_json_wire("/site/landmark_sightings", _sightings_payload(entries))
    assert window.dashboard_page.site_card.state_pill.text() == "3 OF 3 MEASURED"

    window.dashboard_page.site_card.solve_button.click()
    t = window.dashboard_page.site_card.transform
    assert t is not None
    assert t.x == pytest.approx(GT[0], abs=1e-6)
    assert t.y == pytest.approx(GT[1], abs=1e-6)
    assert t.yaw == pytest.approx(GT[2], abs=1e-6)
    assert t.rms_m < 1e-6

    window.dashboard_page.site_card.lock_button.click()
    assert window._site_locked
    assert window._site_transform == t

    _enter_autonomy(window)
    nav_row = window.dashboard_page.nav_row

    # Typed waypoint 1, with a yaw.
    nav_row.x_input.setText("3.0")
    nav_row.y_input.setText("-1.5")
    nav_row.yaw_input.setText("0.4")
    nav_row.add_button.click()

    # Typed waypoint 2, no yaw.
    nav_row.x_input.setText("1.0")
    nav_row.y_input.setText("2.0")
    nav_row.add_button.click()

    # Waypoint 3 via a click on the plan canvas, in MAP-frame world
    # coordinates (NavMapView.point_clicked's own frame, nav_map_view.py).
    canvas_click_map = (8.0, -6.0)
    nav_row.map_view.point_clicked.emit(*canvas_click_map)

    assert len(nav_row.waypoints) == 3

    nav_row.go_button.click()
    request = published(window, "/nav_request")[-1]
    waypoints = request["waypoints"]
    assert len(waypoints) == 3

    expected_1 = _forward(GT, 3.0, -1.5)
    expected_1_yaw = _forward_yaw(GT, 0.4)
    assert waypoints[0]["x"] == pytest.approx(expected_1[0], abs=1e-6)
    assert waypoints[0]["y"] == pytest.approx(expected_1[1], abs=1e-6)
    assert waypoints[0]["yaw"] == pytest.approx(expected_1_yaw, abs=1e-6)

    expected_2 = _forward(GT, 1.0, 2.0)
    assert waypoints[1]["x"] == pytest.approx(expected_2[0], abs=1e-6)
    assert waypoints[1]["y"] == pytest.approx(expected_2[1], abs=1e-6)
    assert waypoints[1]["yaw"] is None

    # §3.9's invariant: a canvas click is converted back to the operator's
    # frame on the way in (map_to_site) and forward again on the way out
    # (site_to_map at Go) - the round trip must land it exactly where the
    # operator clicked, regardless of where the site origin is.
    assert waypoints[2]["x"] == pytest.approx(canvas_click_map[0], abs=1e-9)
    assert waypoints[2]["y"] == pytest.approx(canvas_click_map[1], abs=1e-9)
    assert waypoints[2]["yaw"] is None


# --- scenario 7: noisy sightings ------------------------------------------


def test_noisy_sightings_recover_the_transform_within_expected_error(qtbot):
    window = connected_window(qtbot)
    table = _load_table(window)

    sigma = 0.05
    rng = random.Random(20260901)
    entries = []
    for lm in table.landmarks:
        mx, my = _forward(GT, lm.x, lm.y)
        entries.append(_sighting_entry(lm.id, mx + rng.gauss(0, sigma), my + rng.gauss(0, sigma)))
    _feed_json_wire("/site/landmark_sightings", _sightings_payload(entries))

    window.dashboard_page.site_card.solve_button.click()
    t = window.dashboard_page.site_card.transform
    assert t is not None
    # rms_m within a factor of 2 of the injected sigma either way.
    assert 0.5 * sigma <= t.rms_m <= 2.0 * sigma

    window.dashboard_page.site_card.lock_button.click()
    _enter_autonomy(window)
    nav_row = window.dashboard_page.nav_row
    nav_row.x_input.setText("3.0")
    nav_row.y_input.setText("-1.5")
    nav_row.add_button.click()
    nav_row.go_button.click()

    waypoint = published(window, "/nav_request")[-1]["waypoints"][0]
    expected = _forward(GT, 3.0, -1.5)
    error = math.hypot(waypoint["x"] - expected[0], waypoint["y"] - expected[1])
    # A few sigma of slack: the fit does not reproduce ground truth exactly
    # under noise, but the Go point must stay close to it, not merely
    # "somewhere in the yard".
    assert error < 10.0 * sigma


# --- scenario 8: one sighting under the wrong id ---------------------------


def test_a_mislabeled_sighting_is_named_and_unticking_it_fixes_the_go(qtbot):
    window = connected_window(qtbot)
    table = _load_table(window)
    by_id = {lm.id: lm for lm in table.landmarks}

    entries = []
    for lm in table.landmarks:
        if lm.id == "52":
            # The operator clicked landmark 53's marker but reported it as
            # 52: the map position is real (landmark 53's), the label is
            # wrong.
            other = by_id["53"]
            mx, my = _forward(GT, other.x, other.y)
        else:
            mx, my = _forward(GT, lm.x, lm.y)
        entries.append(_sighting_entry(lm.id, mx, my))
    _feed_json_wire("/site/landmark_sightings", _sightings_payload(entries))

    card = window.dashboard_page.site_card
    card.solve_button.click()
    bad_transform = card.transform
    assert bad_transform is not None
    assert bad_transform.rms_m > 0.5
    assert bad_transform.worst_id == "52"
    assert "52" in card.detail_label.text()

    def _row(landmark_id):
        for i in range(card.landmark_list.count()):
            item = card.landmark_list.item(i)
            widget = card.landmark_list.itemWidget(item)
            if widget.id == landmark_id:
                return widget
        return None

    _row("52").checkbox.setChecked(False)
    card.solve_button.click()
    good_transform = card.transform
    assert good_transform.rms_m < 1e-6
    assert good_transform.rms_m < bad_transform.rms_m

    card.lock_button.click()
    _enter_autonomy(window)
    nav_row = window.dashboard_page.nav_row
    nav_row.x_input.setText("3.0")
    nav_row.y_input.setText("-1.5")
    nav_row.add_button.click()
    nav_row.go_button.click()

    waypoint = published(window, "/nav_request")[-1]["waypoints"][0]
    expected = _forward(GT, 3.0, -1.5)
    assert waypoint["x"] == pytest.approx(expected[0], abs=1e-6)
    assert waypoint["y"] == pytest.approx(expected[1], abs=1e-6)


# --- scenario 9: stage-2 depth probes instead of ArUco sightings ----------


def test_stage_two_probe_results_produce_the_same_locked_transform_and_go(qtbot):
    window = connected_window(qtbot)
    table = _load_table(window)

    for i, lm in enumerate(table.landmarks):
        mx, my = _forward(GT, lm.x, lm.y)
        _feed_json_wire("/site/probe_result", _probe_payload(f"p-{i}", lm.id, mx, my))

    card = window.dashboard_page.site_card
    assert card.state_pill.text() == "3 OF 3 MEASURED"
    card.solve_button.click()
    t = card.transform
    assert t is not None
    assert t.x == pytest.approx(GT[0], abs=1e-6)
    assert t.y == pytest.approx(GT[1], abs=1e-6)
    assert t.yaw == pytest.approx(GT[2], abs=1e-6)
    assert t.rms_m < 1e-6

    card.lock_button.click()
    _enter_autonomy(window)
    nav_row = window.dashboard_page.nav_row
    nav_row.x_input.setText("3.0")
    nav_row.y_input.setText("-1.5")
    nav_row.add_button.click()
    nav_row.go_button.click()

    waypoint = published(window, "/nav_request")[-1]["waypoints"][0]
    expected = _forward(GT, 3.0, -1.5)
    assert waypoint["x"] == pytest.approx(expected[0], abs=1e-6)
    assert waypoint["y"] == pytest.approx(expected[1], abs=1e-6)


# --- scenario 10: no anchoring at all -> byte-identical JSON --------------


def test_no_anchoring_produces_byte_identical_json_to_a_plain_window(qtbot, monkeypatch):
    monkeypatch.setattr(main_window, "time", lambda: 1234.5)

    waypoints = [Waypoint(3.0, -1.5, 0.4), Waypoint(1.0, 2.0, None)]

    window_a = connected_window(qtbot)
    assert not window_a.dashboard_page.site_card.isVisibleTo(window_a)
    window_a.dashboard_page.nav_row.go_requested.emit(waypoints)
    raw_a = [message["data"] for name, message in FakeTopic.call_log
            if name == "/nav_request"][-1]

    # make_window (via connected_window) clears FakeTopic.call_log, so
    # window_a's raw string is captured above before window_b exists.
    window_b = connected_window(qtbot)
    assert not window_b.dashboard_page.site_card.isVisibleTo(window_b)
    window_b.dashboard_page.nav_row.go_requested.emit(waypoints)
    raw_b = [message["data"] for name, message in FakeTopic.call_log
            if name == "/nav_request"][-1]

    assert raw_a == raw_b

    expected_raw = nav_request_json("go", waypoints, new_run_id(1234.5))
    assert raw_a == expected_raw
    assert json.loads(raw_a) == {
        "action": "go",
        "run_id": new_run_id(1234.5),
        "frame_id": "map",
        "waypoints": [
            {"x": 3.0, "y": -1.5, "yaw": 0.4},
            {"x": 1.0, "y": 2.0, "yaw": None},
        ],
    }


# --- §3.10: surviving the wrapper restart, driven end to end -------------


def test_camera_restart_reexpression_lands_the_same_rover_relative_waypoint(qtbot):
    window = connected_window(qtbot)
    table = _load_table(window)

    gt = (6.0, 2.0, -0.4)
    entries = [_sighting_entry(lm.id, *_forward(gt, lm.x, lm.y)) for lm in table.landmarks]
    _feed_json_wire("/site/landmark_sightings", _sightings_payload(entries))
    window.dashboard_page.site_card.solve_button.click()
    assert window.dashboard_page.site_card.transform.rms_m < 1e-6

    # The rover's own /localization/pose in the OLD map frame, at the
    # moment of Lock - fed over the fake wire before Lock is pressed, the
    # same order the real system observes it in.
    pose = (1.2, -0.6, 0.15)
    _feed_pose(*pose)

    window.dashboard_page.site_card.lock_button.click()
    assert window._site_locked
    assert window._site_lock_pose == {"x": pose[0], "y": pose[1], "yaw": pytest.approx(pose[2])}

    _enter_autonomy(window)
    nav_row = window.dashboard_page.nav_row
    nav_row.x_input.setText("4.0")
    nav_row.y_input.setText("-2.5")
    nav_row.add_button.click()

    nav_row.go_button.click()
    before = published(window, "/nav_request")[-1]["waypoints"][0]
    map_before = (before["x"], before["y"])

    # The wrapper restart: the rover has not moved, so the new map frame is
    # born exactly at `pose` in the old one.
    window.dashboard_page.site_card.camera_restarted.emit()
    assert "re-expressed" in window.dashboard_page.site_card.state_pill.text().lower()

    nav_row.go_button.click()
    after = published(window, "/nav_request")[-1]["waypoints"][0]
    map_after = (after["x"], after["y"])

    # Independent check (no site_frame call): the new frame's origin IS
    # the rover, at yaw 0, at the instant of restart - so the same
    # site-frame waypoint's position relative to the rover must be
    # identical before and after. Rotate map_before into the rover's
    # old-frame heading and subtract its old-frame position by hand; that
    # must equal map_after exactly.
    px, py, pyaw = pose
    dx, dy = map_before[0] - px, map_before[1] - py
    c, s = math.cos(pyaw), math.sin(pyaw)
    rover_relative_before = (c * dx + s * dy, -s * dx + c * dy)
    assert map_after[0] == pytest.approx(rover_relative_before[0], abs=1e-9)
    assert map_after[1] == pytest.approx(rover_relative_before[1], abs=1e-9)
