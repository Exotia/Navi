"""The square the fake localisation walks.

Loaded by path rather than imported: mock/ is not a package, and a plain
`import square_walk` would depend on the working directory.
"""

import importlib.util
import math
import pathlib

import pytest

_PATH = pathlib.Path(__file__).resolve().parent.parent / "mock" / "square_walk.py"
_spec = importlib.util.spec_from_file_location("square_walk", _PATH)
square_walk = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(square_walk)


def test_the_walk_starts_at_the_origin_facing_along_x():
    assert square_walk.square_pose(0.0) == (0.0, 0.0, 0.0)


def test_the_first_leg_runs_along_x_at_the_given_speed():
    x, y, yaw = square_walk.square_pose(4.0, side=4.0, speed=0.5)

    assert x == pytest.approx(2.0)
    assert y == pytest.approx(0.0)
    assert yaw == pytest.approx(0.0)


def test_each_corner_is_a_quarter_turn():
    # side 4 m at 0.5 m/s is 8 s a leg.
    assert square_walk.square_pose(8.0) == pytest.approx((4.0, 0.0, math.pi / 2))
    assert square_walk.square_pose(16.0) == pytest.approx((4.0, 4.0, math.pi))
    assert square_walk.square_pose(24.0) == pytest.approx((0.0, 4.0, -math.pi / 2))


def test_the_square_closes_and_repeats():
    # A closed loop is what makes this fixture useful: whatever is watching
    # the far end can be left running and must keep coming back to the
    # origin, so a drift or an accumulating offset shows up as the picture
    # walking away rather than as a number someone has to read.
    assert square_walk.square_pose(32.0) == pytest.approx((0.0, 0.0, 0.0))
    assert square_walk.square_pose(36.0) == pytest.approx(square_walk.square_pose(4.0))


def test_a_shorter_faster_square_scales_both_ways():
    x, y, yaw = square_walk.square_pose(1.0, side=2.0, speed=1.0)

    assert (x, y, yaw) == pytest.approx((1.0, 0.0, 0.0))


def test_the_odometry_message_is_shaped_the_way_the_ground_station_reads_it():
    from ground_station.models import pose_readout_from_odometry

    message = square_walk.odometry_message(1.5, -2.25, math.pi / 2)

    assert message["header"]["frame_id"] == "map"
    assert message["child_frame_id"] == "base_footprint"
    readout = pose_readout_from_odometry(message)
    assert readout["x"] == pytest.approx(1.5)
    assert readout["y"] == pytest.approx(-2.25)
    assert readout["yaw"] == pytest.approx(math.pi / 2)


def test_the_status_payload_is_what_ros_client_parses():
    import json

    from ground_station.ros_client import RosBridgeClient

    parsed = RosBridgeClient._parse_localization_status(
        square_walk.status_payload("SEARCHING", 4.2, 12.5))

    assert parsed["state"] == "SEARCHING"
    assert parsed["seconds_since_ok"] == 4.2
    assert parsed["source"] == "zed_vio"
    assert parsed["distance_travelled"] == 12.5
    assert parsed["mount_offset_verified"] is True
    assert json.loads(square_walk.status_payload("OK", 0.0, 0.0))["state"] == "OK"
