import json

import pytest

from navi_localization.pose_composition import IDENTITY, Transform
from navi_localization.tracker import LocalizationTracker


def at(x):
    return Transform(x, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)


def test_starts_off_with_nothing_to_publish():
    t = LocalizationTracker()
    assert t.state == LocalizationTracker.OFF
    assert t.pose_to_publish() is None
    assert t.distance_travelled == 0.0


def test_a_good_pose_makes_it_ok_and_publishes_that_pose():
    t = LocalizationTracker()
    t.on_pose(now=10.0, pose=at(1.0), stamp=9.99, tracking_ok=True)
    assert t.state == LocalizationTracker.OK
    assert t.pose_to_publish() == (at(1.0), 9.99)
    assert t.seconds_since_ok(now=10.5) == pytest.approx(0.5)


def test_distance_accumulates_over_good_poses_only():
    t = LocalizationTracker()
    t.on_pose(10.0, at(0.0), 10.0, True)
    t.on_pose(10.1, at(3.0), 10.1, True)
    t.on_pose(10.2, at(7.0), 10.2, True)
    assert t.distance_travelled == pytest.approx(7.0)
    t.on_pose(10.3, at(100.0), 10.3, False)   # searching: a jump is not travel
    assert t.distance_travelled == pytest.approx(7.0)


def test_searching_freezes_the_last_good_pose_and_its_stamp():
    t = LocalizationTracker()
    t.on_pose(10.0, at(2.0), 10.0, True)
    t.on_pose(10.1, at(2.5), 10.1, False)
    assert t.state == LocalizationTracker.SEARCHING
    assert t.pose_to_publish() == (at(2.0), 10.0)
    assert t.seconds_since_ok(now=13.0) == pytest.approx(3.0)


def test_recovery_returns_to_ok_and_continues_from_the_new_pose():
    t = LocalizationTracker()
    t.on_pose(10.0, at(2.0), 10.0, True)
    t.on_pose(10.1, at(9.0), 10.1, False)
    t.on_pose(10.2, at(2.2), 10.2, True)
    assert t.state == LocalizationTracker.OK
    assert t.pose_to_publish() == (at(2.2), 10.2)
    # The 2.0 -> 2.2 hop while OK counts; the searching jump did not.
    assert t.distance_travelled == pytest.approx(0.2)


def test_silence_becomes_off_after_the_timeout_and_keeps_the_last_pose():
    t = LocalizationTracker(off_after_seconds=2.0)
    t.on_pose(10.0, at(1.0), 10.0, True)
    t.on_tick(now=11.9)
    assert t.state == LocalizationTracker.OK
    t.on_tick(now=12.1)
    assert t.state == LocalizationTracker.OFF
    assert t.pose_to_publish() == (at(1.0), 10.0)


def test_searching_also_times_out_to_off():
    t = LocalizationTracker(off_after_seconds=2.0)
    t.on_pose(10.0, at(1.0), 10.0, True)
    t.on_pose(10.5, at(1.0), 10.5, False)
    t.on_tick(now=13.0)
    assert t.state == LocalizationTracker.OFF


def test_status_json_carries_the_fields_the_ground_station_reads():
    t = LocalizationTracker()
    t.on_pose(10.0, at(0.0), 10.0, True)
    t.on_pose(10.1, at(4.0), 10.1, True)
    data = json.loads(t.status_json(now=10.6))
    assert data == {
        "state": "OK",
        "seconds_since_ok": pytest.approx(0.5),
        "source": "zed_vio",
        "distance_travelled": pytest.approx(4.0),
        "mount_offset_verified": True,
    }


def test_status_json_before_any_pose_reports_off_with_no_age():
    data = json.loads(LocalizationTracker().status_json(now=5.0))
    assert data["state"] == "OFF"
    assert data["seconds_since_ok"] is None
