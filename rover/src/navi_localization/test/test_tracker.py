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
    t.on_pose(10.1, at(0.3), 10.1, True)
    t.on_pose(10.2, at(0.7), 10.2, True)
    assert t.distance_travelled == pytest.approx(0.7)
    t.on_pose(10.3, at(100.0), 10.3, False)   # searching: a jump is not travel
    assert t.distance_travelled == pytest.approx(0.7)


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
    t.on_pose(10.1, at(0.4), 10.1, True)
    data = json.loads(t.status_json(now=10.6))
    assert data == {
        "state": "OK",
        "reason": "",
        "seconds_since_ok": pytest.approx(0.5),
        "source": "zed_vio",
        "distance_travelled": pytest.approx(0.4),
        "mount_offset_verified": True,
    }


def test_status_json_before_any_pose_reports_off_with_no_age():
    data = json.loads(LocalizationTracker().status_json(now=5.0))
    assert data["state"] == "OFF"
    assert data["seconds_since_ok"] is None


def test_a_pose_jump_while_the_zed_says_ok_is_searching_and_not_published():
    # 2026-08-30: a restart with the desk under a metre away made the SDK
    # report OK at z = -1764 m and 4.8 km travelled within seconds.
    t = LocalizationTracker()
    t.on_pose(10.0, at(0.0), 10.0, True)
    t.on_pose(10.07, at(0.05), 10.07, True)
    t.on_pose(10.13, Transform(0.1, 0.0, -1764.0, 0.0, 0.0, 0.0, 1.0), 10.13, True)
    assert t.state == LocalizationTracker.SEARCHING
    assert t.reason == "pose jump"
    assert t.pose_to_publish() == (at(0.05), 10.07)         # last good pose, frozen
    assert t.distance_travelled == pytest.approx(0.05)      # the jump is not driving
    assert json.loads(t.status_json(10.2))["reason"] == "pose jump"


def test_a_teleport_in_the_plane_is_also_a_jump():
    t = LocalizationTracker()
    t.on_pose(10.0, at(0.0), 10.0, True)
    t.on_pose(10.07, at(30.0), 10.07, True)                  # 30 m in 70 ms
    assert t.state == LocalizationTracker.SEARCHING
    assert t.distance_travelled == 0.0


def test_consistent_poses_after_a_jump_reanchor_without_counting_the_jump():
    # The SDK's own tracking reset puts the rover back near the origin: a
    # jump against the old anchor, but every pose after it agrees with the
    # first one. After REACQUIRE_POSES of those the tracker follows again.
    from navi_localization.tracker import REACQUIRE_POSES
    t = LocalizationTracker()
    t.on_pose(10.0, at(40.0), 10.0, True)
    t.on_pose(10.07, at(40.05), 10.07, True)
    now = 10.07
    for i in range(REACQUIRE_POSES):
        now += 0.067
        t.on_pose(now, at(0.0 + 0.01 * i), now, True)
        if i < REACQUIRE_POSES - 1:
            assert t.state == LocalizationTracker.SEARCHING
    assert t.state == LocalizationTracker.OK
    assert t.pose_to_publish()[0].x == pytest.approx(0.01 * (REACQUIRE_POSES - 1))
    assert t.distance_travelled == pytest.approx(0.05)      # 40 m relocation not counted
    now += 0.067
    t.on_pose(now, at(0.2), now, True)
    assert t.state == LocalizationTracker.OK               # and it keeps following


def test_a_wandering_estimate_never_reanchors():
    # Poses that disagree with each other as well as with the anchor stay
    # SEARCHING however many arrive.
    t = LocalizationTracker()
    t.on_pose(10.0, at(0.0), 10.0, True)
    now = 10.0
    for i in range(40):
        now += 0.067
        t.on_pose(now, at(100.0 + 10.0 * i), now, True)
        assert t.state == LocalizationTracker.SEARCHING


def test_normal_driving_at_one_metre_per_second_is_never_a_jump():
    t = LocalizationTracker()
    now, x = 10.0, 0.0
    t.on_pose(now, at(x), now, True)
    for _ in range(150):
        now += 0.067
        x += 0.067
        t.on_pose(now, at(x), now, True)
        assert t.state == LocalizationTracker.OK
    assert t.distance_travelled == pytest.approx(150 * 0.067)
