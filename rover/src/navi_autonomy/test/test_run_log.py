"""The ride diary: one file, truncated per ride, decisions with timestamps.
Pure - the clock is injected, no ROS."""

import time

from navi_autonomy.run_log import RunLog


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def make_log(tmp_path):
    clock = Clock()
    path = tmp_path / "ride.log"
    return RunLog(str(path), clock=clock,
                  walltime=lambda: time.localtime(0)), clock, path


def test_a_new_ride_truncates_the_old_one(tmp_path):
    log, clock, path = make_log(tmp_path)
    log.start("gs-1", [(3.0, -1.5, None)])
    log.event("go", "accepted with 1 waypoint(s)")
    log.start("gs-2", [(8.0, 0.0, 1.57)])
    text = path.read_text()
    assert "gs-1" not in text and "go" not in text
    assert "ride gs-2" in text
    assert "waypoint 1/1: (8.00, 0.00) yaw 1.57" in text


def test_events_carry_the_seconds_since_the_ride_started(tmp_path):
    log, clock, path = make_log(tmp_path)
    log.start("gs-1", [(3.0, -1.5, None)])
    clock.t += 12.4
    log.event("nav2_goal", "FAILED - Nav2 goal ended with status 6")
    text = path.read_text()
    assert "waypoint 1/1: (3.00, -1.50) yaw free" in text
    assert "+   12.4s  nav2_goal          FAILED - Nav2 goal ended with status 6" in text


def test_a_throttled_tag_logs_once_per_window_and_others_pass(tmp_path):
    log, clock, path = make_log(tmp_path)
    log.start("gs-1", [(3.0, -1.5, None)])
    log.event("feedback", "5.00 m", throttle_s=2.0)
    clock.t += 0.5
    log.event("feedback", "4.90 m", throttle_s=2.0)   # inside the window
    log.event("run_state", "running")                  # different tag: passes
    clock.t += 2.0
    log.event("feedback", "4.00 m", throttle_s=2.0)
    text = path.read_text()
    assert text.count("feedback") == 2
    assert "running" in text


def test_events_without_a_ride_are_dropped_not_crashed(tmp_path):
    log, clock, path = make_log(tmp_path)
    log.event("mode", "manual (startup)")     # no ride open, no file
    assert not path.exists()


def test_an_unwritable_path_never_raises(tmp_path):
    log = RunLog(str(tmp_path / "no" / "such" / "dir" / "ride.log"),
                 clock=Clock(), walltime=lambda: time.localtime(0))
    log.start("gs-1", [(0.0, 0.0, None)])
    log.event("go", "accepted")               # both must be silent no-ops
