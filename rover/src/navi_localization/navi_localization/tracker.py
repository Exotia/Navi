"""The OK / SEARCHING / OFF state of the localisation, kept apart from ROS.

Three rules, all in the name of never inventing a position:

- While the ZED reports tracking OK, the latest pose is the pose.
- While it reports anything else (SEARCHING), the last good pose is
  republished with its *original* stamp, so any consumer that looks at the
  stamp sees a pose that stopped ageing. Nothing is extrapolated.
- When no pose at all has arrived for `off_after_seconds`, the state is OFF:
  the wrapper is dead or not started. The last good pose is still offered,
  still with its old stamp, for the same reason.

Distance travelled is summed between consecutive OK poses, skipping any
searching poses entirely: recovery is measured from the last good pose to
the newly-acquired one, not from wherever the tracker wandered while
searching, so a jump during a search is never counted as driving.
"""

import json

from navi_localization.pose_composition import (
    MOUNT_OFFSET_VERIFIED, Transform, translation_distance)


class LocalizationTracker:
    OK = "OK"
    SEARCHING = "SEARCHING"
    OFF = "OFF"

    def __init__(self, off_after_seconds: float = 2.0) -> None:
        self._off_after = off_after_seconds
        self._state = self.OFF
        self._last_message_at: float | None = None
        self._last_ok_at: float | None = None
        self._good_pose: tuple[Transform, float] | None = None
        self._previous_ok_pose: Transform | None = None
        self._distance = 0.0

    @property
    def state(self) -> str:
        return self._state

    @property
    def distance_travelled(self) -> float:
        return self._distance

    def on_pose(self, now: float, pose: Transform, stamp: float, tracking_ok: bool) -> None:
        self._last_message_at = now
        if tracking_ok:
            if self._previous_ok_pose is not None:
                self._distance += translation_distance(self._previous_ok_pose, pose)
            self._previous_ok_pose = pose
            self._good_pose = (pose, stamp)
            self._last_ok_at = now
            self._state = self.OK
        else:
            # Do not clear _previous_ok_pose here: it is the anchor for the
            # distance calculation on recovery, skipping the searching jump
            # entirely rather than measuring from wherever tracking landed.
            self._state = self.SEARCHING

    def on_tick(self, now: float) -> None:
        if self._last_message_at is None:
            return
        if now - self._last_message_at > self._off_after:
            self._state = self.OFF
            self._previous_ok_pose = None

    def seconds_since_ok(self, now: float) -> float | None:
        if self._last_ok_at is None:
            return None
        return now - self._last_ok_at

    def pose_to_publish(self) -> tuple[Transform, float] | None:
        return self._good_pose

    def status_json(self, now: float) -> str:
        return json.dumps({
            "state": self._state,
            "seconds_since_ok": self.seconds_since_ok(now),
            "source": "zed_vio",
            "distance_travelled": self._distance,
            "mount_offset_verified": MOUNT_OFFSET_VERIFIED,
        })
