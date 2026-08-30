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

A fourth rule, from 2026-08-30: the ZED can report OK while its estimate
has blown up (a restart with the desk under a metre away put the rover at
z = -1764 m and 4.8 km "travelled" within seconds). A pose that is
implausible - more than `MAX_JUMP_M` from the previous good pose, faster
than `MAX_SPEED_MPS`, or further than `MAX_ABS_Z_M` from the start height
- is treated as SEARCHING with reason "pose jump", never published, never
counted. The tracker re-anchors only once `REACQUIRE_POSES` consecutive
poses agree with each other, so a legitimate relocalisation (the SDK's
own tracking reset) is accepted after about a second.
"""

import json

from navi_localization.pose_composition import (
    MOUNT_OFFSET_VERIFIED, Transform, translation_distance)


MAX_JUMP_M = 2.0          # between consecutive OK poses (15 Hz -> 30 m/s)
MAX_SPEED_MPS = 5.0       # the rover does 1 m/s
MAX_ABS_Z_M = 20.0        # the yard is flat; the stairwell is 5 m
REACQUIRE_POSES = 15      # ~1 s of self-consistent poses before re-anchoring


class LocalizationTracker:
    OK = "OK"
    SEARCHING = "SEARCHING"
    OFF = "OFF"

    def __init__(self, off_after_seconds: float = 2.0) -> None:
        self._off_after = off_after_seconds
        self._state = self.OFF
        self._reason = ""
        self._last_message_at: float | None = None
        self._last_ok_at: float | None = None
        self._good_pose: tuple[Transform, float] | None = None
        self._previous_ok_pose: Transform | None = None
        self._distance = 0.0
        # Re-anchoring after a rejected jump: the candidate pose and how
        # many consecutive poses have agreed with it.
        self._candidate: Transform | None = None
        self._agreeing = 0

    @property
    def reason(self) -> str:
        return self._reason

    @property
    def state(self) -> str:
        return self._state

    @property
    def distance_travelled(self) -> float:
        return self._distance

    def on_pose(self, now: float, pose: Transform, stamp: float, tracking_ok: bool) -> None:
        self._last_message_at = now
        if not tracking_ok:
            # Do not clear _previous_ok_pose here: it is the anchor for the
            # distance calculation on recovery, skipping the searching jump
            # entirely rather than measuring from wherever tracking landed.
            self._state = self.SEARCHING
            self._reason = ""
            self._candidate = None
            self._agreeing = 0
            return
        if self._implausible(now, pose):
            self._state = self.SEARCHING
            self._reason = "pose jump"
            return
        if self._previous_ok_pose is not None and self._candidate is None:
            self._distance += translation_distance(self._previous_ok_pose, pose)
        # A re-anchor (candidate accepted) adds no distance: the jump to the
        # new anchor is the SDK relocating, not the rover driving.
        self._candidate = None
        self._agreeing = 0
        self._previous_ok_pose = pose
        self._good_pose = (pose, stamp)
        self._last_ok_at = now
        self._state = self.OK
        self._reason = ""

    def _implausible(self, now: float, pose: Transform) -> bool:
        """True when `pose` cannot follow the last good pose, unless enough
        consecutive poses have agreed with each other to re-anchor."""
        if abs(pose.z) > MAX_ABS_Z_M:
            self._candidate = None
            return True
        anchor = self._previous_ok_pose
        if anchor is None:
            return False
        jump = translation_distance(anchor, pose)
        dt = now - self._last_ok_at if self._last_ok_at is not None else None
        too_fast = dt is not None and dt > 0 and jump / dt > MAX_SPEED_MPS
        if jump <= MAX_JUMP_M and not too_fast:
            return False
        # Implausible against the anchor. Consistent with the candidate?
        if self._candidate is not None and translation_distance(self._candidate, pose) <= MAX_JUMP_M:
            self._agreeing += 1
            if self._agreeing >= REACQUIRE_POSES:
                return False            # re-anchor here
        else:
            self._candidate = pose
            self._agreeing = 1
        return True

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
            "reason": self._reason,
            "seconds_since_ok": self.seconds_since_ok(now),
            "source": "zed_vio",
            "distance_travelled": self._distance,
            "mount_offset_verified": MOUNT_OFFSET_VERIFIED,
        })
