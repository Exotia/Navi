"""The ride's diary: every decision the autonomy makes, in one file.

The operator's request (2026-09-01 night session, verbatim intent): when a
run goes wrong, the why should be readable from one place - what was asked,
what was sent to Nav2 and the coordinator, what came back, and why the run
ended. One file per ride: `start()` truncates it, so the file always holds
the LAST ride and nothing older.

Pure besides the file handle - the clock is injected so the tests can pin
the timestamps. goal_relay owns the wiring; nothing here imports ROS.
"""

import time


class RunLog:

    def __init__(self, path, clock=time.monotonic, walltime=time.localtime):
        self._path = path
        self._clock = clock
        self._walltime = walltime
        self._file = None
        self._t0 = None
        self._last_at = {}       # tag -> monotonic stamp, for throttling

    def start(self, run_id, waypoints):
        """A new ride: truncate the file and write the header. Never raises -
        a full disk must not stop the mission it was meant to explain."""
        self._close()
        self._last_at = {}
        try:
            self._file = open(self._path, "w", buffering=1)
            self._t0 = self._clock()
            stamp = time.strftime("%Y-%m-%d %H:%M:%S", self._walltime())
            self._file.write(f"ride {run_id} started {stamp}\n")
            for i, (x, y, yaw) in enumerate(waypoints):
                yaw_txt = "free" if yaw is None else f"{yaw:.2f}"
                self._file.write(
                    f"  waypoint {i + 1}/{len(waypoints)}: "
                    f"({x:.2f}, {y:.2f}) yaw {yaw_txt}\n")
        except OSError:
            self._file = None

    def event(self, tag, detail="", throttle_s=0.0):
        """One decision line: `+12.4s  tag  detail`. Dropped silently when no
        ride is open (the file holds rides, not idle chatter) or when the
        same tag logged less than throttle_s ago (feedback would otherwise
        drown the decisions it is there to explain)."""
        if self._file is None:
            return
        now = self._clock()
        if throttle_s > 0.0:
            last = self._last_at.get(tag)
            if last is not None and now - last < throttle_s:
                return
        self._last_at[tag] = now
        try:
            self._file.write(f"+{now - self._t0:7.1f}s  {tag:<18} {detail}\n")
        except OSError:
            self._file = None

    def _close(self):
        if self._file is not None:
            try:
                self._file.close()
            except OSError:
                pass
            self._file = None
