"""What the coordinator's NaVi calls mean on this side, and nothing else.

`navi_rpc_protocol.py` owns the wire; this owns the contract: the waypoint
list, the run flags, and the side effects a call implies. Like
supervisor_state.py it queues actions rather than performing them - the node
drains them on a timer, because an RPC arrives on a socket thread and rclpy
publishers may only be touched from the executor's.

It is locked, because those socket threads are real: every public method
takes the same RLock, `snapshot()` and `take_actions()` included.

Two things this file deliberately does NOT do:

  * `start_navigation` does not start anything. Runs are operator-initiated
    (spec 3); the coordinator's F4 is the signal that it has reached
    Autonomous and the run is armed, which SP11's goal_relay waits for
    before it sends a Nav2 goal.
  * `movement_enabled` never gates a run. The coordinator's
    movementUpdateLoop for NaVi is commented out (CoordinatorImpl.cpp:38),
    so F7 may never arrive at all; a run gated on it would never start.
    Only the transition to False has an effect, and that effect is a stop.
  * no stop of any kind asks for a mode. The supervisor is the single
    authority on mode (SP5), and a `manual` request from here would both
    clear a latched e-stop and turn the coordinator's pause into an abort.
    A stop bumps stop_seq instead; see _stop_actions().
"""

import math
import threading
from time import monotonic

from navi_supervisor.navi_rpc_protocol import RpcRefusal

# navi::TAG_WaypointReached / TAG_DestinationReached, from NaVi.idl.
TAG_WAYPOINT_REACHED = 0x31
TAG_DESTINATION_REACHED = 0x32

# The side effects the node performs on this class's behalf. There is
# deliberately no "request manual" among them - see _stop_actions().
PUBLISH_TARGETS = "publish_targets"        # /navi_rpc/targets (nav_msgs/Path)
CHASSIS_STOP = "chassis_stop"              # /drive_command {"action": "stop"}
NOTIFY_WAYPOINT = "notify_waypoint"        # coordinator F8, tag 0x31
NOTIFY_DESTINATION = "notify_destination"  # coordinator F8, tag 0x32

# Deduplicated within one batch; the notifies never are, because two
# waypoints reached between two drains are two things the coordinator has to
# be told about.
_DEDUPED = (PUBLISH_TARGETS, CHASSIS_STOP)

# The vocabulary of /navi_rpc/progress, which SP11's goal_relay publishes.
WAYPOINT_REACHED = "waypoint_reached"
DESTINATION_REACHED = "destination_reached"
NAVIGATION_FAILED = "failed"
PROGRESS_EVENTS = (WAYPOINT_REACHED, DESTINATION_REACHED, NAVIGATION_FAILED)

# An ERC run is a handful of waypoints. A list longer than this is not a
# coordinator, and storing it would be the whole attack.
MAX_TARGETS = 64


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"target components must be numbers, got {value!r}")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("target components must be finite")
    return value


def parse_targets(targets):
    """The wire's [[x, y, w], ...] as a list of (x, y, w) floats, or ValueError.

    `w` is yaw in radians in the map frame - see the plan's rulings: the IDL
    gives no unit, and we are the only producer end to end.
    """
    if isinstance(targets, (str, bytes)) or not isinstance(targets, (list, tuple)):
        raise ValueError("setTargets takes an array of (x, y, w) targets")
    if not targets:
        raise ValueError("setTargets needs at least one target")
    if len(targets) > MAX_TARGETS:
        raise ValueError(f"setTargets takes at most {MAX_TARGETS} targets, "
                         f"got {len(targets)}")
    parsed = []
    for target in targets:
        if (isinstance(target, (str, bytes))
                or not isinstance(target, (list, tuple)) or len(target) != 3):
            raise ValueError(f"a target must be (x, y, w), got {target!r}")
        parsed.append(tuple(_number(component) for component in target))
    return parsed


class NaviRpcState:

    def __init__(self, clock=monotonic):
        self._clock = clock
        self._lock = threading.RLock()
        self._targets = []
        self._inited = False
        self._navigation_requested = False
        self._start_seq = 0
        self._started_at = None
        self._target_reached = False
        self._reached_index = None
        self._last_point_reached = False
        self._movement_enabled = False
        self._stop_seq = 0
        self._stop_requested = False
        self._last_method = None
        self._last_call_at = None
        self._last_error = None
        self._mode = None
        self._actions = []

    # --- the served methods ---------------------------------------------
    def init(self):
        with self._lock:
            self._inited = True
            self._mark("init")

    def set_targets(self, targets):
        parsed = parse_targets(targets)
        with self._lock:
            self._targets = parsed
            self._navigation_requested = False
            self._target_reached = False
            self._reached_index = None
            self._last_point_reached = False
            self._last_error = None
            self._mark("setTargets")
            self._queue(PUBLISH_TARGETS)

    def start_navigation(self):
        with self._lock:
            if not self._targets:
                self._mark("startNavigation")
                raise ValueError("no targets set")
            self._navigation_requested = True
            self._start_seq += 1
            self._started_at = self._clock()
            # A new run answers the last stop. The counter itself is
            # monotonic - goal_relay dedupes on it - so only the flag clears.
            self._stop_requested = False
            # The per-leg latch. Without clearing it, F5 would answer true
            # for a leg that has not been driven yet - the coordinator calls
            # F4 again the instant it is told a waypoint was reached.
            self._target_reached = False
            self._mark("startNavigation")

    def is_target_reached(self):
        with self._lock:
            self._mark("isTargetReached")
            return bool(self._target_reached)

    def stop_navigation(self):
        with self._lock:
            self._navigation_requested = False
            self._target_reached = False
            self._mark("stopNavigation")
            self._stop_actions()

    def set_movement_enabled(self, enable):
        if not isinstance(enable, bool):
            raise ValueError("setMovementEnabled takes a bool")
        with self._lock:
            self._movement_enabled = enable
            self._mark("setMovementEnabled")
            if not enable:
                self._navigation_requested = False
                self._target_reached = False
                self._stop_actions()

    # --- inputs from the ROS side ---------------------------------------
    def on_progress(self, event, index=None, reason=None):
        with self._lock:
            if event == WAYPOINT_REACHED:
                self._target_reached = True
                self._reached_index = None if index is None else int(index)
                self._queue(NOTIFY_WAYPOINT)
            elif event == DESTINATION_REACHED:
                self._target_reached = True
                self._last_point_reached = True
                self._navigation_requested = False
                self._reached_index = None if index is None else int(index)
                self._queue(NOTIFY_DESTINATION)
            elif event == NAVIGATION_FAILED:
                # No completion is invented: the coordinator has no failure
                # tag, and both of the tags it has advance its state machine
                # as if we had arrived. The operator aborts.
                self._navigation_requested = False
                self._target_reached = False
                self._last_error = str(reason) if reason else "navigation failed"
                # The same stop path as F6 and F7(false), not a weaker one:
                # goal_relay watches stop_seq, so all three must move it.
                self._stop_actions()
            else:
                self._last_error = f"unknown progress event {event!r}"

    def on_mode(self, mode):
        with self._lock:
            self._mode = mode if isinstance(mode, str) else None

    # --- output ----------------------------------------------------------
    def snapshot(self):
        with self._lock:
            now = self._clock()
            age = (None if self._last_call_at is None
                   else round(now - self._last_call_at, 2))
            return {
                "inited": self._inited,
                "targets": [list(target) for target in self._targets],
                "target_count": len(self._targets),
                "navigation_requested": self._navigation_requested,
                "start_seq": self._start_seq,
                "started_at": self._started_at,
                "target_reached": self._target_reached,
                "reached_index": self._reached_index,
                "last_point_reached": self._last_point_reached,
                "movement_enabled": self._movement_enabled,
                # What SP11's goal_relay watches to cancel a Nav2 goal. The
                # counter is monotonic so a missed status message cannot lose
                # a stop; the flag says whether the last stop still stands.
                "stop_seq": self._stop_seq,
                "stop_requested": self._stop_requested,
                "last_method": self._last_method,
                "last_call_age_s": age,
                "last_error": self._last_error,
                "supervisor_mode": self._mode,
            }

    def take_actions(self):
        with self._lock:
            actions, self._actions = self._actions, []
            return actions

    # --- internals -------------------------------------------------------
    def _mark(self, method):
        self._last_method = method
        self._last_call_at = self._clock()

    def _stop_actions(self):
        """The one stop path, shared by F6, F7(false) and a failed run.

        A chassis stop and a bumped counter - never a mode request. Two
        reasons, either of which is sufficient:

          * SupervisorState.on_mode_request clears the e-stop latch for an
            accepted `manual`, so the primary's state machine could clear
            our operator's e-stop.
          * CoordinatorImpl::pause() calls F6 while it is still in
            Autonomous. A `manual` request there comes back through the
            supervisor as CANCEL_GOAL/DEACTIVATE_NAV2/COORDINATOR_ABORT, so
            the operator's pause would silently become an abort and Resume
            would be refused from Idle.

        Mode belongs to the supervisor, alone (SP5). What actually stops
        /autonomy_twist is SP11's goal_relay cancelling the Nav2 goal when
        it sees stop_seq move.
        """
        self._queue(CHASSIS_STOP)
        self._stop_seq += 1
        self._stop_requested = True

    def _queue(self, *actions):
        for action in actions:
            if action in _DEDUPED and action in self._actions:
                continue
            self._actions.append(action)


# --- the method table -----------------------------------------------------
#
# Names, arities and guard flags come from coordinator/deps/naviInterface:
# NaViProxy binds F0-F12, and NaVi.idl marks F2, F8 and F9 @noguard. The
# camera methods F10-F12 are deliberately not bound - spec 3 lists F0-F9 as
# the interface, and the pan/tilt/zoom head is not ours; an unknown-method
# error beats a hang.

GUARDED_METHODS = frozenset({"F0", "F1", "F3", "F4", "F5", "F6", "F7"})

STUBS = {
    "F1": "setPosition",
    "F2": "getPosition",
    "F8": "getTofData",
    "F9": "takeSnapshot",
}

# What F2 answers. NaViEP::getPosition() unpacks the reply as a
# std::array<std::array<float,4>,4>, so a 4x4 identity is the correctly
# shaped way to say "no pose information from here".
IDENTITY_POSE = [[1.0, 0.0, 0.0, 0.0],
                 [0.0, 1.0, 0.0, 0.0],
                 [0.0, 0.0, 1.0, 0.0],
                 [0.0, 0.0, 0.0, 1.0]]


def _refusal(method):
    return RpcRefusal(f"{method} ({STUBS[method]}) is not served by "
                      f"navi_rpc_server")


def navi_method_table(state, logger=None):
    """The F-methods, bound to a NaviRpcState. Arities match the IDL, so a
    call with the wrong number of arguments is answered as bad arguments
    rather than silently accepted.

    F1 is guarded, and NaViEP::setPosition() catches rpc::rpc_error, so it
    refuses by name. F2, F8 and F9 are @noguard, and their client half -
    WeakNaViEP::getPosition/getTofData/takeSnapshot - does NOT catch: an
    error reply there propagates as rpc::rpc_error into whatever thread
    made the call, and std::terminate if it escapes a thread function.
    Since the .18 alias makes us the NaVi endpoint for the whole rover LAN,
    anything that used to poll the real NaVi for a pose, a ToF frame or a
    snapshot now reaches us - so those three answer benign, correctly-shaped
    values (an identity pose, an empty frame, nil) and log the call. That is
    deliberate: "no data" is readable, an exception is not.
    """

    def _note(method):
        if logger is None:
            return
        try:
            logger.info(f"{method} ({STUBS[method]}) is not served by "
                        f"navi_rpc_server; answering with an empty value")
        except Exception:                       # a logger must not kill us
            pass

    def f0():
        state.init()

    def f1(x, y):
        raise _refusal("F1")

    def f2():
        _note("F2")
        return IDENTITY_POSE

    def f3(targets):
        state.set_targets(targets)

    def f4():
        state.start_navigation()

    def f5():
        return state.is_target_reached()

    def f6():
        state.stop_navigation()

    def f7(enable):
        state.set_movement_enabled(enable)

    def f8():
        _note("F8")
        return []                               # an empty std::vector<float>

    def f9(index):
        _note("F9")
        return None

    return {"F0": f0, "F1": f1, "F2": f2, "F3": f3, "F4": f4,
            "F5": f5, "F6": f6, "F7": f7, "F8": f8, "F9": f9}
