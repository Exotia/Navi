"""goal_relay's rules, with no ROS in them.

Spec section 7: the ground station's Go/Pause/Resume/Abort row drives one
run at a time through Nav2, gated on the coordinator's mission state the
same way mode_supervisor.py gates /rover_twist on a single writer. Time is
passed in rather than read, so the whole thing runs under plain pytest
against a fake clock (spec section 9 rung 1), the same idiom as
navi_supervisor/supervisor_state.py.

nav_run.py owns the rules; the node (goal_relay) owns the topics, the
Nav2 action client, and the coordinator RPC poll.
"""

import math

# -- the seven states -------------------------------------------------------
IDLE = "idle"
REFUSED = "refused"
STARTING = "starting"
RUNNING = "running"
PAUSED = "paused"
ABORTED = "aborted"
SUCCEEDED = "succeeded"

# -- the side effects, drained by take_actions() in the order queued --------
START_TASK = "start_task"           # coordinator startNaViTask(waypoints)
SEND_GOAL = "send_goal"             # Nav2Control.send_goal(index, x, y, yaw)
CANCEL_GOAL = "cancel_goal"         # Nav2Control.cancel_goal(reason)
PAUSE_TASK = "pause_task"           # coordinator {"action": "pause"}
RESUME_TASK = "resume_task"         # coordinator {"action": "resume"}
ABORT_TASK = "abort_task"           # coordinator {"action": "abort"}
NOTIFY_WAYPOINT = "notify_waypoint"         # /nav_status waypoint reached
NOTIFY_DESTINATION = "notify_destination"   # /nav_status destination reached

# Declared and never emitted: the supervisor is the single authority on
# mode (spec section 4). It exists so a test can assert its absence, and
# so a future change that starts emitting it is a visible one.
REQUEST_MODE = "request_mode"

# The coordinator's PrepareAutonomous -> Autonomous transition takes ~5 s
# (spec section 3); doubled, plus the RPC round trip.
ARM_TIMEOUT_S = 12.0

# The Spec section 10 first-stage speed cap, used only to turn remaining
# metres into an ETA - never to command a speed.
NOMINAL_SPEED_MPS = 0.05

# This build only plans in the map frame.
PLAN_FRAME = "map"

AUTONOMOUS = "autonomous"

# The coordinator's own name for its driving state, distinct from the
# supervisor's lower-case AUTONOMOUS mode string above.
_COORDINATOR_AUTONOMOUS = "Autonomous"

_ACTIVE_STATES = (STARTING, RUNNING, PAUSED)


class NavRun:

    def __init__(self, clock, arm_timeout_s=ARM_TIMEOUT_S):
        self.clock = clock
        # A constructor arg rather than a module constant read at each
        # tick(), so goal_relay's declared "arm_timeout_s" ROS parameter
        # (SP11 task 5, parked pending this wiring) can actually reach the
        # timeout it names. The default keeps ARM_TIMEOUT_S the effective
        # value for every caller that does not pass one, tests included.
        self._arm_timeout_s = float(arm_timeout_s)
        self._state = IDLE
        self._run_id = None
        self._waypoints = []           # list of (x, y, yaw)
        self._index = 0
        self._error = None
        self._mode = None
        self._mode_reason = ""
        self._coordinator_state = None
        self._armed_at = None
        self._feedback = None          # (distance_remaining, eta_s) or None
        self._actions = []
        self._changed = False

    @property
    def state(self):
        return self._state

    # -- requests from the ground station ----------------------------------
    def on_request(self, now, request):
        action = request.get("action")
        run_id = request.get("run_id")
        if action == "go":
            self._on_go(now, request)
        elif action == "pause":
            self._on_pause(run_id)
        elif action == "resume":
            self._on_resume(now, run_id)
        elif action == "abort":
            self._on_abort(run_id)
        else:
            self._refuse(f"unknown action {action!r}")

    def _on_go(self, now, request):
        # Two gates on Go, deliberately: the row disables the button while a
        # run is active, but a stray `ros2 topic pub` or a second ground
        # station must be refused here too. Checked first so a go that
        # replaced _waypoints/_index mid-drive can never happen.
        if self._state in _ACTIVE_STATES:
            self._refuse(f"a run is already {self._state}; abort it first")
            return
        if self._mode != AUTONOMOUS:
            self._refuse("mode must be autonomous to go")
            return
        waypoints = request.get("waypoints") or []
        if not waypoints:
            self._refuse("go requires at least one waypoint")
            return
        frame_id = request.get("frame_id")
        if frame_id != PLAN_FRAME:
            self._refuse(f"cannot plan in frame {frame_id!r}; "
                         f"only {PLAN_FRAME!r} is supported")
            return
        self._run_id = request.get("run_id")
        self._waypoints = [(float(w["x"]), float(w["y"]), w.get("yaw"))
                           for w in waypoints]
        self._index = 0
        self._error = None
        self._feedback = None
        self._state = STARTING
        self._arm(now)
        self._changed = True
        self._queue(START_TASK, tuple((x, y) for x, y, _ in self._waypoints))

    def _on_pause(self, run_id):
        if self._state in (STARTING, RUNNING) and run_id == self._run_id:
            self._state = PAUSED
            self._error = None
            self._changed = True
            self._queue(CANCEL_GOAL, "operator paused")
            self._queue(PAUSE_TASK, None)
        else:
            self._refuse(f"no active run {run_id!r} to pause")

    def _on_resume(self, now, run_id):
        if self._state == PAUSED and run_id == self._run_id:
            # Resume runs the identical arming path as Go: back to STARTING,
            # _index kept, and it waits for the same OBSERVED transition -
            # a coordinator left in Disconnected or Waiting by an unbounded
            # pause must not be sent a goal on a value it cached earlier.
            self._state = STARTING
            self._error = None
            self._arm(now)
            self._changed = True
            self._queue(RESUME_TASK, None)
        else:
            self._refuse(f"no paused run {run_id!r} to resume")

    def _on_abort(self, run_id):
        if self._state in _ACTIVE_STATES and run_id == self._run_id:
            self._abort("operator aborted", cancel_reason="operator aborted")
        else:
            self._refuse(f"no active run {run_id!r} to abort")

    def _refuse(self, reason):
        """Record the reason without touching an active run's waypoints.

        Only a refusal that arrives with no active run moves the machine to
        REFUSED. A refusal that arrives while a run is starting/running/
        paused leaves _state, _run_id, _waypoints and _index exactly as
        they were - the run keeps driving and /nav_status keeps reporting
        it; a _refuse that reset the run would turn a stray publish into a
        stopped mission.
        """
        self._error = reason
        self._changed = True
        if self._state not in _ACTIVE_STATES:
            self._state = REFUSED

    # -- coordinator / mode feed ---------------------------------------------
    def on_mode_status(self, now, mode, reason):
        was_autonomous = self._mode == AUTONOMOUS
        self._mode = mode
        self._mode_reason = reason
        self._changed = True
        if was_autonomous and mode != AUTONOMOUS and self._state in _ACTIVE_STATES:
            # The supervisor has already cancelled the goal and deactivated
            # Nav2 on its own; asking again is harmless and makes this
            # correct when the mode changed for some other reason.
            self._abort(reason, cancel_reason=reason)

    def on_coordinator_state(self, name):
        self._coordinator_state = name
        self._changed = True

    def on_coordinator_stop(self, now):
        """navi_rpc_server bumped stop_seq on /navi_rpc/status: F6
        stopNavigation, F7(false), or a failed run all share the one stop
        path on that side (SP8's navi_rpc_state._stop_actions), and none of
        them asks the supervisor for a mode change - SP8's own plan is
        explicit that completing this side is SP11's job. The coordinator
        has already stopped the chassis, so a Nav2 goal still in flight
        is not driving anything, it is fighting a machine that no longer
        listens; cancelling it is what actually stops /autonomy_twist.

        No PAUSE_TASK is queued back: the coordinator caused this stop, so
        telling it to pause again would only be a round trip to itself.
        The run goes to PAUSED rather than ABORTED because F6 is also how
        CoordinatorImpl::pause() reaches us while still Autonomous - the
        operator's own Resume (which re-arms exactly like Go) is still the
        right next step, and if the coordinator will not accept it the arm
        timeout aborts the run with a reason, same as any other resume.
        """
        if self._state not in (STARTING, RUNNING):
            return
        self._state = PAUSED
        self._error = "coordinator stopped navigation"
        self._changed = True
        self._queue(CANCEL_GOAL, "coordinator stop")

    def _arm(self, now):
        # Arming requires an OBSERVED transition, never a cached value: a
        # value cached from the previous run, or from a manual
        # /drive_command, is not evidence that THIS task is armed.
        self._coordinator_state = None
        self._armed_at = now

    def tick(self, now):
        if self._state != STARTING:
            return
        if self._coordinator_state == _COORDINATOR_AUTONOMOUS:
            self._send_current_goal()
            return
        if now - self._armed_at > self._arm_timeout_s:
            self._abort("coordinator did not reach Autonomous", cancel_reason=None)

    def _send_current_goal(self):
        self._state = RUNNING
        self._changed = True
        x, y, yaw = self._waypoints[self._index]
        self._queue(SEND_GOAL, (self._index, x, y, yaw))

    # -- Nav2 goal results ----------------------------------------------------
    def on_goal_succeeded(self, now):
        # Late results from superseded goals are ignored: NavigateToPose's
        # result future fires for a cancelled goal too.
        if self._state != RUNNING:
            return
        self._queue(NOTIFY_WAYPOINT, self._index)
        self._feedback = None
        self._index += 1
        if self._index < len(self._waypoints):
            # NOT the next SEND_GOAL: notifyTaskFinished(TAG_WaypointReached)
            # moves the coordinator to Waiting, which force-disables BEMA
            # movement until the operator resumes (CoordinatorImpl.cpp:218-236
            # - its LED even shows the Waiting pattern). A goal sent now would
            # drive a braked chassis into the 45 s progress abort. PAUSED is
            # the state whose Resume already runs the full re-arming path
            # (RESUME_TASK -> observed Autonomous -> goal), so the operator's
            # Resume at the waypoint is the release, exactly as the
            # coordinator's state machine intends.
            self._state = PAUSED
            self._error = (f"waypoint {self._index}/{len(self._waypoints)} "
                           "reached - coordinator holds in Waiting, resume "
                           "to continue")
        else:
            self._queue(NOTIFY_DESTINATION, None)
            self._state = SUCCEEDED
        self._changed = True

    def on_goal_failed(self, now, reason):
        if self._state != RUNNING:
            return
        # The goal already ended itself - nothing to cancel, only to abort.
        self._abort(reason, cancel_reason=None)

    def on_feedback(self, now, distance_remaining, eta_s):
        if self._state != RUNNING:
            return
        self._feedback = (float(distance_remaining), float(eta_s))
        self._changed = True

    def _abort(self, reason, cancel_reason=None):
        # Ordering is the contract: the goal must stop before the mission
        # state is told it did.
        self._state = ABORTED
        self._error = reason
        self._changed = True
        if cancel_reason is not None:
            self._queue(CANCEL_GOAL, cancel_reason)
        self._queue(ABORT_TASK, None)

    # -- status / actions -----------------------------------------------------
    def _legs_ahead(self):
        """Straight-line distance for the waypoints after the one in flight.

        The active goal's own remaining distance comes from /nav_feedback;
        this is only the legs Nav2 has not been told about yet.
        """
        pts = self._waypoints
        return sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
                   for i in range(self._index, len(pts) - 1))

    def status(self, now):
        legs_ahead = self._legs_ahead()
        if self._feedback is None:
            distance_remaining_m = None
            eta_s = None
        else:
            distance, eta = self._feedback
            distance_remaining_m = distance + legs_ahead
            eta_s = eta + legs_ahead / NOMINAL_SPEED_MPS
        return {
            "state": self._state,
            "run_id": self._run_id,
            "waypoint_index": self._index,
            "waypoint_count": len(self._waypoints),
            "distance_remaining_m": distance_remaining_m,
            "eta_s": eta_s,
            "error": self._error,
            "mode": self._mode,
            "coordinator_state": self._coordinator_state,
            "stamp_s": float(now),
        }

    def _queue(self, name, payload):
        self._actions.append((name, payload))

    def take_actions(self):
        actions, self._actions = self._actions, []
        return actions

    def take_changed(self):
        changed, self._changed = self._changed, False
        return changed
