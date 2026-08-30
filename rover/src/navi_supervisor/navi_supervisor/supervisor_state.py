"""The mode supervisor's rules, with no ROS in them.

Modes are manual, semi_auto, autonomous and estop. This class decides
which twist reaches the chassis, when the deadman fires, when the e-stop
latches, and which side effects the node owes as a result - cancelling the
Nav2 goal, telling the coordinator to abort and re-enter Manual, stopping
the wheels. Time is passed in rather than read, so the whole thing runs
against a fake clock the way bema_session does.

mode_supervisor.py owns the timers and the topics; this file owns the
rules, and is the only place they are written down.
"""

MANUAL = "manual"
SEMI_AUTO = "semi_auto"
AUTONOMOUS = "autonomous"
ESTOP = "estop"

MODES = (MANUAL, SEMI_AUTO, AUTONOMOUS, ESTOP)
MANUAL_MODES = (MANUAL, SEMI_AUTO)

MANUAL_DEADMAN_S = 1.0
AUTONOMY_DEADMAN_S = 0.5

# Above these a /manual_twist counts as the operator taking over. The
# ground station's smallest non-zero output is its gamepad deadzone times
# its speed cap - 0.1 * 0.05 = 0.005 m/s and 0.1 * 0.1 = 0.01 rad/s - so
# these sit a factor of two below any real stick deflection and far above
# the float noise around a zero.
TAKEOVER_LINEAR_MPS = 0.002
TAKEOVER_ANGULAR_RPS = 0.004

ZERO = (0.0, 0.0, 0.0)

# The side effects the node performs on this class's behalf, drained by
# take_actions() in the order they were queued.
CANCEL_GOAL = "cancel_goal"                 # Nav2Control.cancel_goal()
DEACTIVATE_NAV2 = "deactivate_nav2"         # Nav2Control.deactivate()
COORDINATOR_ABORT = "coordinator_abort"     # {"action": "abort"} on /drive_command
COORDINATOR_MANUAL = "coordinator_manual"   # {"action": "manual"}
CHASSIS_STOP = "chassis_stop"               # {"action": "stop"}


class SupervisorState:

    def __init__(self, mode=MANUAL, manual_deadman_s=MANUAL_DEADMAN_S,
                 autonomy_deadman_s=AUTONOMY_DEADMAN_S):
        self._mode = mode if mode in MODES else MANUAL
        self._manual_deadman_s = float(manual_deadman_s)
        self._autonomy_deadman_s = float(autonomy_deadman_s)
        self._manual = ZERO
        self._manual_at = None
        self._autonomy = ZERO
        self._autonomy_at = None
        self._localization = None          # None means none has arrived yet
        self._reason = "startup"
        self._estop_latched = False
        # Whether the chassis has already been told to stop. Starts True so
        # that coming up with no source yet does not send a stop to a
        # chassis that has never been asked to move.
        self._stopped = True
        self._actions = []

    @property
    def mode(self):
        return self._mode

    @property
    def estop_latched(self):
        return self._estop_latched

    # --- inputs ----------------------------------------------------------
    def on_manual_twist(self, now, vx, vy, wz):
        self._manual = (float(vx), float(vy), float(wz))
        self._manual_at = now

    def on_autonomy_twist(self, now, vx, vy, wz):
        self._autonomy = (float(vx), float(vy), float(wz))
        self._autonomy_at = now

    def on_estop_request(self, now, reason=""):
        """Rule 2: STOP is latched and local.

        Nothing here consults the mode: an e-stop from any state is an
        e-stop, and the goal cancel goes out unconditionally rather than
        only when this class believes it is autonomous - a Nav2 that was
        still winding down from a takeover must not survive the stop
        because of a bookkeeping disagreement. The stub cancel is a no-op,
        so the unconditional call costs nothing.
        """
        self._estop_latched = True
        self._mode = ESTOP
        self._reason = str(reason) if reason else "e-stop"
        self._manual = ZERO
        self._manual_at = None
        self._autonomy = ZERO
        self._autonomy_at = None
        self._stopped = True
        self._queue(CANCEL_GOAL, DEACTIVATE_NAV2, CHASSIS_STOP)

    def on_mode_request(self, now, mode):
        """Honour a /mode_request, or return the reason it was refused."""
        if mode not in MODES:
            return f"unknown mode {mode!r}"
        if mode == ESTOP:
            self.on_estop_request(now, "mode request")
            return None
        if self._estop_latched and mode != MANUAL:
            # Rule 2 again: an explicit request back to manual is the only
            # thing that clears the latch. Anything else leaves the rover
            # stopped, and says so on /mode_status rather than silently.
            self._reason = f"e-stop latched, {mode} refused"
            return self._reason
        was_autonomous = self._mode == AUTONOMOUS
        self._estop_latched = False
        self._mode = mode
        self._reason = "mode request"
        if was_autonomous and mode != AUTONOMOUS:
            # Spec rule 1's ordering, on the path the operator actually
            # uses: the coordinator must be aborted out of the autonomous
            # task before anything asks it for Manual, or ERC's mission
            # state machine is left claiming a run that is over. Same
            # sequence as _take_over(), so the DRIVE row's Manual button
            # and a deflected stick leave the coordinator in one state.
            self._queue(CANCEL_GOAL, DEACTIVATE_NAV2, COORDINATOR_ABORT)
            if mode in MANUAL_MODES:
                self._queue(COORDINATOR_MANUAL)
        if mode in MANUAL_MODES:
            # Whatever manual twist is still retained predates this
            # request - it may be the stick position from before an e-stop.
            # It must not become the first thing the rover does on the way
            # back; the deadman holds zero until a genuinely new one lands.
            self._manual = ZERO
            self._manual_at = None
        return None

    # --- output ----------------------------------------------------------
    def _source_age(self, now):
        """(age of the mode's source, the deadman it is judged against).

        estop has no source: it is not "silent", it is stopped, so it
        returns no age at all and deadman_active answers True directly.
        """
        if self._mode in MANUAL_MODES:
            at, limit = self._manual_at, self._manual_deadman_s
        elif self._mode == AUTONOMOUS:
            at, limit = self._autonomy_at, self._autonomy_deadman_s
        else:
            return None, 0.0
        return (None if at is None else now - at), limit

    def deadman_active(self, now):
        if self._mode == ESTOP:
            return True
        age, limit = self._source_age(now)
        return age is None or age > limit

    def output(self, now):
        """The twist to publish on /rover_twist this tick.

        Called at the publish rate. The live -> stopped edge is where the
        chassis stop is queued: /rover_twist keeps carrying zeros so the
        bridge's own deadman never fires while this node lives, which means
        the wheels only really stop if something says so - once, on the
        edge, rather than twenty times a second.
        """
        if self.deadman_active(now):
            if not self._stopped:
                self._stopped = True
                self._queue(CHASSIS_STOP)
            return ZERO
        self._stopped = False
        if self._mode == AUTONOMOUS:
            return self._autonomy
        return self._manual

    def status(self, now):
        age, _ = self._source_age(now)
        return {
            "mode": self._mode,
            "reason": self._reason,
            "source": (None if self._mode == ESTOP
                       else "/autonomy_twist" if self._mode == AUTONOMOUS
                       else "/manual_twist"),
            "deadman_active": self.deadman_active(now),
            "estop_latched": self._estop_latched,
            "localization_state": self._localization,
            "source_age_s": None if age is None else round(age, 2),
        }

    # --- side effects ----------------------------------------------------
    def _queue(self, *actions):
        # Deduplicated within a batch: two localisation-loss messages in
        # one tick must not abort the coordinator twice.
        for action in actions:
            if action not in self._actions:
                self._actions.append(action)

    def take_actions(self):
        """The side effects owed since the last call, in order, drained."""
        actions, self._actions = self._actions, []
        return actions
