"""What goal_relay needs from the primary's coordinator, bound to SP8's
actual navi_rpc / bema_bridge wire (SP11 task 8 - the integration
checkpoint this file names in its own docstring history).

SP8 built `navi_rpc_server` (in `navi_supervisor`, not a separate `navi_rpc`
package - see its plan's decision 1) and the coordinator client for task
state (`navi_teleop.bema_session.BemaSession`). Reconciled against the real
code rather than guessed:

  * `start_task`/`pause`/`resume`/`abort` publish JSON on `/drive_command`.
    `navi_teleop.bema_bridge.BemaBridge._on_command` is the consumer; it
    turns `{"action": "navi_task", "waypoints": [[x, y, w], ...]}` into
    `BemaSession.start_navi_task` (coordinator's guarded F0), `pause_task`/
    `resume_task` into F4/F5, and `{"action": "abort"}` (unchanged from the
    stopgap) into `BemaSession.abort` (F7). All four already do the
    coordinator's own just-in-time `__sam__` lease dance inside
    `bema_session.py`'s `_coord_guarded` - there is no second client to
    bind, so `abort` keeps the stopgap's path rather than growing one.

  * `notify_waypoint_reached`/`notify_destination_reached` do NOT publish
    `{"action": "task_finished"}` on `/drive_command` - that would be a
    second writer of the coordinator's F8. `navi_rpc_server`
    (`navi_supervisor.navi_rpc_state.NaviRpcState.on_progress`) is the
    single path from a completed leg to `notifyTaskFinished`: it consumes
    `/navi_rpc/progress` (`{"event": "waypoint_reached"|"destination_reached",
    "index": int|None, "reason": None}`) and performs the F8 publish
    itself. SP8's own task-6 brief says this in as many words: "a second
    path here would desynchronise /navi_rpc/status's reached_index from
    the coordinator's real state."

  * `coordinator_state` is untouched: `/drive_status` still carries the
    coordinator's mission-state int (`BemaSession.status()`'s
    `coordinator_state`, published by `bema_bridge`), and nothing SP8 built
    replaces that poll. The mapping table is kept local rather than
    imported for the same reason `nav2_goals.py` keeps its own copy of
    `PLAN_FRAME`: this rover package must not import ground_station, and
    `ground_station.models._COORDINATOR_STATES` is the same seven values -
    keep them identical.

One finding worth recording rather than papering over: `nav_run.py`'s
START_TASK action queues bare `(x, y)` pairs with no yaw (pinned by
`test_nav_run.py` and `test_goal_relay.py` - unchanged by this task, since
moving that pinned sequence was explicitly out of scope). SP8's own plan
says "we are the only producer end-to-end" for a target's yaw and expects
`goal_relay` to supply it; since NavRun does not carry one this far, the
missing third component is filled with `0.0` here. Harmless: the
coordinator relays these triples straight back to us as its own NaVi F3
targets for its own tracking, not for Nav2's path - Nav2 (fed the operator's
real per-waypoint yaw via goal_relay's own `_resolve_yaw`) is what actually
drives the rover.
"""

import json

from std_msgs.msg import String


class TaskControl:
    """The interface SP8's client is bound to. Do not change these names."""

    def start_task(self, waypoints) -> None:
        """Coordinator F0 startNaViTask(waypoints)."""
        raise NotImplementedError

    def pause(self) -> None: raise NotImplementedError        # F4
    def resume(self) -> None: raise NotImplementedError       # F5
    def abort(self) -> None: raise NotImplementedError        # F7
    def notify_waypoint_reached(self, index: int) -> None:    # F8, TAG 0x31
        raise NotImplementedError
    def notify_destination_reached(self) -> None:             # F8, TAG 0x32
        raise NotImplementedError

    def coordinator_state(self):
        """The mission state name, or None if unknown. Never blocks."""
        raise NotImplementedError


class RecordingTaskControl(TaskControl):
    """The tests' double, and the shape SP8's client must satisfy."""

    def __init__(self, state=None):
        self.calls = []
        self.state = state

    def start_task(self, waypoints): self.calls.append(("start_task", tuple(waypoints)))
    def pause(self): self.calls.append(("pause", None))
    def resume(self): self.calls.append(("resume", None))
    def abort(self): self.calls.append(("abort", None))
    def notify_waypoint_reached(self, index): self.calls.append(("waypoint", index))
    def notify_destination_reached(self): self.calls.append(("destination", None))
    def coordinator_state(self): return self.state


class NaviRpcTaskControl(TaskControl):
    """SP8's real bindings - see the module docstring for the reconciliation.

    Constructed with the rclpy node so it can own its own publishers/
    subscription, the same shape the stopgap it replaces used.
    """

    # /drive_status carries coordinator_state as an INT (BemaSession.status(),
    # published by bema_bridge). NavRun compares against the NAME, so the
    # mapping happens here. A local copy of the table rather than an import:
    # the rover package must not import ground_station, and
    # ground_station.models._COORDINATOR_STATES is the same seven values -
    # keep them identical.
    _COORDINATOR_STATES = {
        0: "Disconnected", 1: "Idle", 2: "PrepareManual", 3: "Manual",
        4: "PrepareAutonomous", 5: "Autonomous", 6: "Waiting",
    }

    def __init__(self, node):
        self._logger = node.get_logger()
        self.calls = []
        self._last_coordinator_state = None
        self._command_pub = node.create_publisher(String, "/drive_command", 10)
        self._progress_pub = node.create_publisher(String, "/navi_rpc/progress", 10)
        node.create_subscription(String, "/drive_status", self._on_drive_status, 10)

    def _on_drive_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError, AttributeError):
            self._logger.warn(f"unreadable /drive_status: {msg.data!r}")
            return
        self._last_coordinator_state = (
            payload.get("coordinator_state") if isinstance(payload, dict) else None)

    def start_task(self, waypoints) -> None:
        waypoints = tuple(waypoints)
        self.calls.append(("start_task", waypoints))
        # See the module docstring's finding: NavRun supplies (x, y) only,
        # yaw filled with 0.0.
        self._publish_command({
            "action": "navi_task",
            "waypoints": [[float(x), float(y), 0.0] for x, y in waypoints]})

    def pause(self) -> None:
        self.calls.append(("pause", None))
        self._publish_command({"action": "pause_task"})

    def resume(self) -> None:
        self.calls.append(("resume", None))
        self._publish_command({"action": "resume_task"})

    def abort(self) -> None:
        # Unchanged from the stopgap: BemaSession.abort already takes the
        # coordinator's own __sam__ lease before F7, so there is nothing
        # left for a second client to do.
        self.calls.append(("abort", None))
        self._publish_command({"action": "abort"})

    def notify_waypoint_reached(self, index: int) -> None:
        self.calls.append(("waypoint", index))
        self._publish_progress({
            "event": "waypoint_reached", "index": int(index), "reason": None})

    def notify_destination_reached(self) -> None:
        self.calls.append(("destination", None))
        self._publish_progress({
            "event": "destination_reached", "index": None, "reason": None})

    def coordinator_state(self):
        """The name for the last int seen on /drive_status, or None.

        None rather than a guess when the field is missing or is not an
        int: an unknown coordinator is exactly the case NavRun's arm
        timeout exists for, and a wrong name would arm a run that is not
        armed.
        """
        value = self._last_coordinator_state
        if isinstance(value, bool) or not isinstance(value, int):
            return None
        return self._COORDINATOR_STATES.get(value)

    def _publish_command(self, payload):
        msg = String()
        msg.data = json.dumps(payload)
        self._command_pub.publish(msg)

    def _publish_progress(self, payload):
        msg = String()
        msg.data = json.dumps(payload)
        self._progress_pub.publish(msg)
