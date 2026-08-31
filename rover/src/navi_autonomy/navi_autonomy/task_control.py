"""What goal_relay needs from the primary's coordinator, and the stopgap
that stands in until SP8's navi_rpc client exists.

SP8 builds `navi_rpc_server` (the :21021 endpoint the coordinator calls
back into) and the coordinator client for task state. SP11 must not guess
its symbol names, so it names an interface instead and binds SP8's real
client to it in the integration checkpoint (SP11 task 8). The interface is
the *sequence* goal_relay asks for; that sequence must not change when the
stopgap stops being a stopgap.
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


class TopicTaskControl(TaskControl):
    """The stopgap. One method is real today and stays real after SP8.

    `abort` publishes {"action": "abort"} on /drive_command - the path
    bema_bridge already implements (BemaSession.abort: take the
    coordinator's own __sam__ lease, then F7). Everything else logs and
    records that it was asked, exactly as NullNav2Control does, and reports
    the coordinator state bema_bridge already puts on /drive_status.

    `start_task` is a NO-OP here: nothing in this stopgap puts the
    coordinator into PrepareAutonomous. So with this bound, Go reaches
    Nav2 only when the coordinator already happens to be in Autonomous
    from a manual /drive_command; otherwise the run aborts at
    ARM_TIMEOUT_S with the coordinator named. SP11 task 8 is REQUIRED for
    the feature, not a tidy-up.

    Removed in SP11 task 8, except `abort`, which SP8's client inherits.
    """

    # /drive_status carries coordinator_state as an INT (bema_bridge
    # publishes BemaSession._coord_state). NavRun compares against the
    # NAME, so the mapping happens here. A local copy of the table rather
    # than an import: the rover package must not import ground_station,
    # and ground_station.models._COORDINATOR_STATES is the same seven
    # values - keep them identical.
    _COORDINATOR_STATES = {
        0: "Disconnected", 1: "Idle", 2: "PrepareManual", 3: "Manual",
        4: "PrepareAutonomous", 5: "Autonomous", 6: "Waiting",
    }

    def __init__(self, node):
        self._logger = node.get_logger()
        self.calls = []
        self._last_coordinator_state = None
        self._command_pub = node.create_publisher(String, "/drive_command", 10)
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
        self.calls.append(("start_task", tuple(waypoints)))
        self._logger.info(
            f"navi_task requested for {len(tuple(waypoints))} waypoint(s); "
            "the stopgap does not arm the coordinator - SP11 task 8 does")

    def pause(self) -> None:
        self.calls.append(("pause", None))
        self._logger.info("pause requested; no coordinator RPC until SP8")

    def resume(self) -> None:
        self.calls.append(("resume", None))
        self._logger.info("resume requested; no coordinator RPC until SP8")

    def abort(self) -> None:
        self.calls.append(("abort", None))
        msg = String()
        msg.data = json.dumps({"action": "abort"})
        self._command_pub.publish(msg)

    def notify_waypoint_reached(self, index: int) -> None:
        self.calls.append(("waypoint", index))
        self._logger.info(f"waypoint {index} reached; no coordinator RPC until SP8")

    def notify_destination_reached(self) -> None:
        self.calls.append(("destination", None))
        self._logger.info("destination reached; no coordinator RPC until SP8")

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
