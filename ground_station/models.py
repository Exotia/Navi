"""Pure-Python state models for the ground station — no Qt or ROS imports."""

import json
import math
from dataclasses import dataclass, field
from time import monotonic


@dataclass
class TwistSample:
    linear_x: float
    linear_y: float
    angular_z: float
    received_at: float


class DriveCommandTracker:
    """Tracks the latest /cmd_vel Twist message and its incoming rate."""

    def __init__(self, rate_window_seconds: float = 2.0):
        self.rate_window_seconds = rate_window_seconds
        self.latest: TwistSample | None = None
        self._timestamps: list[float] = []

    def ingest(self, linear_x: float, linear_y: float, angular_z: float,
               now: float | None = None) -> None:
        now = monotonic() if now is None else now
        self.latest = TwistSample(linear_x, linear_y, angular_z, now)
        self._timestamps.append(now)
        cutoff = now - self.rate_window_seconds
        self._timestamps = [t for t in self._timestamps if t >= cutoff]

    @property
    def rate_hz(self) -> float:
        if len(self._timestamps) < 2:
            return 0.0
        span = self._timestamps[-1] - self._timestamps[0]
        if span <= 0:
            return 0.0
        return (len(self._timestamps) - 1) / span

    def seconds_since_last(self, now: float | None = None) -> float | None:
        if self.latest is None:
            return None
        now = monotonic() if now is None else now
        return now - self.latest.received_at


@dataclass
class NodeStatus:
    name: str
    alive: bool
    last_seen: float


class NodeRegistry:
    """Tracks which ROS2 nodes are currently present, from periodic polls
    of rosbridge's rosapi node list."""

    def __init__(self, stale_after_seconds: float = 5.0):
        self.stale_after_seconds = stale_after_seconds
        self._nodes: dict[str, NodeStatus] = {}

    def update(self, present_node_names: list[str], now: float | None = None) -> None:
        now = monotonic() if now is None else now
        for name in present_node_names:
            self._nodes[name] = NodeStatus(name=name, alive=True, last_seen=now)
        cutoff = now - self.stale_after_seconds
        for status in self._nodes.values():
            if status.last_seen < cutoff:
                status.alive = False

    def snapshot(self) -> list[NodeStatus]:
        return sorted(self._nodes.values(), key=lambda s: s.name)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Heading in radians, from a quaternion.

    The standard ZYX extraction, written out rather than pulled from a
    library: this file is deliberately dependency-free (no Qt, no ROS, no
    numpy), and the alternative is a transforms3d dependency for four
    multiplications.
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_readout_from_odometry(message: dict) -> dict:
    """x, y and yaw out of a nav_msgs/Odometry as rosbridge delivers it.

    Every level is defended with a default because the message comes off the
    wire as whatever JSON arrived: a truncated one must produce a visibly
    wrong readout at the origin, not an exception inside a Qt slot while the
    operator is driving.
    """
    pose = (message.get("pose") or {}).get("pose") or {}
    position = pose.get("position") or {}
    orientation = pose.get("orientation") or {}
    return {
        "x": float(position.get("x", 0.0)),
        "y": float(position.get("y", 0.0)),
        "yaw": yaw_from_quaternion(
            float(orientation.get("x", 0.0)), float(orientation.get("y", 0.0)),
            float(orientation.get("z", 0.0)), float(orientation.get("w", 1.0))),
    }


@dataclass
class MapState:
    """/localization/map_status as the Map row shows it."""
    cells_seen: int
    extent_m: tuple
    tiles: int
    loaded: str | None
    maps: list
    last_command: dict | None


def _safe_int(value, default=0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _safe_extent(value) -> tuple:
    if not isinstance(value, list) or len(value) != 2:
        return (0.0, 0.0)
    try:
        return (float(value[0]), float(value[1]))
    except (TypeError, ValueError):
        return (0.0, 0.0)


def _safe_maps(value) -> list:
    if not isinstance(value, list):
        return []
    return [str(name) for name in value]


def parse_map_status(payload: str):
    """A best-effort MapState from /localization/map_status's JSON.

    The payload comes off the wire as whatever arrived: a field of the
    wrong type (a string where a list was expected, a one-element extent,
    non-numeric counts) must fall back to that field's default rather than
    raise, the same defensive stance as pose_readout_from_odometry.
    """
    try:
        status = json.loads(payload)
    except (json.JSONDecodeError, TypeError):
        return None
    if not isinstance(status, dict):
        return None
    last = status.get("last_command")
    return MapState(
        cells_seen=_safe_int(status.get("cells_seen", 0)),
        extent_m=_safe_extent(status.get("extent_m")),
        tiles=_safe_int(status.get("tiles", 0)),
        loaded=status.get("loaded") if isinstance(status.get("loaded"), str) else None,
        maps=_safe_maps(status.get("maps", [])),
        last_command=last if isinstance(last, dict) else None)


def map_command_json(action: str, name: str | None = None) -> str:
    command = {"action": action}
    if name is not None:
        command["name"] = name
    return json.dumps(command)


_COORDINATOR_STATES = {
    0: "Disconnected", 1: "Idle", 2: "PrepareManual", 3: "Manual",
    4: "PrepareAutonomous", 5: "Autonomous", 6: "Waiting",
}


def _coordinator_name(value):
    return _COORDINATOR_STATES.get(value) if isinstance(value, int) and not isinstance(value, bool) else None


@dataclass
class DriveState:
    """/drive_status as the Drive row shows it."""
    connected: bool
    lease: bool
    coordinator_state: str | None
    deadman_active: bool
    twist_age_s: float | None
    last_action: str | None
    last_error: str | None


def parse_drive_status(payload: str):
    try:
        status = json.loads(payload)
    except (json.JSONDecodeError, TypeError):
        return None
    if not isinstance(status, dict):
        return None
    age = status.get("twist_age_s")
    return DriveState(
        connected=status.get("connected") is True,
        lease=status.get("lease") is True,
        coordinator_state=_coordinator_name(status.get("coordinator_state")),
        deadman_active=status.get("deadman_active") is True,
        twist_age_s=age if isinstance(age, (int, float)) and not isinstance(age, bool) else None,
        last_action=status.get("last_action") if isinstance(
            status.get("last_action"), str) else None,
        last_error=status.get("last_error") if isinstance(
            status.get("last_error"), str) else None)


def drive_command_json(action: str) -> str:
    return json.dumps({"action": action})


# The modes in which the ground station streams /manual_twist continuously.
# In autonomous it streams nothing - only a deflected stick is published,
# see may_publish_takeover_twist - so a stick that does move is a real
# takeover signal rather than one message in a 20 Hz stream of zeros; in
# estop it publishes nothing at all, because there is nothing to say.
DRIVING_MODES = ("manual", "semi_auto")
AUTONOMOUS_MODE = "autonomous"


@dataclass
class ModeState:
    """/mode_status as the DRIVE row's mode chip shows it."""
    mode: str
    reason: str
    source: str | None
    deadman_active: bool
    estop_latched: bool
    localization_state: str | None
    source_age_s: float | None


def parse_mode_status(payload: str):
    """A best-effort ModeState from /mode_status's JSON.

    The same defensive stance as parse_drive_status: a payload that will
    not parse, or a field of the wrong type, falls back to that field's
    default rather than raising inside a Qt slot while someone is driving.
    """
    try:
        status = json.loads(payload)
    except (json.JSONDecodeError, TypeError):
        return None
    if not isinstance(status, dict):
        return None
    age = status.get("source_age_s")
    source = status.get("source")
    localization = status.get("localization_state")
    return ModeState(
        mode=str(status.get("mode", "")),
        reason=str(status.get("reason") or ""),
        source=source if isinstance(source, str) else None,
        deadman_active=status.get("deadman_active") is True,
        estop_latched=status.get("estop_latched") is True,
        localization_state=localization if isinstance(localization, str) else None,
        source_age_s=(age if isinstance(age, (int, float))
                      and not isinstance(age, bool) else None))


def may_publish_manual_twist(state) -> bool:
    """Spec rule 5: stream /manual_twist only in manual and semi_auto.

    This is the *stream* gate. In autonomous it is False, and the takeover
    path (may_publish_takeover_twist + is_stick_deflected) is what still
    lets a deflected stick through - rule 5's own justification is that
    rule 1 becomes "a real signal rather than a constant stream", not that
    rule 1 becomes unreachable.

    No mode status at all means no supervisor is running on that rover -
    an older build, or one brought up with --no-supervisor. Then this
    returns True, because publishing is the behaviour that rover needs and
    the stream reaches nothing that can act on it anyway; a rover that does
    have a supervisor answers within half a second of the subscription
    being made, so the permissive window is short.

    An unknown mode name is the other way round: a supervisor exists and
    is in a state this ground station does not know about, and the
    supervisor is the authority on what its own modes mean. Guessing in
    the permissive direction there is how a takeover gets faked.
    """
    if state is None:
        return True
    return state.mode in DRIVING_MODES


def may_publish_takeover_twist(state) -> bool:
    """Whether a *deflected* stick may still be published in this mode.

    Only autonomous: that is where rule 1 lives. In estop nothing may be
    published, and in a mode this build does not know the supervisor is
    the authority on what its own modes mean - guessing permissively there
    is how a takeover gets faked. With no mode status at all the stream
    gate is already open, so this never has to decide.
    """
    return state is not None and state.mode == AUTONOMOUS_MODE


def is_stick_deflected(twist) -> bool:
    """"Above the deadzone", in the ground station's own terms.

    No new threshold: gamepad_input.GamepadReader.read_twist() has already
    run every axis through _apply_deadzone(DEADZONE = 0.1) before scaling,
    so a centred stick is exactly (0.0, 0.0, 0.0) and the smallest
    deflection it can report is DEADZONE * MAX_LINEAR_SPEED = 0.005 m/s
    (0.01 rad/s) - well above the supervisor's TAKEOVER_LINEAR_MPS of
    0.002. A non-zero component is therefore past the deadzone by
    construction, and gamepad_input.py stays untouched.
    """
    return any(float(v) != 0.0 for v in twist)


def mode_request_json(mode: str) -> str:
    return json.dumps({"mode": mode})


def estop_request_json(reason: str) -> str:
    return json.dumps({"reason": reason})
