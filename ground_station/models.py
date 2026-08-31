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


# --- autonomy: the /nav_request, /nav_status and /nav_path_summary wire ----

# The states goal_relay's run machine can be in. Kept here as documentation
# and for the row's styling; an unknown state is still shown verbatim,
# because the rover is the authority on its own states.
NAV_STATES = ("idle", "starting", "running", "paused",
              "succeeded", "aborted", "refused")
NAV_ACTIVE_STATES = ("starting", "running", "paused")

# The rover's elevation map is capped at 60 m across (spec 1.5), so a
# waypoint beyond that can never be planned onto mapped ground. Refused at
# the point of typing, with the reason, rather than sent and refused later
# by a rover the operator cannot see.
MAX_WAYPOINT_RANGE_M = 60.0


@dataclass
class Waypoint:
    x: float
    y: float
    yaw: float | None = None


@dataclass
class NavStatus:
    """/nav_status as the NAV row shows it."""
    state: str
    run_id: str | None
    waypoint_index: int | None
    waypoint_count: int
    distance_remaining_m: float | None
    eta_s: float | None
    error: str
    mode: str | None
    coordinator_state: str | None


@dataclass
class PathSummary:
    """/nav_path_summary - the decimated plan, for the canvas and the sim."""
    run_id: str | None
    frame_id: str
    points: list
    waypoints: list
    length_m: float
    source_points: int


def new_run_id(now_s: float) -> str:
    """A run id from a wall clock, in milliseconds.

    The clock is passed in rather than read so a test can pin the id, the
    same reason every other model here takes `now`.
    """
    return f"gs-{int(now_s * 1000)}"


def nav_request_json(action: str, waypoints=None, run_id: str | None = None,
                     frame_id: str = "map") -> str:
    return json.dumps({
        "action": action,
        "run_id": run_id,
        "frame_id": frame_id,
        "waypoints": [{"x": float(w.x), "y": float(w.y),
                       "yaw": None if w.yaw is None else float(w.yaw)}
                      for w in (waypoints or [])],
    })


def _safe_float(value, default=None):
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return float(value)
    return default


def parse_nav_status(payload: str):
    """A best-effort NavStatus from /nav_status's JSON.

    The same defensive stance as parse_mode_status: a field of the wrong
    type falls back to that field's default rather than raising inside a Qt
    slot while a mission is running.

    `stamp_s` is deliberately NOT carried onto NavStatus. It is
    goal_relay's monotonic clock, meaningful only as a difference against
    another reading of *that* clock, and the ground station cannot take
    one. Staleness is judged by the ground station's own monotonic() at
    the moment the message arrived (_on_nav_status), exactly as
    /drive_status and /mode_status are.
    """
    try:
        status = json.loads(payload)
    except (json.JSONDecodeError, TypeError):
        return None
    if not isinstance(status, dict):
        return None
    index = status.get("waypoint_index")
    return NavStatus(
        state=str(status.get("state", "")),
        run_id=status.get("run_id") if isinstance(status.get("run_id"), str) else None,
        waypoint_index=(int(index) if isinstance(index, int)
                        and not isinstance(index, bool) else None),
        waypoint_count=_safe_int(status.get("waypoint_count", 0)),
        distance_remaining_m=_safe_float(status.get("distance_remaining_m")),
        eta_s=_safe_float(status.get("eta_s")),
        error=str(status.get("error") or ""),
        mode=status.get("mode") if isinstance(status.get("mode"), str) else None,
        coordinator_state=(status.get("coordinator_state")
                           if isinstance(status.get("coordinator_state"), str) else None))


def _safe_points(value) -> list:
    """Every well-formed [x, y] pair, and nothing else.

    A malformed pair is dropped rather than failing the whole message: a
    plan with one bad vertex is still worth drawing, and a canvas that
    blanks itself because of one is worse than one that is short a point.
    """
    if not isinstance(value, list):
        return []
    points = []
    for item in value:
        if (isinstance(item, list) and len(item) == 2
                and all(isinstance(v, (int, float)) and not isinstance(v, bool)
                        for v in item)):
            points.append((float(item[0]), float(item[1])))
    return points


def parse_path_summary(payload: str):
    try:
        summary = json.loads(payload)
    except (json.JSONDecodeError, TypeError):
        return None
    if not isinstance(summary, dict):
        return None
    return PathSummary(
        run_id=summary.get("run_id") if isinstance(summary.get("run_id"), str) else None,
        frame_id=str(summary.get("frame_id", "map")),
        points=_safe_points(summary.get("points")),
        waypoints=_safe_points(summary.get("waypoints")),
        length_m=_safe_float(summary.get("length_m"), 0.0),
        source_points=_safe_int(summary.get("source_points", 0)))


def parse_waypoint_text(x_text: str, y_text: str, yaw_text: str = ""):
    """(Waypoint, "") or (None, reason). The reason is shown on the row."""
    try:
        x = float(str(x_text).strip())
    except (TypeError, ValueError):
        return None, f"x is not a number: {str(x_text)!r}"
    try:
        y = float(str(y_text).strip())
    except (TypeError, ValueError):
        return None, f"y is not a number: {str(y_text)!r}"
    yaw = None
    if str(yaw_text).strip():
        try:
            yaw = float(str(yaw_text).strip())
        except (TypeError, ValueError):
            return None, f"yaw is not a number: {str(yaw_text)!r}"
    if math.hypot(x, y) > MAX_WAYPOINT_RANGE_M:
        return None, (f"({x:.1f}, {y:.1f}) is beyond the "
                      f"{MAX_WAYPOINT_RANGE_M:.0f} m map")
    return Waypoint(x, y, yaw), ""


class WaypointList:
    """The operator's waypoint list, in order. No Qt: the row owns the
    widget, this owns the order - so reordering is testable without a
    screen and the row cannot drift from the list it sends."""

    def __init__(self):
        self._items: list = []

    @property
    def items(self) -> list:
        return list(self._items)

    def __len__(self) -> int:
        return len(self._items)

    def add(self, waypoint) -> None:
        self._items.append(waypoint)

    def remove(self, index: int) -> None:
        if 0 <= index < len(self._items):
            del self._items[index]

    def clear(self) -> None:
        self._items = []

    def move_up(self, index: int) -> None:
        if 0 < index < len(self._items):
            self._items[index - 1], self._items[index] = (
                self._items[index], self._items[index - 1])

    def move_down(self, index: int) -> None:
        if 0 <= index < len(self._items) - 1:
            self._items[index + 1], self._items[index] = (
                self._items[index], self._items[index + 1])


@dataclass(frozen=True)
class ViewTransform:
    """World (map frame, metres) <-> canvas pixels, seen from above.

    World +x is UP the screen and world +y is LEFT: the map frame viewed
    from above, which is the operator's mental model of the rover's own
    axes. Rotating that into screen coordinates instead would make every
    clicked waypoint arrive somewhere the operator did not point at.
    """
    centre_x: float
    centre_y: float
    metres_per_pixel: float
    width_px: int
    height_px: int

    def to_pixel(self, x: float, y: float) -> tuple:
        return (self.width_px / 2.0 - (y - self.centre_y) / self.metres_per_pixel,
                self.height_px / 2.0 - (x - self.centre_x) / self.metres_per_pixel)

    def to_world(self, px: float, py: float) -> tuple:
        return (self.centre_x + (self.height_px / 2.0 - py) * self.metres_per_pixel,
                self.centre_y + (self.width_px / 2.0 - px) * self.metres_per_pixel)

    def zoomed(self, factor: float) -> "ViewTransform":
        return ViewTransform(self.centre_x, self.centre_y,
                             self.metres_per_pixel * factor,
                             self.width_px, self.height_px)

    def centred_on(self, x: float, y: float) -> "ViewTransform":
        return ViewTransform(x, y, self.metres_per_pixel,
                             self.width_px, self.height_px)

    def resized(self, width_px: int, height_px: int) -> "ViewTransform":
        return ViewTransform(self.centre_x, self.centre_y,
                             self.metres_per_pixel, width_px, height_px)
