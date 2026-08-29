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


class DriveState:
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
