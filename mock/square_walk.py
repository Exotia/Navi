"""A fake rover pose that walks a closed square.

Used by two fixtures that share nothing else: mock/ros_bridge.py serves it
to the ground station over websockets, and mock/fake_localization.py
publishes it on a real DDS domain for sim_bridge and Gazebo. "Gazebo walks
the same square as the panel" is only a real check if both ends read the
same code, which is why this is its own module rather than two similar
loops.

Standard library only, on purpose: the ground station's venv cannot import
ROS, and ROS's python3 is not the venv. This module has to load in both.
"""

import json
import math


def square_pose(elapsed: float, side: float = 4.0,
                speed: float = 0.5) -> tuple[float, float, float]:
    """Where the fake rover is at `elapsed` seconds, and which way it faces.

    Anticlockwise from the origin: +X, +Y, -X, -Y, turning instantly at each
    corner. The turn is not simulated because nothing downstream learns
    anything from it - what this fixture is for is a path that closes, so a
    drift anywhere in the chain shows up as the picture walking away from
    the square instead of as a number somebody has to read.

    Repeats forever: a fixture that stops after one lap is a fixture that
    has to be restarted every 32 seconds during a manual check.
    """
    leg_seconds = side / speed
    lap_seconds = 4.0 * leg_seconds
    t = elapsed % lap_seconds
    leg = int(t // leg_seconds)
    along = (t - leg * leg_seconds) * speed
    if leg == 0:
        return along, 0.0, 0.0
    if leg == 1:
        return side, along, math.pi / 2.0
    if leg == 2:
        return side - along, side, math.pi
    return 0.0, side - along, -math.pi / 2.0


def odometry_message(x: float, y: float, yaw: float) -> dict:
    """A nav_msgs/Odometry in the JSON shape rosbridge delivers.

    map -> base_footprint, matching what localization_status publishes on
    the rover. The covariance is left out: rosbridge fills absent fields
    with zeros, and this fixture has no uncertainty to claim either way.
    """
    return {
        "header": {"frame_id": "map"},
        "child_frame_id": "base_footprint",
        "pose": {"pose": {
            "position": {"x": x, "y": y, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0,
                            "z": math.sin(yaw / 2.0), "w": math.cos(yaw / 2.0)},
        }},
    }


def status_payload(state: str, seconds_since_ok: float | None,
                   distance_travelled: float) -> str:
    """The /localization/status JSON, exactly as localization_status
    publishes it, so the ground station's parser is exercised for real."""
    return json.dumps({
        "state": state,
        "seconds_since_ok": seconds_since_ok,
        "source": "zed_vio",
        "distance_travelled": distance_travelled,
        "mount_offset_verified": True,
    })
