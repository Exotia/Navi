"""Rigid-transform arithmetic for re-expressing the ZED's pose at the rover.

The ZED ROS 2 wrapper (4.2) tracks a frame it insists on calling
`<camera_name>_camera_link`, and it publishes the TF for it. The rover's
own frame, base_footprint, cannot be hung *above* that in TF without giving
zed_front_camera_link two parents, so instead the pose is re-expressed here in
plain arithmetic and published as its own message. Pure Python on purpose:
no tf2, no numpy, so it is testable on any machine and the mount offset is
one constant in one file.

Quaternions are (x, y, z, w), ROS order.
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Transform:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


IDENTITY = Transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

# Front ZED 2i body centre in base_footprint: the URDF's zed_camera_joint
# (zed_front_camera_joint: 0.345, 0, 0.139 in base_link, the 1/4" mounting screw) plus base_footprint_joint's 0.409 m of
# height. tests/test_mount_offset_agrees_with_urdf.py in the Navi repository
# keeps the two in step.
CAMERA_IN_BASE_FOOTPRINT = Transform(0.345, 0.0, 0.548, 0.0, 0.0, 0.0, 1.0)

# The offset above was read off a URDF visual that was authored from
# photographs. Flip this only after measuring camera centre to base_link.
MOUNT_OFFSET_VERIFIED = True


def _quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate(q, v):
    """Rotate vector v by unit quaternion q: q * v * q^-1."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    # t = 2 * cross(q.xyz, v)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    # v' = v + w * t + cross(q.xyz, t)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def compose(a: Transform, b: Transform) -> Transform:
    """a * b: apply b, then a. If a is X-in-W and b is Y-in-X, the result
    is Y-in-W."""
    qa = (a.qx, a.qy, a.qz, a.qw)
    rx, ry, rz = _rotate(qa, (b.x, b.y, b.z))
    qx, qy, qz, qw = _quat_multiply(qa, (b.qx, b.qy, b.qz, b.qw))
    return Transform(a.x + rx, a.y + ry, a.z + rz, qx, qy, qz, qw)


def inverse(t: Transform) -> Transform:
    q_inv = (-t.qx, -t.qy, -t.qz, t.qw)
    x, y, z = _rotate(q_inv, (-t.x, -t.y, -t.z))
    return Transform(x, y, z, *q_inv)


def footprint_pose_from_camera_pose(
        camera_in_map: Transform,
        camera_in_footprint: Transform = CAMERA_IN_BASE_FOOTPRINT) -> Transform:
    """T_map_footprint = T_map_camera * inverse(T_footprint_camera)."""
    return compose(camera_in_map, inverse(camera_in_footprint))


def yaw_of(t: Transform) -> float:
    siny_cosp = 2.0 * (t.qw * t.qz + t.qx * t.qy)
    cosy_cosp = 1.0 - 2.0 * (t.qy * t.qy + t.qz * t.qz)
    return math.atan2(siny_cosp, cosy_cosp)


def translation_distance(a: Transform, b: Transform) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
