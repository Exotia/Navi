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

# Front ZED 2i body centre in base_footprint: the URDF's
# zed_front_camera_joint (0.345, 0, 0.139 in base_link - the 1/4" mounting
# screw) plus base_footprint_joint's 0.409 m of height.
# tests/test_mount_offset_agrees_with_urdf.py in the Navi repository keeps
# the two in step, so a re-measurement has to change both together.
CAMERA_IN_BASE_FOOTPRINT = Transform(0.345, 0.0, 0.548, 0.0, 0.0, 0.0, 1.0)

# The numbers come from the hardware team's measured optical centres, which
# is what zed_front_camera_joint in the URDF carries. Flip this only if a
# re-measurement of camera centre to base_link disagrees with them.
MOUNT_OFFSET_VERIFIED = True

# base_link above base_footprint: the URDF's base_footprint_joint, which is
# the wheel axle at -0.284 plus the 0.125 m wheel radius. Nav2 needs the link
# to exist in TF; nothing on the rover computes with it.
# tests/test_mount_offset_agrees_with_urdf.py keeps this equal to the URDF.
BASE_LINK_IN_BASE_FOOTPRINT = Transform(0.0, 0.0, 0.409, 0.0, 0.0, 0.0, 1.0)


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


def _cross(a, b):
    ax, ay, az = a
    bx, by, bz = b
    return (ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx)


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


def footprint_twist_from_camera_twist(
        linear, angular,
        camera_in_footprint: Transform = CAMERA_IN_BASE_FOOTPRINT):
    """The camera's body twist, re-expressed at base_footprint.

    Both arguments are (x, y, z) in the camera's own axes - the ROS
    convention for nav_msgs/Odometry.twist, which is expressed in
    child_frame_id. The result is the same for base_footprint.

    Two points on one rigid body do not share a linear velocity: with R and p
    the mount's rotation and translation (camera in footprint),

        omega_footprint = R * omega_camera
        v_footprint     = R * v_camera + omega_footprint x (-p)

    Not an optional refinement. The camera sits 0.345 m ahead of and 0.548 m
    above base_footprint, so at a 0.5 rad/s yaw the two points differ by
    0.17 m/s sideways, which is the size of the speeds the controller works
    with.
    """
    q = (camera_in_footprint.qx, camera_in_footprint.qy,
         camera_in_footprint.qz, camera_in_footprint.qw)
    vx, vy, vz = _rotate(q, tuple(linear))
    omega = _rotate(q, tuple(angular))
    lx, ly, lz = _cross(omega, (-camera_in_footprint.x,
                                -camera_in_footprint.y,
                                -camera_in_footprint.z))
    return (vx + lx, vy + ly, vz + lz), omega


def yaw_of(t: Transform) -> float:
    siny_cosp = 2.0 * (t.qw * t.qz + t.qx * t.qy)
    cosy_cosp = 1.0 - 2.0 * (t.qy * t.qy + t.qz * t.qz)
    return math.atan2(siny_cosp, cosy_cosp)


def translation_distance(a: Transform, b: Transform) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


# The only two transforms the Orin publishes on /tf_static, parent to child.
#
# The ZED wrapper owns map -> odom -> zed_front_camera_link and must stay its
# only owner, so base_footprint is hung *below* the camera rather than above
# it: topologically upside down relative to the URDF, but one tree with one
# root, which is what Nav2's costmap lookups need. Both entries add children,
# never a second parent - that is the invariant, and test_pose_composition.py
# asserts it. A robot_state_publisher on the Orin would violate it.
# (The simulation's own bringup does run one, with the full URDF, publishing
# these two edges the other way up. That is fine only while the sim and the
# rover never share a ROS domain.)
STATIC_FRAMES = (
    ('zed_front_camera_link', 'base_footprint', inverse(CAMERA_IN_BASE_FOOTPRINT)),
    ('base_footprint', 'base_link', BASE_LINK_IN_BASE_FOOTPRINT),
)
