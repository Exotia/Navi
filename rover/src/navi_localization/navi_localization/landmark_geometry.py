"""A pixel plus a depth becomes a landmark in the map frame.

This is the shared arithmetic of site-anchor stages 2 (the manual depth
probe) and 3 (the ArUco anchor phase): rescale a clicked pixel onto the
depth image's own size, cast a ray through it using the camera's pinhole
intrinsics, turn a z-depth into a range along that ray, correct the range
for the 0.125 m pole-axis offset (D4 — the camera-to-marker ray stands in
for the true inward face normal, so the whole correction collapses to
adding a distance along the same ray), and walk the resulting point through
base_footprint into map with `navi_localization.pose_composition`.

Pure Python on purpose, the same way `pose_composition.py` is: no `rclpy`,
no `cv2`, no `zed_msgs`, no numpy. `math` and `dataclasses` only, so this
imports on a laptop with no ROS sourced at all and is fully testable there.
"""

import math
from dataclasses import dataclass
from typing import Sequence

from navi_localization.pose_composition import Transform, transform_point

# Half the ZED 2i's 120 mm stereo baseline: depth and the rectified left
# image are both expressed at the LEFT lens, not the body centre that
# CAMERA_IN_BASE_FOOTPRINT names (that constant is the URDF's
# zed_front_camera_joint, the 1/4" mounting screw in the middle of the bar).
# The left lens sits ZED_BASELINE_M / 2 to the left of that centre, i.e.
# +0.060 m in base_footprint y. Systematic, not noise, so it is a named
# term rather than a magic number folded into the constant below.
ZED_BASELINE_M = 0.120

# The LEFT optical frame (z forward, x right, y down - the ROS camera_optical
# convention) expressed in base_footprint.
#
# Rotation: the fixed optical->body axis remap. In the optical frame,
# +z is forward, +x is right, +y is down; in base_footprint, +x is forward,
# +y is left, +z is up. So a vector's optical z becomes the footprint's x,
# its optical x becomes the footprint's -y, and its optical y becomes the
# footprint's -z. That remap is the quaternion (-0.5, 0.5, -0.5, 0.5) - the
# same optical-frame-to-body-frame rotation used throughout ROS (REP 103).
#
# Translation: CAMERA_IN_BASE_FOOTPRINT's (x, z) - the mount is not
# re-measured here - shifted by ZED_BASELINE_M / 2 in y for the left lens.
CAMERA_OPTICAL_IN_BASE_FOOTPRINT = Transform(
    0.345, ZED_BASELINE_M / 2.0, 0.548, -0.5, 0.5, -0.5, 0.5)


@dataclass(frozen=True)
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def scaled_to(self, width: int, height: int) -> "Intrinsics":
        """Same camera, different published image size. All four numbers
        scale linearly; this is how a click on a 640x360 video stream is
        read against a 1280x720 depth image."""
        sx = width / self.width
        sy = height / self.height
        return Intrinsics(self.fx * sx, self.fy * sy,
                           self.cx * sx, self.cy * sy, width, height)


def rescale_pixel(u: float, v: float, from_wh, to_wh) -> tuple:
    """Map a pixel clicked on an image of size `from_wh` onto the
    equivalent pixel on an image of size `to_wh` of the same scene."""
    from_w, from_h = from_wh
    to_w, to_h = to_wh
    return u * to_w / from_w, v * to_h / from_h


def ray_in_optical(u: float, v: float, intr: Intrinsics) -> tuple:
    """Unit vector from the optical centre through pixel (u, v)."""
    x = (u - intr.cx) / intr.fx
    y = (v - intr.cy) / intr.fy
    z = 1.0
    length = math.sqrt(x * x + y * y + z * z)
    return (x / length, y / length, z / length)


def point_in_optical(u: float, v: float, range_m: float, intr: Intrinsics) -> tuple:
    """range_m along that unit ray. Note: `range_m` is a RANGE along the
    ray, not a z-depth. The ZED publishes z-depth in its depth image, so
    `depth_to_range` converts before this is called."""
    rx, ry, rz = ray_in_optical(u, v, intr)
    return (rx * range_m, ry * range_m, rz * range_m)


def depth_to_range(u: float, v: float, depth_m: float, intr: Intrinsics) -> float:
    """z-depth (what the depth image holds) -> range along the ray.

    The unit ray's own z-component is cos(theta), theta the angle off the
    optical axis, so dividing the z-depth by it is the same division a
    handwritten `depth_m / cos(theta)` would do.
    """
    _, _, rz = ray_in_optical(u, v, intr)
    return depth_m / rz


def apply_face_offset(range_m: float, offset_m: float) -> float:
    """The pole-axis correction of D4: the marker face is `offset_m` in
    FRONT of the pole axis, so the axis is `offset_m` FURTHER along the same
    ray. Because the inward normal is taken as the camera-to-marker ray, the
    whole correction is one addition. Pass 0.0 for a manual pole click."""
    return range_m + offset_m


def landmark_point_in_map(u: float, v: float, depth_m: float, intr: Intrinsics,
                           footprint_in_map: Transform,
                           offset_m: float = 0.0,
                           optical_in_footprint: Transform = CAMERA_OPTICAL_IN_BASE_FOOTPRINT
                           ) -> tuple:
    """The whole chain: pixel + z-depth -> range -> pole-axis range ->
    optical point -> base_footprint -> map."""
    range_m = depth_to_range(u, v, depth_m, intr)
    range_m = apply_face_offset(range_m, offset_m)
    p_optical = point_in_optical(u, v, range_m, intr)
    p_footprint = transform_point(optical_in_footprint, p_optical)
    return transform_point(footprint_in_map, p_footprint)


def _median(values: Sequence[float]) -> float:
    ordered = sorted(values)
    n = len(ordered)
    mid = n // 2
    if n % 2 == 1:
        return ordered[mid]
    return (ordered[mid - 1] + ordered[mid]) / 2.0


def median_depth(patch: Sequence[float], min_m: float, max_m: float
                  ) -> tuple:
    """(median, n_valid, valid_fraction) over a flat patch of depth values,
    dropping NaN, inf and anything outside [min_m, max_m]. Returns
    (None, n, frac) when nothing is valid. The median, not the mean: one
    background return through a gap beside a 6 cm pole would drag a mean
    metres away."""
    total = len(patch)
    valid = [d for d in patch if math.isfinite(d) and min_m <= d <= max_m]
    n = len(valid)
    if n == 0:
        return (None, 0, 0.0)
    fraction = n / total if total else 0.0
    return (_median(valid), n, fraction)


@dataclass(frozen=True)
class AccumulatedSighting:
    id: str
    x: float
    y: float
    z: float
    n: int
    spread_m: float
    last_seen_s: float
    quality: str


class SightingAccumulator:
    """Per-id ring buffer of accumulated map-frame landmark detections,
    for the stage-3 ArUco anchor phase (§3.5)."""

    def __init__(self, max_samples: int = 150,
                 min_samples: int = 50,
                 spread_warn_m: float = 0.15):
        self._max_samples = max_samples
        self._min_samples = min_samples
        self._spread_warn_m = spread_warn_m
        self._points = {}

    def add(self, id: str, x: float, y: float, z: float, t: float) -> None:
        buf = self._points.setdefault(id, [])
        buf.append((x, y, z, t))
        if len(buf) > self._max_samples:
            del buf[0]

    def reset(self) -> None:
        self._points = {}

    def ids(self):
        return list(self._points.keys())

    def snapshot(self, now: float):
        out = []
        for id_ in self._points:
            buf = self._points[id_]
            mx = _median([p[0] for p in buf])
            my = _median([p[1] for p in buf])
            mz = _median([p[2] for p in buf])
            dists = [math.hypot(p[0] - mx, p[1] - my) for p in buf]
            spread = 1.4826 * _median(dists)
            n = len(buf)
            last_seen = now - max(p[3] for p in buf)
            if n < self._min_samples:
                quality = "weak"
            elif spread > self._spread_warn_m:
                quality = "noisy"
            else:
                quality = "good"
            out.append(AccumulatedSighting(id_, mx, my, mz, n, spread, last_seen, quality))
        return out
