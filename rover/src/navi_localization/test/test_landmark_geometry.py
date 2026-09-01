import math
import subprocess
import sys

import pytest

from navi_localization.landmark_geometry import (
    CAMERA_OPTICAL_IN_BASE_FOOTPRINT, Intrinsics, SightingAccumulator,
    apply_face_offset, depth_to_range, landmark_point_in_map, median_depth,
    ray_in_optical, rescale_pixel)
from navi_localization.pose_composition import IDENTITY, Transform


def quat_z(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def make_intrinsics(width=1280, height=720, fx=530.0, fy=530.0):
    return Intrinsics(fx=fx, fy=fy, cx=width / 2.0, cy=height / 2.0,
                       width=width, height=height)


def test_scaled_to_halves_everything_for_a_downscaled_stream():
    intr = make_intrinsics(width=1280, height=720, fx=530.0, fy=530.0)
    half = intr.scaled_to(640, 360)
    assert half.fx == pytest.approx(265.0)
    assert half.fy == pytest.approx(265.0)
    assert half.cx == pytest.approx(320.0)
    assert half.cy == pytest.approx(180.0)
    assert half.width == 640
    assert half.height == 360


def test_scaled_to_the_same_size_is_a_no_op():
    intr = make_intrinsics()
    same = intr.scaled_to(intr.width, intr.height)
    assert same == intr


def test_rescale_pixel_maps_centre_to_centre():
    u, v = rescale_pixel(320.0, 180.0, (640, 360), (1280, 720))
    assert (u, v) == pytest.approx((640.0, 360.0))


def test_rescale_pixel_maps_a_corner_to_the_corner():
    u, v = rescale_pixel(640.0, 360.0, (640, 360), (1280, 720))
    assert (u, v) == pytest.approx((1280.0, 720.0))


def test_ray_at_the_principal_point_is_straight_ahead():
    intr = make_intrinsics()
    assert ray_in_optical(intr.cx, intr.cy, intr) == pytest.approx((0.0, 0.0, 1.0))


def test_ray_off_centre_is_still_unit_length():
    intr = make_intrinsics()
    rx, ry, rz = ray_in_optical(intr.cx + 100.0, intr.cy - 40.0, intr)
    assert math.sqrt(rx * rx + ry * ry + rz * rz) == pytest.approx(1.0)


def test_depth_to_range_equals_depth_at_the_principal_point():
    intr = make_intrinsics()
    assert depth_to_range(intr.cx, intr.cy, 4.0, intr) == pytest.approx(4.0)


def test_depth_to_range_off_centre_scales_by_one_over_cos_angle():
    intr = make_intrinsics()
    u = intr.cx + 150.0
    v = intr.cy
    depth = 5.0
    dx = (u - intr.cx) / intr.fx
    # rz of the unit ray is cos(theta); its reciprocal is exactly this.
    one_over_cos_theta = math.sqrt(1.0 + dx * dx)
    assert depth_to_range(u, v, depth, intr) == pytest.approx(depth * one_over_cos_theta)


def test_apply_face_offset_adds_along_the_ray():
    assert apply_face_offset(4.0, 0.125) == pytest.approx(4.125)
    assert apply_face_offset(4.0, 0.0) == pytest.approx(4.0)


def test_landmark_point_in_map_at_the_centre_pixel_is_straight_ahead_of_the_left_lens():
    intr = make_intrinsics()
    range_m = 3.0
    point = landmark_point_in_map(intr.cx, intr.cy, range_m, intr, IDENTITY)
    assert point[0] == pytest.approx(0.345 + range_m)
    assert point[2] == pytest.approx(0.548)
    assert point[1] == pytest.approx(CAMERA_OPTICAL_IN_BASE_FOOTPRINT.y)


def test_landmark_point_in_map_with_the_rover_yawed_90_degrees():
    intr = make_intrinsics()
    range_m = 3.0
    footprint_in_map = Transform(0.0, 0.0, 0.0, *quat_z(math.pi / 2))
    point = landmark_point_in_map(intr.cx, intr.cy, range_m, intr, footprint_in_map)
    # Facing +x rotated 90 deg puts "straight ahead" onto map +y.
    assert point[0] == pytest.approx(-CAMERA_OPTICAL_IN_BASE_FOOTPRINT.y, abs=1e-9)
    assert point[1] == pytest.approx(0.345 + range_m)
    assert point[2] == pytest.approx(0.548)


def test_face_offset_moves_the_landmark_further_along_the_same_bearing():
    intr = make_intrinsics()
    u, v = intr.cx + 60.0, intr.cy - 20.0
    depth_m = 4.0
    footprint_in_map = Transform(1.0, 2.0, 0.0, *quat_z(0.3))

    from navi_localization.pose_composition import compose
    optical_in_map = compose(footprint_in_map, CAMERA_OPTICAL_IN_BASE_FOOTPRINT)
    camera_pos = (optical_in_map.x, optical_in_map.y, optical_in_map.z)

    p_pole = landmark_point_in_map(u, v, depth_m, intr, footprint_in_map, offset_m=0.0)
    p_face = landmark_point_in_map(u, v, depth_m, intr, footprint_in_map, offset_m=0.125)

    dist = math.dist(p_pole, p_face)
    assert dist == pytest.approx(0.125)

    vec_pole = tuple(p_pole[i] - camera_pos[i] for i in range(3))
    vec_face = tuple(p_face[i] - camera_pos[i] for i in range(3))
    range_pole = math.sqrt(sum(c * c for c in vec_pole))
    range_face = math.sqrt(sum(c * c for c in vec_face))
    assert range_face - range_pole == pytest.approx(0.125)
    # Same bearing: the ratio scales every component identically.
    scale = range_face / range_pole
    for a, b in zip(vec_pole, vec_face):
        assert b == pytest.approx(a * scale)


def test_median_depth_drops_nan_inf_and_out_of_range():
    patch = [4.0, 4.1, float("nan"), float("inf"), -1.0, 100.0, 3.9]
    med, n, frac = median_depth(patch, min_m=0.3, max_m=10.0)
    assert med == pytest.approx(4.0)
    assert n == 3
    assert frac == pytest.approx(3 / 7)


def test_median_depth_all_invalid_returns_none():
    patch = [float("nan"), float("inf"), -5.0, 999.0]
    med, n, frac = median_depth(patch, min_m=0.3, max_m=10.0)
    assert med is None
    assert n == 0
    assert frac == 0.0


def test_median_depth_is_unmoved_by_a_single_wild_outlier():
    patch = [4.0, 4.1, 4.05, 4.2, 60.0]
    med, n, frac = median_depth(patch, min_m=0.3, max_m=100.0)
    assert med == pytest.approx(4.1, abs=0.2)
    assert n == 5


def test_sighting_accumulator_median_converges_under_symmetric_noise():
    acc = SightingAccumulator()
    noise = [-0.05, -0.02, 0.0, 0.02, 0.05, -0.03, 0.03, 0.01, -0.01, 0.04]
    for i, dn in enumerate(noise):
        acc.add("51", 4.0 + dn, -1.0 + dn, 0.4, float(i))
    snap = acc.snapshot(now=100.0)
    assert len(snap) == 1
    s = snap[0]
    assert s.id == "51"
    assert s.x == pytest.approx(4.0, abs=0.06)
    assert s.y == pytest.approx(-1.0, abs=0.06)


def test_sighting_accumulator_ring_buffer_never_exceeds_max_samples():
    acc = SightingAccumulator(max_samples=5)
    for i in range(20):
        acc.add("51", float(i), 0.0, 0.0, float(i))
    snap = acc.snapshot(now=100.0)
    assert snap[0].n == 5


def test_sighting_accumulator_quality_weak_below_min_samples():
    acc = SightingAccumulator(min_samples=50, spread_warn_m=0.15)
    for i in range(10):
        acc.add("51", 4.0, -1.0, 0.4, float(i))
    snap = acc.snapshot(now=100.0)
    assert snap[0].quality == "weak"


def test_sighting_accumulator_quality_noisy_above_spread_warn():
    acc = SightingAccumulator(min_samples=5, spread_warn_m=0.05)
    values = [3.5, 4.5, 3.6, 4.4, 3.7, 4.3, 3.8, 4.2]
    for i, x in enumerate(values):
        acc.add("51", x, 0.0, 0.0, float(i))
    snap = acc.snapshot(now=100.0)
    assert snap[0].quality == "noisy"


def test_sighting_accumulator_weak_wins_over_noisy_when_both_apply():
    acc = SightingAccumulator(min_samples=50, spread_warn_m=0.01)
    values = [3.5, 4.5, 3.6, 4.4]
    for i, x in enumerate(values):
        acc.add("51", x, 0.0, 0.0, float(i))
    snap = acc.snapshot(now=100.0)
    assert snap[0].n < 50
    assert snap[0].quality == "weak"


def test_sighting_accumulator_last_seen_s_is_now_minus_t_last():
    acc = SightingAccumulator()
    acc.add("51", 4.0, -1.0, 0.4, 10.0)
    acc.add("51", 4.0, -1.0, 0.4, 12.5)
    snap = acc.snapshot(now=20.0)
    assert snap[0].last_seen_s == pytest.approx(7.5)


def test_sighting_accumulator_reset_empties_it():
    acc = SightingAccumulator()
    acc.add("51", 4.0, -1.0, 0.4, 0.0)
    acc.reset()
    assert acc.ids() == []
    assert acc.snapshot(now=10.0) == []


def test_import_pulls_in_no_ros_or_opencv():
    """A FRESH interpreter, not this one.

    Asserting on this process's `sys.modules` only tested the order pytest
    happened to collect files in: run beside test_site_probe.py (which
    imports rclpy at module scope, as a node test must), the very first
    assertion failed - not because landmark_geometry pulled anything in,
    but because a sibling test file already had. A subprocess is the only
    honest way to ask what one import costs.
    """
    source = (
        "import sys\n"
        "import navi_localization.landmark_geometry\n"
        "leaked = [m for m in ('rclpy', 'cv2', 'zed_msgs', 'numpy')\n"
        "          if m in sys.modules]\n"
        "print(','.join(leaked))\n"
    )
    result = subprocess.run([sys.executable, "-c", source],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == ""
