"""Tests for ground_station/site_frame.py — the rigid 2D site->map fit."""

import math

import pytest

from ground_station.site_frame import (
    LandmarkPair,
    SiteFrameError,
    SiteTransform,
    map_to_site,
    reexpress_at_lock_pose,
    residuals,
    site_to_map,
    site_yaw_to_map_yaw,
    solve_site_to_map,
)


def _rotate(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y, s * x + c * y


def _apply(x, y, yaw, tx, ty):
    rx, ry = _rotate(x, y, yaw)
    return rx + tx, ry + ty


def _pairs(ids, site_pts, map_pts):
    return [
        LandmarkPair(id=i, site_x=sx, site_y=sy, map_x=mx, map_y=my)
        for i, (sx, sy), (mx, my) in zip(ids, site_pts, map_pts)
    ]


# --- 1. known rotation + translation, four points -------------------------


def test_known_rotation_and_translation_recovered_exactly():
    ids = ["a", "b", "c", "d"]
    site_pts = [(0.0, 0.0), (4.0, 0.0), (0.0, 3.0), (4.0, 3.0)]
    yaw0 = math.radians(37.0)
    tx0, ty0 = 5.5, -2.25
    map_pts = [_apply(x, y, yaw0, tx0, ty0) for x, y in site_pts]
    t = solve_site_to_map(_pairs(ids, site_pts, map_pts))

    assert t.x == pytest.approx(tx0, abs=1e-9)
    assert t.y == pytest.approx(ty0, abs=1e-9)
    assert t.yaw == pytest.approx(yaw0, abs=1e-9)
    assert t.rms_m == pytest.approx(0.0, abs=1e-9)
    assert t.n_points == 4
    assert t.ids == ("a", "b", "c", "d")


# --- 2. round trip ----------------------------------------------------------


def test_map_to_site_round_trips_site_to_map():
    ids = ["a", "b", "c", "d"]
    site_pts = [(0.0, 0.0), (4.0, 0.0), (0.0, 3.0), (4.0, 3.0)]
    map_pts = [_apply(x, y, math.radians(12.0), 1.0, 2.0) for x, y in site_pts]
    t = solve_site_to_map(_pairs(ids, site_pts, map_pts))

    mx, my = site_to_map(t, 3.0, -4.0)
    sx, sy = map_to_site(t, mx, my)
    assert sx == pytest.approx(3.0, abs=1e-9)
    assert sy == pytest.approx(-4.0, abs=1e-9)


# --- 3. exactly two points, matching separation -----------------------------


def test_two_points_with_matching_separation_solves_with_zero_rms():
    yaw0 = math.radians(20.0)
    tx0, ty0 = 3.0, -4.0
    site_pts = [(0.0, 0.0), (5.0, 0.0)]
    map_pts = [_apply(x, y, yaw0, tx0, ty0) for x, y in site_pts]
    t = solve_site_to_map(_pairs(["p1", "p2"], site_pts, map_pts))

    assert t.n_points == 2
    assert t.rms_m == pytest.approx(0.0, abs=1e-9)
    assert t.yaw == pytest.approx(yaw0, abs=1e-9)


# --- 4. two points, separation off by 0.2 m ---------------------------------


def test_two_points_baseline_error_gives_half_baseline_rms():
    # site separation is 5.0 m; the measured (map) separation is 5.2 m.
    # With only two points the fit aligns direction perfectly (atan2 of a
    # cross/dot sum over two centred, opposite vectors depends only on
    # direction, not magnitude), so the whole 0.2 m error is absorbed by
    # translation and split evenly between the two points from the shared
    # centroid: each point ends up 0.1 m from where the fit places it, so
    # rms_m = sqrt((0.1^2 + 0.1^2) / 2) = 0.1.
    pairs = _pairs(
        ["p1", "p2"],
        [(0.0, 0.0), (5.0, 0.0)],
        [(0.0, 0.0), (5.2, 0.0)],
    )
    t = solve_site_to_map(pairs)
    assert t.rms_m == pytest.approx(0.1, abs=1e-9)


# --- 5. collinear points are NOT degenerate ---------------------------------


def test_three_collinear_landmarks_are_not_degenerate():
    ids = ["a", "b", "c"]
    site_pts = [(0.0, 0.0), (2.0, 0.0), (5.0, 0.0)]
    yaw0 = math.radians(50.0)
    tx0, ty0 = 1.0, 2.0
    map_pts = [_apply(x, y, yaw0, tx0, ty0) for x, y in site_pts]

    t = solve_site_to_map(_pairs(ids, site_pts, map_pts))

    assert t.x == pytest.approx(tx0, abs=1e-9)
    assert t.y == pytest.approx(ty0, abs=1e-9)
    assert t.yaw == pytest.approx(yaw0, abs=1e-9)
    assert t.rms_m == pytest.approx(0.0, abs=1e-9)


# --- 6. one outlier among four ----------------------------------------------


def test_outlier_landmark_is_reported_and_can_be_dropped():
    ids = ["a", "b", "c", "d"]
    site_pts = [(0.0, 0.0), (4.0, 0.0), (0.0, 3.0), (4.0, 3.0)]
    yaw0 = math.radians(10.0)
    tx0, ty0 = 0.0, 0.0
    map_pts = [_apply(x, y, yaw0, tx0, ty0) for x, y in site_pts]
    # perturb the last landmark's map position
    px, py = map_pts[3]
    map_pts[3] = (px + 0.3, py - 0.2)
    injected = math.hypot(0.3, 0.2)

    pairs = _pairs(ids, site_pts, map_pts)
    t = solve_site_to_map(pairs)

    assert t.worst_id == "d"
    assert t.max_residual_m == pytest.approx(injected, rel=0.5)

    dropped = [p for p in pairs if p.id != "d"]
    t2 = solve_site_to_map(dropped)
    assert t2.rms_m == pytest.approx(0.0, abs=1e-9)


# --- 7. yaw wrapping ---------------------------------------------------------


def test_yaw_is_wrapped_to_minus_pi_pi():
    site_pts = [(0.0, 0.0), (5.0, 0.0)]
    yaw0 = math.radians(190.0)
    map_pts = [_apply(x, y, yaw0, 0.0, 0.0) for x, y in site_pts]
    t = solve_site_to_map(_pairs(["a", "b"], site_pts, map_pts))

    assert t.yaw == pytest.approx(math.radians(-170.0), abs=1e-9)


# --- 8. site_yaw_to_map_yaw ---------------------------------------------------


def test_site_yaw_to_map_yaw_adds_and_wraps():
    t = SiteTransform(
        x=0.0, y=0.0, yaw=math.radians(170.0), rms_m=0.0, max_residual_m=0.0,
        worst_id=None, n_points=2, scale_hint=1.0, ids=("a", "b"),
    )
    result = site_yaw_to_map_yaw(t, math.radians(170.0))
    assert result == pytest.approx(math.radians(-20.0), abs=1e-9)


# --- 9. edge cases from the §3.2 table ---------------------------------------


def test_fewer_than_two_landmarks_raises():
    pairs = _pairs(["a"], [(0.0, 0.0)], [(1.0, 1.0)])
    with pytest.raises(SiteFrameError, match="need at least 2 landmarks, got 1"):
        solve_site_to_map(pairs)


def test_duplicate_id_raises_naming_the_id():
    pairs = _pairs(
        ["51", "51"], [(0.0, 0.0), (4.0, 0.0)], [(0.0, 0.0), (4.0, 1.0)]
    )
    with pytest.raises(SiteFrameError, match="51"):
        solve_site_to_map(pairs)


def test_coincident_site_points_raise():
    pairs = _pairs(
        ["a", "b"], [(1.0, 1.0), (1.0, 1.0)], [(0.0, 0.0), (4.0, 0.0)]
    )
    with pytest.raises(SiteFrameError, match="same site position"):
        solve_site_to_map(pairs)


def test_coincident_map_points_raise():
    pairs = _pairs(
        ["a", "b"], [(0.0, 0.0), (4.0, 0.0)], [(1.0, 1.0), (1.0, 1.0)]
    )
    with pytest.raises(SiteFrameError, match="same map position"):
        solve_site_to_map(pairs)


def test_reflection_symmetric_points_are_the_true_degeneracy():
    # Four points, individually well separated from their centroid (spread
    # 1.0 on both sides), but related by a reflection rather than any single
    # rotation: the numerator/denominator sums used for atan2 both vanish,
    # which is the one genuine degeneracy beyond simple coincidence.
    ids = ["a", "b", "c", "d"]
    site_pts = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)]
    map_pts = [(1.0, 0.0), (0.0, -1.0), (-1.0, 0.0), (0.0, 1.0)]
    with pytest.raises(SiteFrameError, match="same"):
        solve_site_to_map(_pairs(ids, site_pts, map_pts))


def test_non_finite_coordinate_raises_naming_the_id():
    pairs = _pairs(
        ["51", "52"],
        [(0.0, 0.0), (4.0, 0.0)],
        [(0.0, 0.0), (float("nan"), 1.0)],
    )
    with pytest.raises(SiteFrameError, match="51|52"):
        solve_site_to_map(pairs)


# --- 10. scale_hint -----------------------------------------------------------


def test_scale_hint_is_one_for_a_clean_fit():
    ids = ["a", "b", "c", "d"]
    site_pts = [(0.0, 0.0), (4.0, 0.0), (0.0, 3.0), (4.0, 3.0)]
    map_pts = [_apply(x, y, math.radians(15.0), 1.0, 1.0) for x, y in site_pts]
    t = solve_site_to_map(_pairs(ids, site_pts, map_pts))
    assert t.scale_hint == pytest.approx(1.0, abs=1e-9)


def test_scale_hint_is_1_1_when_measured_baselines_are_ten_percent_long():
    ids = ["a", "b", "c", "d"]
    site_pts = [(0.0, 0.0), (4.0, 0.0), (0.0, 3.0), (4.0, 3.0)]
    yaw0 = math.radians(15.0)
    tx0, ty0 = 1.0, 1.0

    def scaled_apply(x, y):
        rx, ry = _rotate(x, y, yaw0)
        return rx * 1.1 + tx0, ry * 1.1 + ty0

    map_pts = [scaled_apply(x, y) for x, y in site_pts]
    t = solve_site_to_map(_pairs(ids, site_pts, map_pts))
    assert t.scale_hint == pytest.approx(1.1, abs=1e-6)


# --- 11. non-finite input never produces a NaN transform ---------------------


def test_infinite_coordinate_raises_rather_than_producing_nan():
    pairs = _pairs(
        ["a", "b"],
        [(0.0, 0.0), (float("inf"), 0.0)],
        [(0.0, 0.0), (4.0, 1.0)],
    )
    with pytest.raises(SiteFrameError):
        solve_site_to_map(pairs)


# --- reexpress_at_lock_pose (review round 3) ---------------------------------


def _valid_transform():
    ids = ["a", "b", "c", "d"]
    site_pts = [(0.0, 0.0), (4.0, 0.0), (0.0, 3.0), (4.0, 3.0)]
    map_pts = [_apply(x, y, math.radians(37.0), 5.5, -2.25) for x, y in site_pts]
    return solve_site_to_map(_pairs(ids, site_pts, map_pts))


def test_reexpress_identity_pose_returns_t_field_for_field():
    t = _valid_transform()
    t2 = reexpress_at_lock_pose(t, 0.0, 0.0, 0.0)

    assert t2.x == pytest.approx(t.x, abs=1e-9)
    assert t2.y == pytest.approx(t.y, abs=1e-9)
    assert t2.yaw == pytest.approx(t.yaw, abs=1e-9)
    assert t2.rms_m == t.rms_m
    assert t2.max_residual_m == t.max_residual_m
    assert t2.worst_id == t.worst_id
    assert t2.n_points == t.n_points
    assert t2.scale_hint == t.scale_hint
    assert t2.ids == t.ids


def test_reexpress_pure_rotation_rotates_a_converted_waypoint_by_that_angle():
    t = _valid_transform()
    pose_yaw = math.radians(30.0)
    t2 = reexpress_at_lock_pose(t, 0.0, 0.0, pose_yaw)

    wx, wy = 3.0, -1.5
    old_map = site_to_map(t, wx, wy)
    new_map = site_to_map(t2, wx, wy)

    c, s = math.cos(-pose_yaw), math.sin(-pose_yaw)
    expected = (c * old_map[0] - s * old_map[1], s * old_map[0] + c * old_map[1])
    assert new_map[0] == pytest.approx(expected[0], abs=1e-9)
    assert new_map[1] == pytest.approx(expected[1], abs=1e-9)


def test_reexpress_roundtrip_matches_manual_rebasing_of_landmark_position():
    t = _valid_transform()
    pose_x, pose_y, pose_yaw = 1.2, -0.7, math.radians(15.0)
    t2 = reexpress_at_lock_pose(t, pose_x, pose_y, pose_yaw)

    site_x, site_y = 6.0, 2.0
    old_map = site_to_map(t, site_x, site_y)

    dx, dy = old_map[0] - pose_x, old_map[1] - pose_y
    c, s = math.cos(pose_yaw), math.sin(pose_yaw)
    rebased = (c * dx + s * dy, -s * dx + c * dy)

    new_map = site_to_map(t2, site_x, site_y)
    assert new_map[0] == pytest.approx(rebased[0], abs=1e-9)
    assert new_map[1] == pytest.approx(rebased[1], abs=1e-9)


# --- residuals() helper -------------------------------------------------------


def test_residuals_matches_solved_rms():
    ids = ["a", "b", "c", "d"]
    site_pts = [(0.0, 0.0), (4.0, 0.0), (0.0, 3.0), (4.0, 3.0)]
    map_pts = [_apply(x, y, math.radians(8.0), 2.0, 3.0) for x, y in site_pts]
    pairs = _pairs(ids, site_pts, map_pts)
    t = solve_site_to_map(pairs)

    rs = residuals(t, pairs)
    assert [r[0] for r in rs] == ids
    rms = math.sqrt(sum(r[1] ** 2 for r in rs) / len(rs))
    assert rms == pytest.approx(t.rms_m, abs=1e-9)
