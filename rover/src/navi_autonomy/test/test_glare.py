"""Glare detection and detour geometry on synthetic frames - pure numpy, no
ROS.

  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization \
    python3 -m pytest rover/src/navi_autonomy/test/test_glare.py -q'
"""
import math

import numpy as np
import pytest

from navi_autonomy.glare import (
    GLARE_FRACTION, GLARE_MARGIN, SATURATION_LEVEL, DetourPlanner,
    detour_point, glare_side, saturated_fractions)


# -- saturated_fractions -----------------------------------------------------

def test_the_thresholds_are_blunt_enough_that_ordinary_brightness_is_not_sun():
    # Raised after a live run tacked with no strong sun in the sky: 2 per
    # cent of a half frame is a wet stone or a painted line, and a detour
    # costs minutes of mission clock.
    assert SATURATION_LEVEL == 250
    assert GLARE_FRACTION == 0.20
    assert GLARE_MARGIN == 3.0


def test_a_thumbnail_of_white_is_not_a_sun():
    # The exact case that misfired: a small bright patch on one side only.
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    image[:10, :10] = 255                       # 1 per cent of the left half

    left, right = saturated_fractions(image)

    assert left == pytest.approx(0.02)
    assert glare_side(left, right) is None


def test_a_sun_that_whites_out_a_third_of_one_half_is_still_a_sun():
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    image[:60, :30] = 255                       # 36 per cent of the left half

    left, right = saturated_fractions(image)

    assert glare_side(left, right) == 'left'


def test_a_colour_image_where_a_pixel_is_saturated_in_one_channel_only_does_not_count_as_saturated():
    image = np.zeros((4, 4, 3), dtype=np.uint8)
    image[0, 0] = (255, 0, 0)      # only the red channel is blown out
    left, right = saturated_fractions(image)
    assert left == pytest.approx(0.0)
    assert right == pytest.approx(0.0)


def test_an_odd_width_image_does_not_crash_and_does_not_count_the_centre_column_twice():
    image = np.zeros((4, 7), dtype=np.uint8)
    image[:, 3] = 255              # the centre column only, belongs to neither half
    left, right = saturated_fractions(image)
    assert left == pytest.approx(0.0)
    assert right == pytest.approx(0.0)


def test_an_empty_image_returns_zero_fractions():
    assert saturated_fractions(np.zeros((0, 10), dtype=np.uint8)) == (0.0, 0.0)
    assert saturated_fractions(np.zeros((10, 0), dtype=np.uint8)) == (0.0, 0.0)


# -- glare_side ---------------------------------------------------------------

def test_a_frame_saturated_only_on_the_left_names_left():
    image = np.zeros((10, 10), dtype=np.uint8)
    image[:, :5] = 255
    left, right = saturated_fractions(image)
    assert glare_side(left, right) == 'left'


def test_a_frame_saturated_only_on_the_right_names_right():
    image = np.zeros((10, 10), dtype=np.uint8)
    image[:, 5:] = 255
    left, right = saturated_fractions(image)
    assert glare_side(left, right) == 'right'


def test_a_frame_with_no_saturation_anywhere_names_none():
    image = np.zeros((10, 10), dtype=np.uint8)
    left, right = saturated_fractions(image)
    assert glare_side(left, right) is None


def test_a_uniformly_saturated_frame_names_none_because_there_is_no_side_to_steer_away_from():
    image = np.full((10, 10), 255, dtype=np.uint8)
    left, right = saturated_fractions(image)
    assert left == pytest.approx(right)
    assert glare_side(left, right) is None


def test_a_half_only_slightly_brighter_than_the_other_names_none():
    # 3% against 2.5%: both clear GLARE_FRACTION, but their ratio (1.2) is
    # under GLARE_MARGIN (1.5), so neither is a direction worth steering by.
    assert glare_side(0.03, 0.025) is None


# -- detour_point ---------------------------------------------------------------

def test_detour_point_offsets_to_the_opposite_side_of_the_named_glare():
    rover = (0.0, 0.0)
    goal = (10.0, 0.0)

    # Glare on the left means steer right: away from it, toward -y.
    left_glare_point = detour_point(rover, goal, 'left', offset_m=2.0, along_fraction=0.5)
    assert left_glare_point[0] == pytest.approx(5.0)
    assert left_glare_point[1] == pytest.approx(-2.0)

    # Glare on the right means steer left, toward +y.
    right_glare_point = detour_point(rover, goal, 'right', offset_m=2.0, along_fraction=0.5)
    assert right_glare_point[0] == pytest.approx(5.0)
    assert right_glare_point[1] == pytest.approx(2.0)


def test_detour_point_sits_the_documented_perpendicular_distance_off_a_diagonal_leg():
    rover = (1.0, 1.0)
    goal = (4.0, 5.0)              # a leg not aligned to either axis
    point = detour_point(rover, goal, 'left', offset_m=2.0, along_fraction=0.5)

    # Perpendicular distance from `point` to the infinite rover->goal line:
    # the magnitude of the 2-D cross product of the leg vector and the
    # rover->point vector, divided by the leg's length.
    dx, dy = goal[0] - rover[0], goal[1] - rover[1]
    length = math.hypot(dx, dy)
    cross = dx * (point[1] - rover[1]) - dy * (point[0] - rover[0])
    perpendicular_distance = abs(cross) / length
    assert perpendicular_distance == pytest.approx(2.0)


def test_detour_point_with_side_none_returns_the_goal_unchanged():
    goal = (3.0, 4.0)
    assert detour_point((0.0, 0.0), goal, None) == goal


def test_detour_point_with_rover_already_at_the_goal_returns_the_goal_unchanged():
    goal = (5.0, 5.0)
    assert detour_point((5.0, 5.0), goal, 'left') == goal


# -- DetourPlanner ---------------------------------------------------------------

def test_the_planner_returns_detours_until_max_detours_is_reached_then_the_real_goal():
    planner = DetourPlanner(offset_m=2.0, along_fraction=0.5, max_detours=2)
    rover = (0.0, 0.0)
    goal = (10.0, 0.0)

    point1, is_detour1 = planner.next_target(rover, goal, 'left')
    point2, is_detour2 = planner.next_target(rover, goal, 'left')
    point3, is_detour3 = planner.next_target(rover, goal, 'left')

    assert is_detour1 is True and point1 != goal
    assert is_detour2 is True and point2 != goal
    assert is_detour3 is False
    assert point3 == goal
    assert planner.detours_taken == 2


def test_begin_leg_resets_the_counter():
    planner = DetourPlanner(max_detours=1)
    rover = (0.0, 0.0)
    goal = (10.0, 0.0)

    planner.next_target(rover, goal, 'left')
    assert planner.detours_taken == 1

    planner.begin_leg()
    assert planner.detours_taken == 0

    _, is_detour = planner.next_target(rover, goal, 'left')
    assert is_detour is True


def test_the_planner_with_rover_xy_none_returns_the_real_goal():
    planner = DetourPlanner()
    goal = (10.0, 0.0)

    point, is_detour = planner.next_target(None, goal, 'left')

    assert point == goal
    assert is_detour is False
    assert planner.detours_taken == 0


def test_a_planner_at_the_goal_does_not_call_the_real_goal_a_detour():
    # detour_point declines when there is no direction to offset from, and
    # calling its answer a detour would send the real goal to Nav2 under the
    # detour callbacks - the run would learn nothing from its arrival.
    planner = DetourPlanner()

    point, is_detour = planner.next_target((4.0, 4.0), (4.0, 4.0), 'left')

    assert point == (4.0, 4.0)
    assert is_detour is False
    assert planner.detours_taken == 0
