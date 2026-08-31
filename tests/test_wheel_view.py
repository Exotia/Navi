"""The steering picture. The angles come from the rover's own ICR
arithmetic, so these tests pin the physics, not a drawing."""

import math

from ground_station.ui.wheel_view import (IK_AVAILABLE, MOVING_EPS, WheelView,
                                          display_angles)


def deg(radians):
    return [round(math.degrees(a)) for a in radians]


def test_the_steering_model_is_the_rovers_own():
    # If this is ever False in this repo the widget silently degrades to
    # straight wheels, which would be a picture that lies quietly.
    assert IK_AVAILABLE, "navi_shaper.ik_geometry must be importable from the repo"


def test_driving_straight_points_every_wheel_along_the_body():
    assert all(abs(a) <= 2 for a in deg(display_angles(0.2, 0.0, 0.0)))


def test_the_inner_wheels_turn_further_than_the_outer_ones():
    # A left turn puts the ICR at positive y (left), so the left-hand
    # wheels are the inner pair and must be steered harder. Wheel order is
    # WHEEL_NAMES: front right, front left, rear left, rear right.
    front_right, front_left, rear_left, rear_right = deg(
        display_angles(0.2, 0.0, 0.2))
    assert front_left > front_right > 0
    assert rear_left < rear_right < 0
    # Mirrored for a right turn.
    fr, fl, rl, rr = deg(display_angles(0.2, 0.0, -0.2))
    assert fr < fl < 0
    assert rr > rl > 0


def test_a_point_turn_puts_the_wheels_in_an_x():
    # Turning in place: each wheel tangent to a circle about the centre, so
    # the front pair and rear pair are steered opposite ways at ~45 deg.
    front_right, front_left, rear_left, rear_right = deg(
        display_angles(0.0, 0.0, 0.2))
    assert 40 <= abs(front_left) <= 50 and 40 <= abs(front_right) <= 50
    assert front_left * front_right < 0        # opposite signs: an X
    assert rear_left * rear_right < 0


def test_every_drawn_angle_is_a_half_turn_not_a_wrapped_one():
    # The model reports point-turn angles like -225 deg; a wheel is a line,
    # so anything outside +-90 would draw it swung the long way round.
    for cmd in ((0.2, 0.0, 0.0), (0.2, 0.0, 0.2), (0.0, 0.0, 0.2),
                (-0.15, 0.0, -0.2), (0.05, 0.0, 0.05)):
        for angle in display_angles(*cmd):
            assert -math.pi / 2 - 1e-9 <= angle <= math.pi / 2 + 1e-9, cmd


def test_a_stopped_rover_keeps_the_geometry_it_last_had(qtbot):
    # The chassis holds its steering when the command goes to zero, and so
    # must the picture - snapping to a zero-speed ICR would show a turn the
    # wheels are not in.
    view = WheelView()
    qtbot.addWidget(view)
    view.set_twist(0.2, 0.0, 0.2)
    turning = view.angles
    assert view.moving
    view.set_twist(0.0, 0.0, 0.0)
    assert not view.moving
    assert view.angles == turning


def test_a_twist_below_the_epsilon_is_a_stop(qtbot):
    view = WheelView()
    qtbot.addWidget(view)
    view.set_twist(MOVING_EPS / 2, 0.0, 0.0)
    assert not view.moving


def test_the_view_paints_in_every_state(qtbot):
    # A paint that raises takes the whole window down with it.
    view = WheelView()
    qtbot.addWidget(view)
    view.resize(200, 160)
    view.grab()
    view.set_twist(0.2, 0.0, 0.2)
    view.grab()
    view.set_twist(-0.1, 0.0, 0.0)
    view.grab()
