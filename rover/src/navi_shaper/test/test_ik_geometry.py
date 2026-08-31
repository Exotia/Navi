"""The IK geometry map, checked against the real 2.42 model's own output.

Every expected value here came from compiling sim/src/navi_sim_ik/vendor242
with kAsteropeHParams and reading ExtY_kinematics_T. Task 2 widens this into a
generated fixture; these anchors stay because a hand-readable test that fails
tells you more than a 300-row JSON diff.
"""
import math

import pytest

from navi_shaper import ik_geometry as g


def test_hparams_are_widened_float32_not_double():
    # The rover declares these `const float` and IkController widens them into
    # hParams. A double literal here would differ in the 8th decimal and the
    # steering angles would no longer be the rover's arithmetic.
    assert g.HPARAMS[0] == pytest.approx(0.455269992351532, abs=0.0)
    assert g.HPARAMS[1] == pytest.approx(-0.44385001063346863, abs=0.0)
    # wheel3y is 0.44285, not 0.44385 - the suspected upstream typo SP4
    # transcribed deliberately. If this ever equals HPARAMS[3], someone
    # "fixed" it and the sim no longer matches the rover.
    assert g.HPARAMS[5] != g.HPARAMS[3]


@pytest.mark.parametrize("vx,vy,wz,icr_x,icr_y", [
    (0.05, 0.0, 0.0, -0.0, 9.868766),        # "straight" is a 9.87 m arc
    (0.025, 0.0, 0.0, -0.0, 4.983400),       # halving the speed halves it
    (0.005, 0.0, 0.0, -0.0, 0.999867),       # and 1/10 speed gives a 1 m arc
    (0.45, 0.0, 0.0, -0.0, 47.340301),
    (0.05, 0.0, 0.1, -0.0, 0.476176),
    (0.30, 0.0, 0.20, -0.0, 1.462997),
    (0.0, 0.05, 0.0, -9.868766, 0.0),        # pure crab
    (0.0, 0.0, 0.1, -0.0, 0.0),              # point turn: ICR at the origin
    (0.0, 0.0, 0.0, -0.0, 0.0),              # zero maps there too
    (-0.15, 0.0, 0.0, -0.0, -26.852478),     # the reverse floor
    (0.05, 0.02, 0.07, -0.266664, 0.666627),
])
def test_icr_matches_the_model(vx, vy, wz, icr_x, icr_y):
    x, y = g.commanded_icr(vx, vy, wz)
    assert x == pytest.approx(icr_x, abs=1e-6)
    assert y == pytest.approx(icr_y, abs=1e-6)


def test_steering_angles_match_the_model_for_a_straight_command():
    betas = g.steering_angles(*g.commanded_icr(0.05, 0.0, 0.0))
    for got, want in zip(betas, [0.0441, 0.0483, -0.0483, -0.0441]):
        assert got == pytest.approx(want, abs=1e-4)


def test_steering_distance_is_modulo_pi():
    # A settled point turn reports [0.7981, -3.9397, -2.3424, -0.7981]: the
    # middle two are the symmetric values shifted by exactly -pi, because a
    # wheel at beta+pi spinning backwards is the same wheel. Comparing them
    # naively yields a difference of pi that is not motion.
    assert g.angle_distance(0.7981, 0.7981 - math.pi) == pytest.approx(0.0, abs=1e-12)
    assert g.angle_distance(0.0, math.pi / 2) == pytest.approx(math.pi / 2, abs=1e-12)
    assert 0.0 <= g.angle_distance(1.3, -2.9) <= math.pi / 2


def test_steering_distance_between_commands():
    straight = (0.05, 0.0, 0.0)
    point = (0.0, 0.0, 0.1)
    # The wheels swing 0.8475 rad between these two geometries.
    assert g.command_distance(straight, point) == pytest.approx(0.8475, abs=0.001)
    assert g.command_distance(straight, straight) == pytest.approx(0.0, abs=1e-12)
    # An all-zero twist maps to the point-turn ICR, so the map reports the same
    # 0.8475 rad sweep from it. Read this as a statement about the map, and
    # about the BOOT pose only: a freshly initialize()d model has
    # UnitDelay_DSTATE = [0,0,0] and its ICR really is the origin. It is NOT a
    # statement about stopping. kinematics.cpp's '<S1>/Retain Translation'
    # block (lines 798-824) substitutes the last non-zero command whenever the
    # twist is all zeros, so a stop mid-run holds the geometry it had and a
    # resume is free. shaper.py relies on both halves of that.
    assert g.command_distance((0.0, 0.0, 0.0), straight) == pytest.approx(0.8475, abs=0.001)


def test_a_curvature_sign_change_exceeds_one_tick_of_steering():
    # A genuine sign crossing: wz goes from +0.0025 to -0.0025, so the two
    # commands take opposite branches of sgn(eta_w)*delta and the ICR sweeps
    # from +9.87 m out through infinity and back to about -6.6 m. That is
    # 0.1378 rad of steering demanded in one 20 Hz message - more than the
    # 0.09 rad a tick can deliver - even though wz itself moved by 0.005.
    #
    # Both commands must be on OPPOSITE sides of zero. A pair like
    # (0.05, 0, 0.0025) -> (0.05, 0, 0) is not a crossing at all: both have
    # wz >= 0, both take the +delta branch, and the measured distance is
    # 0.0252 rad, comfortably below threshold.
    assert g.command_distance((0.05, 0.0, 0.0025), (0.05, 0.0, -0.0025)) > g.ONE_TICK_BETA
    assert g.command_distance((0.05, 0.0, 0.0025), (0.05, 0.0, -0.0025)) == \
        pytest.approx(0.1378, abs=0.001)
    # Same-side, and not a crossing: below threshold, as the comment above says.
    assert g.command_distance((0.05, 0.0, 0.0025), (0.05, 0.0, 0.0)) < g.ONE_TICK_BETA
    # Away from the crossing, the same sweep rate is comfortably inside it.
    assert g.command_distance((0.05, 0.0, 0.050), (0.05, 0.0, 0.051)) < g.ONE_TICK_BETA


def test_one_tick_of_steering_is_the_feasibility_threshold():
    assert g.ONE_TICK_BETA == pytest.approx(1.5 * 0.06, abs=1e-12)


def test_scaling_a_point_turn_preserves_the_geometry_exactly():
    full = g.commanded_icr(0.0, 0.0, 0.1)
    for k in (0.5, 0.2, 0.05, 0.001):
        assert g.commanded_icr(0.0, 0.0, 0.1 * k) == full


def test_scaling_a_straight_command_wrecks_the_geometry():
    # This is the finding the fidelity guard exists for: scaling 0.05 m/s
    # down to 20% turns a 9.87 m arc into a 2 m arc.
    _, full = g.commanded_icr(0.05, 0.0, 0.0)
    _, fifth = g.commanded_icr(0.01, 0.0, 0.0)
    assert full == pytest.approx(9.8688, abs=1e-3)
    assert fifth == pytest.approx(1.9995, abs=1e-3)
    assert g.command_distance((0.05, 0.0, 0.0), (0.01, 0.0, 0.0)) > 0.15
