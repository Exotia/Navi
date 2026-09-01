"""The gain policy: pure, clock-injected, no ROS."""
import math

import pytest

from navi_shaper import ik_geometry as g
from navi_shaper.shaper import ShaperConfig, TwistShaper


def make(**kw):
    return TwistShaper(ShaperConfig(**kw))


# --- the safety properties, first ------------------------------------------

def test_zero_passes_through_untouched_and_unheld():
    s = make()
    s.shape(0.05, 0.0, 0.0, dt=0.05)
    s.shape(0.0, 0.0, 0.1, dt=0.05)          # open a hold window
    out = s.shape(0.0, 0.0, 0.0, dt=0.05)    # the estop / deadman stream
    assert (out.vx, out.vy, out.wz) == (0.0, 0.0, 0.0)
    assert out.gain == 1.0
    assert out.limited_by == "none"


def test_a_zero_stream_never_becomes_nonzero():
    s = make()
    for _ in range(200):
        out = s.shape(0.0, 0.0, 0.0, dt=0.05)
        assert (out.vx, out.vy, out.wz) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize("vx,vy,wz", [
    (0.05, 0.0, 0.0), (0.0, 0.0, 0.1), (0.05, 0.0, 0.1), (-0.15, 0.0, 0.0),
    (0.0, 0.05, 0.0), (0.45, 0.0, 0.4), (0.02, -0.01, -0.08), (0.3, 0.0, -0.2),
])
def test_output_is_never_larger_than_input_componentwise(vx, vy, wz):
    s = make()
    # Drive a sequence that forces every branch: settled, jumped, recovering.
    for cmd in [(vx, vy, wz), (0.0, 0.0, 0.1), (vx, vy, wz), (0.05, 0.0, 0.0),
                (vx, vy, wz)] * 8:
        out = s.shape(*cmd, dt=0.05)
        assert abs(out.vx) <= abs(cmd[0]) + 1e-15
        assert abs(out.vy) <= abs(cmd[1]) + 1e-15
        assert abs(out.wz) <= abs(cmd[2]) + 1e-15
        assert out.vx * cmd[0] >= 0.0 and out.vy * cmd[1] >= 0.0
        assert out.wz * cmd[2] >= 0.0
        assert 0.0 <= out.gain <= 1.0


def test_the_gain_is_uniform_across_the_three_components():
    """A non-uniform scale would change the geometry, which is the one thing
    the shaper must not do silently."""
    s = make()
    s.shape(0.05, 0.02, 0.07, dt=0.05)
    s.shape(0.0, 0.0, 0.1, dt=0.05)
    out = s.shape(0.05, 0.02, 0.07, dt=0.05)
    assert out.vx == pytest.approx(0.05 * out.gain, rel=1e-12)
    assert out.vy == pytest.approx(0.02 * out.gain, rel=1e-12)
    assert out.wz == pytest.approx(0.07 * out.gain, rel=1e-12)


# --- the policy ------------------------------------------------------------

def settle(s, cmd, seconds=6.0):
    """Hold one command until any hold window has closed.

    The longest hold is 60 ticks = 3.6 s, and the first message after
    construction always opens one - reset() starts _last_cmd at (0,0,0),
    modelling a freshly initialize()d model whose UnitDelay_DSTATE is still
    zero and whose ICR is therefore genuinely the origin. That is a BOOT
    condition, not a stop condition: mid-run, Retain Translation keeps the
    geometry across a zero twist. Either way, 6 s of settling is needed before
    any test that wants a quiescent starting state.
    """
    for _ in range(int(seconds / 0.05)):
        out = s.shape(*cmd, dt=0.05)
    return out


def test_a_settled_command_is_a_pure_relay():
    s = make()
    out = settle(s, (0.05, 0.0, 0.1))
    assert out.gain == 1.0
    assert (out.vx, out.wz) == (0.05, 0.1)


def test_rpp_regime_passes_through_unshaped():
    """A curvature-continuous sweep - the ICR moving a little every message -
    must never be touched. Spec section 5.

    The sweep deliberately stays on one side of wz = 0: crossing zero is a real
    geometry flip and is covered by the next test.
    """
    s = make()
    settle(s, (0.05, 0.0, 0.05))
    for i in range(400):
        wz = 0.05 + 0.04 * math.sin(i / 40.0)
        out = s.shape(0.05, 0.0, wz, dt=0.05)
        assert out.gain == 1.0, f"shaped an RPP-regime command at step {i}"


def test_a_curvature_sign_change_is_briefly_held():
    """Not a false positive: as wz crosses zero the ICR sweeps from +10 m
    through infinity to -7 m, which is more steering than a tick can deliver,
    and the model takes 35 ticks to flip an arc. Worth knowing that ordinary
    driving through a curvature sign change is briefly shaped."""
    s = make()
    settle(s, (0.05, 0.0, 0.02))
    out = s.shape(0.05, 0.0, -0.02, dt=0.05)
    assert out.gain < 1.0
    assert out.delta_beta_rad > g.ONE_TICK_BETA
    assert out.limited_by in ("slew", "fidelity")


def test_a_point_turn_transition_is_held_down():
    s = make()
    settle(s, (0.05, 0.0, 0.0))               # settle straight
    out = s.shape(0.0, 0.0, 0.1, dt=0.05)     # demand a point turn
    # dBeta is 0.8475, so one tick of steering buys 0.09/0.8475 = 0.106 of the
    # commanded motion. A point turn's geometry is scale-invariant, so the
    # fidelity guard does not raise it.
    assert out.gain == pytest.approx(0.106, abs=0.01)
    assert out.limited_by == "slew"
    assert out.wz == pytest.approx(0.1 * out.gain)
    assert out.delta_beta_rad == pytest.approx(0.8475, abs=0.001)


def test_the_hold_releases_and_the_gain_recovers_to_one():
    s = make()
    settle(s, (0.05, 0.0, 0.0))
    gains = [s.shape(0.0, 0.0, 0.1, dt=0.05).gain for _ in range(200)]
    assert gains[0] < 0.2
    assert gains[-1] == 1.0
    assert gains == sorted(gains), "the gain must recover monotonically"
    # 60 ticks at 0.06 s = 3.6 s, the bucket for a 0.8475 rad change, which
    # bounds the model's measured 56-tick worst case.
    held = sum(1 for x in gains if x < 1.0) * 0.05
    assert 3.3 <= held <= 4.2


def test_a_near_straight_command_is_only_partly_scaled():
    """The fidelity guard. The slew policy alone would ask for a gain of 0.106
    here, but scaling (0.05, 0, 0) that far moves its ICR from 9.87 m to 1.2 m -
    a shaper that did that would steer the rover off the very path it is
    protecting. At the default 0.10 rad tolerance the guard floors it at 0.35."""
    s = make()
    settle(s, (0.0, 0.0, 0.1))                # settle a point turn
    out = s.shape(0.05, 0.0, 0.0, dt=0.05)    # now demand straight
    assert out.gain == pytest.approx(0.350, abs=0.01)
    assert out.limited_by == "fidelity"
    assert g.command_distance((0.05, 0.0, 0.0), (out.vx, out.vy, out.wz)) <= 0.10 + 1e-9


@pytest.mark.parametrize("cmd", [
    (0.05, 0.0, 0.0), (0.05, 0.0, 0.1), (0.3, 0.0, 0.2),
    (0.0, 0.05, 0.0), (0.02, 0.0, 0.1), (0.45, 0.0, 0.4),
])
@pytest.mark.parametrize("tol", [0.02, 0.10, 0.20])
def test_the_fidelity_guard_bounds_the_geometry_error_it_allows(cmd, tol):
    s = make(icr_fidelity_tol_rad=tol, backstop_max_vx=0.0, backstop_max_wz=0.0)
    settle(s, (0.0, 0.0, 0.1))    # a point turn, so every cmd below is a jump
    out = s.shape(*cmd, dt=0.05)
    err = g.command_distance(cmd, (out.vx, out.vy, out.wz))
    assert err <= tol + 1e-9, f"{cmd} shaped to a geometry {err:.4f} rad away"


def test_a_stop_retains_the_geometry_so_a_turn_after_it_is_still_held():
    """Ruling 5(a), and the reason the zero path must not clear
    _last_cmd.

    kinematics.cpp's '<S1>/Retain Translation' block feeds the ICR controller
    from the last NON-ZERO command whenever the twist is all zeros, so a stop
    leaves the wheels exactly where they were. Measured: a settled straight run
    held at zero for 461 ticks keeps input_ICR at [-0, 9.8687660112452] with
    beta_next unchanged. The chassis is therefore still in the STRAIGHT pose
    when the point-turn command arrives, and the real model spends 26 ticks and
    0.0441 m getting out of it.

    If the shaper cleared its belief on the stop, command_distance would be 0
    (both a zero twist and a point turn map to the point-turn ICR), no hold
    would open, and a full-speed point turn would go straight through. That
    sequence - drive, stop, turn in place - is what a Nav2
    RotationShimController hand-off produces routinely.
    """
    s = make()
    settle(s, (0.05, 0.0, 0.0))
    for _ in range(40):                       # 2 s of supervisor zero stream
        z = s.shape(0.0, 0.0, 0.0, dt=0.05)
        assert (z.vx, z.vy, z.wz) == (0.0, 0.0, 0.0)
    out = s.shape(0.0, 0.0, 0.1, dt=0.05)
    assert out.delta_beta_rad == pytest.approx(0.8475, abs=0.001), \
        "the stop must not have erased the straight geometry"
    assert out.gain == pytest.approx(0.106, abs=0.01)
    assert out.limited_by == "slew"
    assert out.hold_remaining_s == pytest.approx(60 * g.IK_TIMESTEP_S, abs=0.01)
    assert out.feasible is False


def test_resuming_the_same_command_after_a_stop_is_free():
    """The other half of Retain Translation, and the one that makes the fix
    cheap rather than merely safe.

    Because the wheels did not move during the stop, resuming the SAME geometry
    costs nothing: measured on the real model, settle_tick 0 and 0.000000000
    rad of steering. The shaper must agree, or every Nav2 replan pause, goal
    boundary and recovery would pay a spurious 3.6 s crawl at gain 0.35.
    """
    s = make()
    settle(s, (0.05, 0.0, 0.0))
    for _ in range(40):
        s.shape(0.0, 0.0, 0.0, dt=0.05)
    out = s.shape(0.05, 0.0, 0.0, dt=0.05)
    assert out.delta_beta_rad == pytest.approx(0.0, abs=1e-12)
    assert out.gain == 1.0
    assert out.limited_by == "none"
    assert (out.vx, out.wz) == (0.05, 0.0)


def test_an_overriding_backstop_reports_the_error_it_causes():
    """The backstop is a hard speed ceiling and outranks the fidelity guard -
    a speed cap is a safety limit, geometry fidelity is a quality one. It is
    therefore allowed to push the geometry error past icr_fidelity_tol_rad,
    and when it does, fidelity_err_rad must report the TRUE error rather than
    the tolerance the guard would have enforced on its own."""
    s = make(backstop_max_vx=0.01, icr_fidelity_tol_rad=0.10)
    out = settle(s, (0.05, 0.0, 0.0))
    assert out.limited_by == "backstop"
    assert out.gain == pytest.approx(0.2, abs=0.01)     # 0.01 / 0.05
    assert out.fidelity_err_rad == pytest.approx(0.2365, abs=0.01)
    assert out.fidelity_err_rad > 0.10, \
        "the backstop overrode the guard and the report must say so"
    assert out.fidelity_err_rad == pytest.approx(
        g.command_distance((0.05, 0.0, 0.0), (out.vx, out.vy, out.wz)), abs=1e-12)


def test_backstop_caps_clamp_down_only():
    s = make(backstop_max_vx=0.05, backstop_max_wz=0.1)
    out = settle(s, (0.45, 0.0, 0.4))
    assert abs(out.vx) <= 0.05 + 1e-12
    assert abs(out.wz) <= 0.1 + 1e-12
    assert out.limited_by == "backstop"
    # still a uniform scale of the input, still never larger
    assert abs(out.vx) <= 0.45 and abs(out.wz) <= 0.4
    assert out.vx / 0.45 == pytest.approx(out.wz / 0.4, rel=1e-12)


def test_a_gap_in_the_stream_does_not_blow_up_the_hold():
    s = make()
    settle(s, (0.05, 0.0, 0.0))
    s.shape(0.0, 0.0, 0.1, dt=0.05)
    out = s.shape(0.0, 0.0, 0.1, dt=30.0)   # the link dropped for 30 s
    assert out.gain == 1.0, "a long gap means the chassis has long since settled"


def test_diagnostics_report_the_straight_line_bias():
    s = make()
    out = s.shape(0.05, 0.0, 0.0, dt=0.05)
    assert out.icr[1] == pytest.approx(9.8688, abs=1e-3)
    # 0.05 m/s round a 9.8688 m ICR. The model's own eta_dot_constrained
    # reports 0.00496 rad/s for the same command; the 2% difference is the
    # wheel solution the ICR alone does not capture, and does not matter for a
    # diagnostic whose job is to say "this is not zero".
    assert out.straight_bias_rad_s == pytest.approx(0.00507, abs=1e-4)
    assert out.straight_bias_rad_s > 0.0


def test_the_backstop_is_the_drive_trains_limit_not_todays_driving_speed():
    # It was 0.2/0.2 - the speed being driven during the first careful
    # sessions - which scaled the ground station's speed slider straight
    # back down above 0.2 m/s, so the slider appeared to do nothing.
    cfg = ShaperConfig()

    assert cfg.backstop_max_vx == pytest.approx(0.5)
    assert cfg.backstop_max_wz == pytest.approx(1.0)


def test_a_command_at_the_sliders_ceiling_settles_there_unscaled():
    # The shaper ramps toward a command, so the steady state - which is
    # what the backstop acts on - takes more than one tick to reach.
    out = settle(make(), (0.5, 0.0, 0.0))

    assert out.vx == pytest.approx(0.5)
    assert out.limited_by != "backstop"


def test_a_command_past_the_drive_train_is_still_scaled_back():
    out = settle(make(), (0.9, 0.0, 0.0))

    assert out.vx == pytest.approx(0.5)
    assert out.limited_by == "backstop"
