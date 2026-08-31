"""The feasibility clamp's gain policy. Pure: no ROS, no wall clock, no I/O.

The rover's steering cannot jump between turn geometries. Moving the ICR from
"straight ahead" to "turn in place" takes the real 2.42 model up to 56 ticks -
3.36 s - and while that sweep is in progress the chassis follows neither the
old command nor the new one. What this module does about it:

  1. Measure the geometry change each incoming twist demands, as the exact
     closed-form steering distance between the previous commanded geometry and
     this one (ik_geometry.command_distance).
  2. If it is within one tick of steering (0.09 rad), do nothing at all. That
     is the regime a curvature-continuous controller like RPP lives in, and it
     is measurably the regime the IK absorbs immediately.
  3. Otherwise open a hold window whose length is the *measured* worst case for
     a change that size, and scale the whole twist down for its duration, so
     the ground covered while the wheels are wrong is small. The gain then
     recovers linearly to 1.0 as the window closes.
  4. Never choose a gain whose own effect on the geometry exceeds
     icr_fidelity_tol - scaling is not geometry-preserving in this model, and
     for a near-straight command it is actively harmful (see ik_geometry.DELTA).

Everything the policy does is a single uniform gain in [0, 1] applied to all
three components, so the output is never larger than the input in any
component and never points anywhere the input did not.
"""
import math
from dataclasses import dataclass, field

from navi_shaper import ik_geometry as g

#: Worst-case settle time, in IK ticks, as a function of the commanded steering
#: change. Measured on the real 2.42 model over all 342 transitions between the
#: 19 representative commands in feasibility_harness_242.cpp: for each bucket,
#: the largest settle count observed, rounded up to a round number.
#:
#: Do not fit a curve to this. The cost is wildly non-monotone - a 0.75 rad
#: change can cost 56 ticks where a 1.45 rad change costs 17 - because the
#: price is paid when the ICR must travel through the wheelbase, or when the
#: model enters its latched indirect mode and swings the ICR out to +-50 m to
#: flip sides. test_golden_parity.py asserts this table bounds every measured
#: transition; regenerate the fixture and re-check it after any re-vendor.
#:
#: (upper bound of the dBeta bucket, hold ticks)
HOLD_TICKS = (
    (0.09, 3),     # measured max 3   - the RPP regime, effectively a no-op
    (0.30, 6),     # measured max 5
    (0.70, 12),    # measured max 10
    (1.20, 60),    # measured max 56  - straight <-> point turn lives here
    # pi/2, not pi: angle_distance() reduces modulo pi and returns
    # min(d, pi - d), so dBeta can never exceed 1.5708 and a `math.pi` upper
    # bound here would be dead code that reads as if the table covered twice
    # the range it does.
    (math.pi / 2, 56),  # measured max 51
)


def hold_ticks_for(delta_beta: float) -> int:
    """How long the chassis may take to absorb a steering change of this size."""
    for upper, ticks in HOLD_TICKS:
        if delta_beta <= upper:
            return ticks
    # Unreachable for any value angle_distance() can produce (the last bucket
    # ends at its maximum, pi/2). Kept as a defensive floor so a future caller
    # passing an unreduced angle gets the most conservative hold rather than a
    # TypeError on a None return.
    return HOLD_TICKS[-1][1]


@dataclass(frozen=True)
class ShaperConfig:
    #: Never scale below this. Zero would stop the chassis steering toward the
    #: new geometry at all: the IK derives the ICR from the *direction* of the
    #: twist, so any positive gain keeps the wheels sweeping at full rate while
    #: a gain of zero leaves it with nothing to aim at.
    min_gain: float = 0.05
    #: The most steering error the shaper's own down-scaling may introduce.
    #: 0.10 rad is 5.7 degrees at the wheel - see ruling 4 for the measured
    #: table this default was chosen from. Tightening it to 0.02 would floor a
    #: straight command's gain at 0.71 and leave the shaper unable to act.
    icr_fidelity_tol_rad: float = 0.10
    #: Section 10 caps, as a backstop. The smoother and gamepad_input.py own
    #: these numbers; this is where a misconfigured one is caught.
    backstop_max_vx: float = 0.2
    backstop_max_wz: float = 0.2
    #: A gap longer than this means the chassis has settled whatever it was
    #: doing; drop any hold rather than carrying a stale one across a dropout.
    max_dt_s: float = 1.0


@dataclass
class ShapedTwist:
    vx: float
    vy: float
    wz: float
    gain: float
    feasible: bool
    limited_by: str          # "none" | "slew" | "fidelity" | "backstop"
    #: What was limiting *before* the backstop overrode it, so the backstop
    #: does not discard the diagnostic it replaces. "none" unless
    #: limited_by == "backstop".
    also_limited_by: str
    #: command_distance(cmd, output): the geometry error the shaped output
    #: really carries. Bounded by icr_fidelity_tol_rad when the guard set the
    #: gain; unbounded when the backstop overrode it - see shape().
    fidelity_err_rad: float
    delta_beta_rad: float
    hold_remaining_s: float
    icr: tuple = (0.0, 0.0)
    straight_bias_rad_s: float = 0.0


@dataclass
class TwistShaper:
    config: ShaperConfig = field(default_factory=ShaperConfig)

    def __post_init__(self):
        self.reset()

    def reset(self):
        # (0, 0, 0) here is correct and is NOT the same statement the zero
        # short-circuit used to make. It models the model's own zero
        # UnitDelay_DSTATE at initialize(): before a freshly booted kinematics
        # has ever seen a non-zero command, Retain Translation has nothing to
        # retain and the ICR really is the origin, i.e. the point-turn pose. So
        # the very first command after boot is correctly treated as a sweep
        # from the point turn. Only at boot - never again after a stop.
        self._last_cmd = (0.0, 0.0, 0.0)
        self._hold_remaining_s = 0.0
        self._hold_total_s = 0.0
        self._hold_entry_gain = 1.0

    # -- the one entry point ------------------------------------------------
    def shape(self, vx: float, vy: float, wz: float, dt: float) -> ShapedTwist:
        """Shape one commanded twist. `dt` is the time since the previous call."""
        cfg = self.config

        # A stop is relayed untouched and immediately - before any maths runs,
        # so that no future change to the policy below can delay or reshape it.
        #
        # _last_cmd is deliberately NOT cleared here. kinematics.cpp's
        # '<S1>/Retain Translation' block (lines 798-824) feeds the ICR
        # Position Controller from UnitDelay_DSTATE - the last NON-ZERO
        # command - whenever VX, VY and U are all exactly zero. So a full-zero
        # twist does not steer the chassis to the point-turn pose: it RETAINS
        # the geometry it already had. Measured on the real model, a settled
        # straight run held at zero for 461 ticks keeps input_ICR at
        # [-0, 9.8687660112452] with beta_next unchanged, and resuming straight
        # afterwards costs zero ticks and zero radians.
        #
        # The shaper's belief about the chassis geometry must therefore survive
        # a stop exactly as the chassis' own geometry does. Clearing it would
        # make command_distance((0,0,0), point_turn) = 0 - both map to the
        # point-turn ICR - so the sequence
        #   settled straight -> supervisor zero stream -> point-turn command
        # would open no hold, report feasible, and pass a full-speed point turn
        # into a 26-tick / 0.0441 m wrong-geometry sweep. That is a routine
        # Nav2 RotationShimController hand-off, and it is the exact failure
        # this whole module exists to prevent.
        #
        # _hold_remaining_s IS cleared: standing still genuinely does let any
        # in-flight sweep finish, and a stop must not leave a hold armed to
        # bite the next command.
        if vx == 0.0 and vy == 0.0 and wz == 0.0:
            self._hold_remaining_s = 0.0
            return ShapedTwist(0.0, 0.0, 0.0, 1.0, True, "none", "none", 0.0,
                               0.0, 0.0, g.commanded_icr(0.0, 0.0, 0.0), 0.0)

        cmd = (vx, vy, wz)
        icr = g.commanded_icr(*cmd)

        # A gap in the stream means the chassis has had time to settle.
        if dt > cfg.max_dt_s:
            self._hold_remaining_s = 0.0
        else:
            self._hold_remaining_s = max(0.0, self._hold_remaining_s - dt)

        delta = g.command_distance(self._last_cmd, cmd)

        # A demand larger than one tick of steering opens (or re-opens) a hold.
        if delta > g.ONE_TICK_BETA:
            hold_s = hold_ticks_for(delta) * g.IK_TIMESTEP_S
            if hold_s > self._hold_remaining_s:
                self._hold_remaining_s = hold_s
                self._hold_total_s = hold_s
                # What one tick can deliver, over what was asked for: the
                # fraction of the commanded motion the chassis can honour now.
                self._hold_entry_gain = max(cfg.min_gain, g.ONE_TICK_BETA / delta)

        if self._hold_remaining_s > 0.0 and self._hold_total_s > 0.0:
            # Recover linearly as the window closes. The chassis reaches the
            # new geometry at full steering rate regardless of the gain - the
            # gain buys distance, not time - so a linear release is honest.
            progress = 1.0 - (self._hold_remaining_s / self._hold_total_s)
            slew_gain = self._hold_entry_gain + (1.0 - self._hold_entry_gain) * progress
        else:
            slew_gain = 1.0

        fidelity_gain = self._min_faithful_gain(cmd)
        gain = max(slew_gain, fidelity_gain)

        # "fidelity" means the guard is what stopped the shaper going lower -
        # including the case where it stopped it dead at 1.0. "slew" means the
        # hold policy alone set the gain. The distinction is the whole content
        # of the diagnostic: one says "I am protecting the transition", the
        # other says "I wanted to and could not".
        if fidelity_gain > slew_gain:
            limited_by = "fidelity"
        elif gain < 1.0:
            limited_by = "slew"
        else:
            limited_by = "none"

        # The backstop, applied last and downward only. Downward-only means it
        # can never increase a component - but it multiplies `gain` AFTER the
        # fidelity bisection has already chosen the smallest admissible value,
        # so the product is NOT bounded by icr_fidelity_tol_rad and the
        # resulting geometry error can exceed it. That is deliberate, and the
        # precedence is: a speed cap is a SAFETY limit and geometry fidelity is
        # a QUALITY one, so the backstop wins. With backstop_max_vx = 0.01 and
        # a 0.05 m/s command the effective gain is 0.2, whose measured geometry
        # error is 0.2365 rad - 2.4x the 0.10 rad default. The answer is not to
        # weaken the cap but to make the cost visible: fidelity_err_rad below
        # carries the true error, and /ik_feasibility publishes it.
        backstop = 1.0
        if cfg.backstop_max_vx > 0.0 and abs(vx * gain) > cfg.backstop_max_vx:
            backstop = min(backstop, cfg.backstop_max_vx / abs(vx * gain))
        if cfg.backstop_max_wz > 0.0 and abs(wz * gain) > cfg.backstop_max_wz:
            backstop = min(backstop, cfg.backstop_max_wz / abs(wz * gain))
        also_limited_by = "none"
        if backstop < 1.0:
            gain *= backstop
            # Keep whatever was limiting before, rather than discarding it:
            # "backstop" alone cannot tell you whether a transition was also
            # being protected at the time.
            also_limited_by = limited_by
            limited_by = "backstop"

        gain = min(1.0, max(0.0, gain))
        self._last_cmd = cmd
        scaled = (vx * gain, vy * gain, wz * gain)
        return ShapedTwist(
            vx=scaled[0], vy=scaled[1], wz=scaled[2],
            gain=gain,
            feasible=delta <= g.ONE_TICK_BETA,
            limited_by=limited_by,
            also_limited_by=also_limited_by,
            # The geometry error this output actually carries. Inside
            # icr_fidelity_tol_rad whenever the guard set the gain; possibly
            # well outside it when the backstop overrode the guard. Reported
            # either way - a limit that silently misbehaves is worse than one
            # that says what it cost.
            fidelity_err_rad=g.command_distance(cmd, scaled),
            delta_beta_rad=delta,
            hold_remaining_s=self._hold_remaining_s,
            icr=icr,
            straight_bias_rad_s=self._straight_bias(cmd, icr),
        )

    # -- the fidelity guard -------------------------------------------------
    def _min_faithful_gain(self, cmd) -> float:
        """The smallest gain whose own geometry error stays inside tolerance.

        Scaling a twist is only geometry-preserving for a pure point turn. The
        IK's delta = 0.005 rad/s floor on the yaw rate means a scaled-down
        near-straight command reads as a much tighter arc: (0.05, 0, 0) at 20%
        moves the ICR from 9.87 m to 2.0 m. The error is monotone decreasing in
        the gain, so bisection finds the boundary.
        """
        tol = self.config.icr_fidelity_tol_rad
        lo, hi = self.config.min_gain, 1.0
        if self._geometry_error(cmd, lo) <= tol:
            return lo
        for _ in range(24):
            mid = 0.5 * (lo + hi)
            if self._geometry_error(cmd, mid) <= tol:
                hi = mid
            else:
                lo = mid
        return hi

    @staticmethod
    def _geometry_error(cmd, gain: float) -> float:
        scaled = (cmd[0] * gain, cmd[1] * gain, cmd[2] * gain)
        return g.command_distance(cmd, scaled)

    @staticmethod
    def _straight_bias(cmd, icr) -> float:
        """The yaw the IK adds to a command that asked for none.

        Reported, not corrected: correcting it would mean increasing |wz|,
        which the never-larger rule forbids. SP12 yard-tuning input.
        """
        if cmd[2] != 0.0:
            return 0.0
        radius = math.hypot(icr[0], icr[1])
        if radius < 1e-9:
            return 0.0
        return math.hypot(cmd[0], cmd[1]) / radius
