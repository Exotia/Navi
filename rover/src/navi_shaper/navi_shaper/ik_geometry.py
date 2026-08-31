"""The 2.42 IK's own command-to-geometry map, transcribed exactly.

This is not a model of the rover's kinematics. It is a transcription of two
formulas out of `sim/src/navi_sim_ik/vendor242/kinematics.cpp`, checked
bit-for-bit against the compiled model's own outputs over the whole command
range this rover uses:

  input_ICR   - the '<S1>/ICR Position Controller' MATLAB Function, lines
                826-845, with the '<S1>/Rmax' and '<S1>/delta' constants.
  beta_next   - the '<S1>/ICR2SteerAngles' MATLAB Function, line 1386.

Nothing here models how the steering *moves* between two geometries. That part
of the model is an optimiser with border-point search and a latched indirect
mode, and it is deliberately not reimplemented - see HOLD_TICKS in shaper.py,
which takes the cost of a transition from measurements of the real model
instead of predicting it.

No ROS. No numpy. Import this from anywhere.
"""
import math
import struct

__all__ = [
    "HPARAMS", "RMAX", "DELTA", "BETA_DOT_MAX", "IK_TIMESTEP_S", "ONE_TICK_BETA",
    "commanded_icr", "steering_angles", "angle_distance", "command_distance",
]


def _f32(value: float) -> float:
    """Widen a float32 to double, the way IkController does.

    The rover declares its wheel offsets `const float` and hands them to
    IkController in a std::vector<float>, whose constructor widens them into
    hParams. Writing 0.45527 as a Python float is a *different number* in the
    8th decimal place, and the steering angles would then not be the rover's
    arithmetic. See the FLOAT, NOT DOUBLE note in asterope_params.hpp.
    """
    return struct.unpack("f", struct.pack("f", value))[0]


# From bemacontroller/src/RoverParameters.h, the #if ASTEROPE block, in the
# order BemaServer.cpp:32 passes them to IkController: wheel1x, wheel1y,
# wheel2x, wheel2y, wheel3x, wheel3y, wheel4x, wheel4y. Wheel order is
# front_left, front_right, rear_right, rear_left.
#
# wheel3y is 0.44285 where the other three use 0.44385 - a 1 mm asymmetry in an
# otherwise symmetric chassis and almost certainly an upstream typo. It is
# transcribed as the rover has it, for the same reason SP4 did: a clamp is only
# trustworthy if it clamps against the model the wheels obey.
HPARAMS = tuple(_f32(v) for v in (
    0.45527, -0.44385,
    0.45527, 0.44385,
    -0.45527, 0.44285,
    -0.45527, -0.44385,
))

#: '<S1>/Rmax' - the ICR is saturated to this radius, so "straight ahead" is
#: represented as a 50 m arc rather than as infinity.
RMAX = 50.0

#: '<S1>/delta' - added to |yaw rate| to keep the ICR division finite. It is
#: also why a straight-line command is not straight: with wz = 0 the effective
#: yaw rate is 0.005 rad/s, which at 0.05 m/s is a 9.87 m radius left arc.
DELTA = 0.005

#: IkController.h. Steering rate ceiling, rad/s.
BETA_DOT_MAX = 1.5

#: IkController.h - the rover hardcodes 0.06 s and its update thread sleeps it.
IK_TIMESTEP_S = 0.06

#: The most steering one IK tick can deliver. A commanded geometry change
#: smaller than this is one the chassis absorbs immediately; larger, and the
#: chassis spends several ticks catching up.
ONE_TICK_BETA = BETA_DOT_MAX * IK_TIMESTEP_S  # 0.09 rad


def commanded_icr(vx: float, vy: float, wz: float) -> tuple:
    """The instantaneous centre of rotation the IK will steer toward.

    Body frame, metres, saturated to +-RMAX. Exactly kinematics.cpp:836-845.
    """
    sign = 1.0 if wz >= 0.0 else -1.0
    denom = sign * DELTA + wz
    return (
        math.tanh(-vy / denom / RMAX) * RMAX,
        math.tanh(vx / denom / RMAX) * RMAX,
    )


def steering_angles(icr_x: float, icr_y: float) -> tuple:
    """The four steering angles that ICR implies. Exactly kinematics.cpp:1386.

    Returned unwrapped, in (-3*pi/2, pi/2], the way the model emits them. Use
    angle_distance() to compare two of these - never plain subtraction.
    """
    return tuple(
        math.atan2(icr_y - HPARAMS[2 * i + 1], icr_x - HPARAMS[2 * i]) - math.pi / 2
        for i in range(4)
    )


def angle_distance(a: float, b: float) -> float:
    """How far a steering axle must actually turn to get from a to b.

    Modulo pi, reduced to [0, pi/2]. A wheel steered to beta + pi and spun
    backwards is the same wheel in the same place, and the model's own
    SteerAngles2SteerSpeed block picks whichever of the two is nearer
    (kinematics.cpp:1391-1400). Plain subtraction reports differences of pi
    that correspond to no motion at all.
    """
    d = (a - b) % math.pi
    return min(d, math.pi - d)


def command_distance(cmd_a, cmd_b) -> float:
    """The largest steering change between two commanded geometries, rad.

    Each argument is (vx, vy, wz). This is the single scalar the gain policy
    keys on: zero when two commands ask for the same geometry (whatever their
    speeds), and at most pi/2.
    """
    betas_a = steering_angles(*commanded_icr(*cmd_a))
    betas_b = steering_angles(*commanded_icr(*cmd_b))
    return max(angle_distance(a, b) for a, b in zip(betas_a, betas_b))
