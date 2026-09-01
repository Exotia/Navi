"""Reads an Xbox-style gamepad via pygame's joystick module and converts
stick position into a holonomic Twist (linear_x, linear_y, angular_z).

Axis indices below are for a standard Xbox controller and are the common
case, but pygame's raw joystick axis numbering can vary by OS/driver -
if the mapping is wrong on your machine, adjust the LEFT_STICK_X/
LEFT_STICK_Y/RIGHT_STICK_X constants (print(joystick.get_numaxes()) and
watch get_axis(i) while moving each stick to find the right indices).

No Qt/ROS dependency - polled from Qt via a QTimer in MainWindow.
"""

import signal

import pygame as _pygame

LEFT_STICK_X = 0
LEFT_STICK_Y = 1
RIGHT_STICK_X = 3

DEADZONE = 0.15
MAX_LINEAR_SPEED = 0.05  # m/s at full stick deflection - deliberately 1/10 of the 0.5 the drive train can take, for the first careful hardware sessions
MAX_ANGULAR_SPEED = 0.1  # rad/s at full stick deflection - same 1/10 caution factor

#: What the speed slider may ask for. The floor is slower than the default
#: (there are days when 5 cm/s is still too fast to park with); the ceiling
#: is the 0.5 m/s the drive train takes, which the default deliberately
#: sits a tenth of.
MIN_SETTABLE_LINEAR_SPEED = 0.02
MAX_SETTABLE_LINEAR_SPEED = 0.50

#: Turning rate per unit of top speed, from the pair of constants above.
#: The slider sets one number; this keeps the other in the same proportion,
#: so raising the speed does not quietly leave the rover turning at the
#: crawl rate that was chosen to match 0.05 m/s.
ANGULAR_PER_LINEAR = MAX_ANGULAR_SPEED / MAX_LINEAR_SPEED


def _apply_deadzone(value: float, deadzone: float = DEADZONE) -> float:
    # Rescaled, not a hard cutoff: the live [deadzone..1] range maps back
    # onto [0..1], so crossing the edge starts the rover from zero instead
    # of jumping straight to deadzone-fraction speed - at these deliberately
    # tiny caps that step would be a fifth of full speed.
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0.0 else -1.0
    return sign * min(1.0, (abs(value) - deadzone) / (1.0 - deadzone))


class GamepadReader:
    def __init__(self, pygame_module=_pygame):
        self._pygame = pygame_module
        self._joystick = None
        # Per-reader, not module constants, because the operator sets this
        # from the speed slider at run time. It starts at the cautious
        # default, so a session that never touches the slider drives
        # exactly as it did before the slider existed.
        self.max_linear_speed = MAX_LINEAR_SPEED
        self.max_angular_speed = MAX_ANGULAR_SPEED
        self._pygame.init()
        self._pygame.joystick.init()
        # pygame's SDL backend installs its own SIGINT/SIGTERM handlers as a
        # side effect of init (even just the joystick subsystem) - without
        # this, Ctrl+C (and a plain SIGTERM) stop terminating the process at
        # all once a GamepadReader exists. Restore Python's normal handling.
        signal.signal(signal.SIGINT, signal.default_int_handler)
        signal.signal(signal.SIGTERM, signal.SIG_DFL)

    def poll(self) -> bool:
        """Refresh hot-plug state. Must be called before read_twist(), on
        a regular timer. Returns True if a joystick is currently connected."""
        self._pygame.event.pump()
        if self._joystick is None:
            if self._pygame.joystick.get_count() > 0:
                self._joystick = self._pygame.joystick.Joystick(0)
                self._joystick.init()
        elif self._pygame.joystick.get_count() == 0:
            self._joystick = None
        return self._joystick is not None

    def read_twist(self) -> tuple[float, float, float]:
        """Returns (linear_x, linear_y, angular_z) from current stick
        position, or (0.0, 0.0, 0.0) if no joystick is connected."""
        if self._joystick is None:
            return (0.0, 0.0, 0.0)

        raw_x = _apply_deadzone(self._joystick.get_axis(LEFT_STICK_X))
        raw_y = _apply_deadzone(self._joystick.get_axis(LEFT_STICK_Y))
        raw_rot = _apply_deadzone(self._joystick.get_axis(RIGHT_STICK_X))

        # pygame reports stick-forward and stick-left as negative axis
        # values; ROS convention (REP-103) is +x forward, +y left,
        # +angular_z counter-clockwise - hence the negation below.
        linear_x = -raw_y * self.max_linear_speed
        linear_y = -raw_x * self.max_linear_speed
        angular_z = -raw_rot * self.max_angular_speed

        return (linear_x, linear_y, angular_z)

    def set_max_linear_speed(self, speed: float) -> None:
        """Top speed at full stick deflection, in m/s, clamped to the
        settable range. The turning rate follows in proportion."""
        speed = max(MIN_SETTABLE_LINEAR_SPEED,
                    min(MAX_SETTABLE_LINEAR_SPEED, float(speed)))
        self.max_linear_speed = speed
        self.max_angular_speed = speed * ANGULAR_PER_LINEAR
