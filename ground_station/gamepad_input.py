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

DEADZONE = 0.1
MAX_LINEAR_SPEED = 0.5  # m/s at full stick deflection
MAX_ANGULAR_SPEED = 1.0  # rad/s at full stick deflection


def _apply_deadzone(value: float, deadzone: float = DEADZONE) -> float:
    return 0.0 if abs(value) < deadzone else value


class GamepadReader:
    def __init__(self, pygame_module=_pygame):
        self._pygame = pygame_module
        self._joystick = None
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
        linear_x = -raw_y * MAX_LINEAR_SPEED
        linear_y = -raw_x * MAX_LINEAR_SPEED
        angular_z = -raw_rot * MAX_ANGULAR_SPEED

        return (linear_x, linear_y, angular_z)
