"""A top-down picture of what the four steering wheels are doing.

The numbers on the DRIVE card say what was commanded; this says what the
chassis is about to look like, which is the thing an operator can check
against the rover in front of them. It draws the real geometry: wheel
positions come from the rover's own hParams, and the steering angles from
the rover's own ICR arithmetic (navi_shaper.ik_geometry, a transcription of
the compiled 2.42 model), so a wheel drawn at 40 degrees is a wheel the
rover would actually put at 40 degrees.

If that module cannot be imported - a ground station installed away from
the rover tree - the widget degrades to drawing straight wheels and says
so, rather than inventing kinematics of its own.
"""

import math
import os
import sys

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPen, QPolygonF, QTransform
from PySide6.QtWidgets import QWidget

from ground_station import theme

# The shaper package is not installed alongside the ground station, but it
# is a sibling in this repo and imports nothing but math and struct. Kept
# optional on purpose: no ground-station feature may hard-depend on the
# rover tree being present.
_here = os.path.dirname(os.path.abspath(__file__))
_shaper = os.path.normpath(
    os.path.join(_here, "..", "..", "rover", "src", "navi_shaper"))
if _shaper not in sys.path:
    sys.path.append(_shaper)
try:
    from navi_shaper.ik_geometry import HPARAMS, commanded_icr, steering_angles
    IK_AVAILABLE = True
except Exception:                                     # noqa: BLE001
    IK_AVAILABLE = False
    # front_left, front_right, rear_right, rear_left - the chassis is still
    # drawn to scale, only the steering is unknown.
    HPARAMS = (0.45527, -0.44385, 0.45527, 0.44385,
               -0.45527, 0.44285, -0.45527, -0.44385)

#: Wheel order in HPARAMS and in steering_angles' return value, named in
#: the frame the ICR arithmetic actually uses: commanded_icr(0.2, 0, +0.2)
#: returns y = +0.98, so a left turn puts the ICR at positive y and +y is
#: LEFT (REP-103). The upstream comment in ik_geometry.py names these
#: front_left/front_right the other way round; the positions are what this
#: widget draws from, so the picture is right either way.
WHEEL_NAMES = ("front right", "front left", "rear left", "rear right")

#: Below this the command is a stop, and the rover holds whatever geometry
#: it last had - so the view holds it too, dimmed, rather than snapping to
#: a meaningless zero-speed ICR.
MOVING_EPS = 1e-4


def display_angles(vx: float, vy: float, wz: float):
    """The four steering angles to draw, in radians, or None if this build
    cannot compute them.

    Normalised to [-pi/2, pi/2]: a steered wheel is a line, not an arrow,
    so 135 degrees and -45 degrees are the same picture - and the model's
    raw output for a point turn is full of wrapped values like -225 degrees
    that would draw a wheel spinning the long way round.
    """
    if not IK_AVAILABLE:
        return None
    icr_x, icr_y = commanded_icr(float(vx), float(vy), float(wz))
    angles = steering_angles(icr_x, icr_y)
    return [_wrap_to_half_turn(a) for a in angles]


def _wrap_to_half_turn(angle: float) -> float:
    a = (float(angle) + math.pi / 2.0) % math.pi - math.pi / 2.0
    return a


class WheelView(QWidget):
    """Top-down chassis with four steered wheels. `set_twist` is the whole
    input; nothing here reads a topic."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(150, 130)
        self._twist = (0.0, 0.0, 0.0)
        # The last geometry commanded while actually moving. A stopped rover
        # keeps its steering where it was, and so does this picture.
        self._angles = [0.0, 0.0, 0.0, 0.0]
        self._moving = False
        # No fresh command has arrived recently. The drawn geometry is still
        # where the steering is; only the caption stops claiming it is a
        # live command.
        self._stale = False

    def set_twist(self, vx: float, vy: float, wz: float) -> None:
        self._twist = (float(vx), float(vy), float(wz))
        self._stale = False
        self._moving = max(abs(v) for v in self._twist) > MOVING_EPS
        if self._moving:
            angles = display_angles(*self._twist)
            if angles is not None:
                self._angles = angles
        self.update()

    @property
    def angles(self):
        """The angles currently drawn, radians, in WHEEL_NAMES order."""
        return list(self._angles)

    @property
    def moving(self) -> bool:
        return self._moving

    @property
    def stale(self) -> bool:
        return self._stale

    def mark_stale(self) -> None:
        self._stale = True
        self._moving = False
        self.update()

    def paintEvent(self, event) -> None:      # pragma: no cover - painting
        painter = QPainter(self)
        try:
            painter.setRenderHint(QPainter.Antialiasing, True)
            self._paint(painter)
        finally:
            painter.end()

    def _paint(self, painter) -> None:
        w, h = self.width(), self.height()
        painter.fillRect(0, 0, w, h, QColor(theme.BG))

        # Metres -> pixels, with room for the wheels sticking out past the
        # hub positions and a margin for the heading arrow.
        span_x = max(abs(HPARAMS[0]), abs(HPARAMS[4])) * 2.0 + 0.30
        span_y = max(abs(HPARAMS[1]), abs(HPARAMS[3])) * 2.0 + 0.30
        scale = min(w / span_y, h / span_x) if span_x and span_y else 1.0
        cx, cy = w / 2.0, h / 2.0

        def to_screen(x_m: float, y_m: float) -> QPointF:
            # Rover x is forward (up the screen), +y is left (REP-103, and
            # what the ICR arithmetic uses - see WHEEL_NAMES), so a wheel at
            # positive y is drawn left of centre.
            return QPointF(cx - y_m * scale, cy - x_m * scale)

        body = QRectF(to_screen(HPARAMS[0], HPARAMS[3]),
                      to_screen(HPARAMS[4], HPARAMS[1]))
        painter.setPen(QPen(QColor(theme.BORDER), 1))
        painter.setBrush(QColor(theme.PANEL))
        painter.drawRoundedRect(body, 6, 6)

        # Which way is forward. Drawn always, so a stopped rover still says
        # which end is the front.
        nose = QPolygonF([to_screen(HPARAMS[0] + 0.16, 0.0),
                          to_screen(HPARAMS[0] + 0.02, -0.09),
                          to_screen(HPARAMS[0] + 0.02, 0.09)])
        painter.setBrush(QColor(theme.TEXT_DIM))
        painter.setPen(Qt.NoPen)
        painter.drawPolygon(nose)

        vx, _vy, _wz = self._twist
        if self._stale:
            wheel_colour = QColor(theme.OFF)
        elif self._moving:
            wheel_colour = QColor(theme.ACCENT if vx >= 0 else theme.BAD)
        else:
            wheel_colour = QColor(theme.OFF)

        wheel_len = 0.26 * scale
        wheel_wide = 0.10 * scale
        for i in range(4):
            x_m, y_m = HPARAMS[2 * i], HPARAMS[2 * i + 1]
            centre = to_screen(x_m, y_m)
            angle = self._angles[i] if IK_AVAILABLE else 0.0
            painter.save()
            painter.translate(centre)
            # Screen y grows downward, so a left turn (positive model angle)
            # is a negative rotation on screen.
            painter.setTransform(
                QTransform().rotate(-math.degrees(angle)), True)
            painter.setBrush(wheel_colour)
            painter.setPen(Qt.NoPen)
            painter.drawRoundedRect(
                QRectF(-wheel_wide / 2.0, -wheel_len / 2.0, wheel_wide, wheel_len),
                3, 3)
            painter.restore()

        label = QColor(theme.TEXT_DIM)
        painter.setPen(QPen(label, 1))
        font = painter.font()
        font.setPointSize(theme.FONT_SIZE_SMALL - 2)
        painter.setFont(font)
        if not IK_AVAILABLE:
            painter.drawText(QRectF(0, h - 16, w, 14), Qt.AlignCenter,
                             "steering model unavailable")
        elif self._stale:
            painter.drawText(QRectF(0, h - 16, w, 14), Qt.AlignCenter,
                             "no command")
        elif self._moving:
            # The front pair, left then right as they appear on screen.
            painter.drawText(
                QRectF(0, h - 16, w, 14), Qt.AlignCenter,
                f"{math.degrees(self._angles[1]):+.0f}°   "
                f"{math.degrees(self._angles[0]):+.0f}°")
        else:
            painter.drawText(QRectF(0, h - 16, w, 14), Qt.AlignCenter, "stopped")
