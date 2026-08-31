"""The team mark in the window's top-left corner.

Painted rather than loaded from a file: the ground station ships as source
with no asset pipeline, and a missing PNG would leave a broken box in the
corner of every screenshot. If the team's real artwork is dropped in later,
`StarLogo` is the one place to swap.
"""

import math

from PySide6.QtCore import QPointF, Qt
from PySide6.QtGui import QColor, QPainter, QPolygonF
from PySide6.QtWidgets import QWidget

from ground_station import theme


def star_points(centre_x: float, centre_y: float, outer: float,
                inner: float, points: int = 5) -> list:
    """The alternating outer/inner vertices of a star, first point up."""
    vertices = []
    for i in range(points * 2):
        radius = outer if i % 2 == 0 else inner
        angle = -math.pi / 2.0 + i * math.pi / points
        vertices.append(QPointF(centre_x + radius * math.cos(angle),
                                centre_y + radius * math.sin(angle)))
    return vertices


class StarLogo(QWidget):
    """A star mark. Sized to the header's height, drawn in the accent."""

    def __init__(self, parent=None, size: int = 30):
        super().__init__(parent)
        self.setFixedSize(size, size)
        self.setToolTip("STAR Dresden e.V.")

    def paintEvent(self, event) -> None:       # pragma: no cover - painting
        painter = QPainter(self)
        try:
            painter.setRenderHint(QPainter.Antialiasing, True)
            side = min(self.width(), self.height())
            centre = side / 2.0
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(theme.ACCENT))
            painter.drawPolygon(QPolygonF(
                star_points(centre, centre, side * 0.48, side * 0.20)))
        finally:
            painter.end()
