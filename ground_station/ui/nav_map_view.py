"""The top-down plan canvas: click to drop a waypoint, watch the rover and
its plan move.

A click is turned into a world coordinate through the pure `ViewTransform`
(models.py) - no ROS, no window logic here. The window/NavRow wires
`point_clicked` to the waypoint list and feeds `set_pose`/`set_path_summary`
from the wire layer; this widget only ever draws what it is handed.
"""

import math

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QWidget

from ground_station import theme
from ground_station.models import ViewTransform

# Target on-screen length for the scale bar; the nearest 1/2/5-decade metre
# value is picked so the bar reads a round number rather than "37 m".
_SCALE_BAR_TARGET_PX = 80.0


def _nice_scale_length(metres_per_pixel: float) -> float:
    target_m = _SCALE_BAR_TARGET_PX * metres_per_pixel
    if target_m <= 0:
        return 1.0
    exponent = math.floor(math.log10(target_m))
    for mult in (1, 2, 5, 10):
        candidate = mult * (10 ** exponent)
        if candidate >= target_m:
            return float(candidate)
    return float(10 ** (exponent + 1))


class NavMapView(QWidget):
    """The plan canvas. World (map frame, metres) in, pixels on screen -
    the axis convention lives in `ViewTransform`, not here."""

    point_clicked = Signal(float, float)

    MIN_METRES_PER_PIXEL = 0.01
    MAX_METRES_PER_PIXEL = 0.5

    def __init__(self, parent=None):
        super().__init__(parent)
        self.transform = ViewTransform(0.0, 0.0, 0.05,
                                       max(self.width(), 1), max(self.height(), 1))
        self.pose = None
        self.waypoints = []
        self.path_points = []
        self.plan_waypoints = []
        self._follow = True

    # --- state in -----------------------------------------------------

    def set_pose(self, pose) -> None:
        self.pose = pose
        if self._follow and pose is not None:
            self.transform = self.transform.centred_on(pose["x"], pose["y"])
        self.update()

    def set_waypoints(self, waypoints) -> None:
        self.waypoints = list(waypoints)
        self.update()

    def set_path_summary(self, summary) -> None:
        if summary is None:
            self.path_points = []
            self.plan_waypoints = []
        else:
            self.path_points = list(summary.points)
            self.plan_waypoints = list(summary.waypoints)
        self.update()

    def set_follow(self, follow: bool) -> None:
        self._follow = bool(follow)

    # --- view controls --------------------------------------------------

    def set_view_centre(self, x: float, y: float) -> None:
        self.transform = self.transform.centred_on(x, y)
        self.update()

    def set_metres_per_pixel(self, value: float) -> None:
        self._set_metres_per_pixel(value)

    def zoom_in(self) -> None:
        self._set_metres_per_pixel(self.transform.metres_per_pixel * 0.5)

    def zoom_out(self) -> None:
        self._set_metres_per_pixel(self.transform.metres_per_pixel * 2.0)

    def _set_metres_per_pixel(self, value: float) -> None:
        clamped = max(self.MIN_METRES_PER_PIXEL,
                     min(self.MAX_METRES_PER_PIXEL, value))
        self.transform = ViewTransform(self.transform.centre_x, self.transform.centre_y,
                                       clamped, self.transform.width_px,
                                       self.transform.height_px)
        self.update()

    # --- Qt events -----------------------------------------------------

    def resizeEvent(self, event) -> None:
        self.transform = self.transform.resized(self.width(), self.height())
        super().resizeEvent(event)

    def mousePressEvent(self, event) -> None:
        # Belt-and-braces against resizeEvent not having fired yet (a
        # hidden/offscreen widget can have its size changed without Qt
        # delivering the event synchronously) - the click must still land
        # on the geometry the widget actually has right now.
        self.transform = self.transform.resized(self.width(), self.height())
        if event.button() == Qt.MouseButton.LeftButton:
            pos = event.position()
            wx, wy = self.transform.to_world(pos.x(), pos.y())
            self.point_clicked.emit(wx, wy)
        super().mousePressEvent(event)

    def paintEvent(self, event) -> None:
        self.transform = self.transform.resized(self.width(), self.height())
        painter = QPainter(self)
        try:
            self._paint(painter)
        finally:
            painter.end()

    # --- drawing ---------------------------------------------------------
    # Each step below is wrapped so a malformed value (a NaN pose, an empty
    # plan, an out-of-range zoom) cannot kill the window - the worst a bad
    # step can do is leave itself undrawn.

    def _paint(self, painter) -> None:
        try:
            painter.fillRect(self.rect(), QColor(theme.PANEL_DARK))
        except Exception:
            pass
        try:
            self._draw_grid(painter)
        except Exception:
            pass
        try:
            self._draw_plan(painter)
        except Exception:
            pass
        try:
            self._draw_waypoints(painter)
        except Exception:
            pass
        try:
            self._draw_pose(painter)
        except Exception:
            pass
        try:
            self._draw_scale_bar(painter)
        except Exception:
            pass

    def _draw_grid(self, painter) -> None:
        t = self.transform
        mpp = t.metres_per_pixel
        if mpp <= 0 or 1.0 / mpp < 4:
            return  # a metre would be under 4 px - the grid would be noise
        painter.setPen(QPen(QColor(theme.BORDER), 1))
        half_h_m = (t.height_px / 2.0) * mpp
        half_w_m = (t.width_px / 2.0) * mpp
        x_lo = int(math.floor(t.centre_x - half_h_m))
        x_hi = int(math.ceil(t.centre_x + half_h_m))
        for x in range(x_lo, x_hi + 1):
            _, py = t.to_pixel(float(x), t.centre_y)
            painter.drawLine(QPointF(0, py), QPointF(t.width_px, py))
        y_lo = int(math.floor(t.centre_y - half_w_m))
        y_hi = int(math.ceil(t.centre_y + half_w_m))
        for y in range(y_lo, y_hi + 1):
            px, _ = t.to_pixel(t.centre_x, float(y))
            painter.drawLine(QPointF(px, 0), QPointF(px, t.height_px))

    def _draw_plan(self, painter) -> None:
        if len(self.path_points) < 2:
            return
        painter.setPen(QPen(QColor(theme.ACCENT), 2))
        pts = [QPointF(*self.transform.to_pixel(x, y)) for x, y in self.path_points]
        for a, b in zip(pts, pts[1:]):
            painter.drawLine(a, b)

    def _draw_waypoints(self, painter) -> None:
        for i, wp in enumerate(self.waypoints):
            px, py = self.transform.to_pixel(wp.x, wp.y)
            painter.setPen(QPen(QColor(theme.TEXT), 1))
            painter.setBrush(QColor(theme.OK))
            painter.drawEllipse(QPointF(px, py), 5, 5)
            painter.drawText(QPointF(px + 7, py - 7), str(i + 1))

    def _draw_pose(self, painter) -> None:
        if self.pose is None:
            painter.setPen(QColor(theme.TEXT_DIM))
            painter.drawText(QRectF(0, 0, self.width(), self.height()),
                             Qt.AlignmentFlag.AlignCenter, "NO POSE")
            return
        x = self.pose["x"]
        y = self.pose["y"]
        yaw = self.pose.get("yaw", 0.0) if hasattr(self.pose, "get") else self.pose["yaw"]
        cx, cy = self.transform.to_pixel(x, y)
        size = 10.0
        # Forward direction in pixel space for a world heading `yaw`: world
        # +x is up-screen, world +y is left-screen (ViewTransform's own
        # convention), so a unit heading (cos yaw, sin yaw) maps to pixel
        # delta (-sin yaw, -cos yaw).
        fx, fy = -math.sin(yaw), -math.cos(yaw)
        perp_x, perp_y = -fy, fx
        tip = QPointF(cx + size * fx, cy + size * fy)
        back_left = QPointF(cx - size * 0.6 * fx + size * 0.5 * perp_x,
                            cy - size * 0.6 * fy + size * 0.5 * perp_y)
        back_right = QPointF(cx - size * 0.6 * fx - size * 0.5 * perp_x,
                             cy - size * 0.6 * fy - size * 0.5 * perp_y)
        painter.setPen(QPen(QColor(theme.TEXT), 1))
        painter.setBrush(QColor(theme.TEXT))
        painter.drawPolygon(QPolygonF([tip, back_left, back_right]))

    def _draw_scale_bar(self, painter) -> None:
        mpp = self.transform.metres_per_pixel
        if mpp <= 0:
            return
        metres = _nice_scale_length(mpp)
        length_px = metres / mpp
        y = self.height() - 12
        x0 = 12.0
        painter.setPen(QPen(QColor(theme.TEXT), 2))
        painter.drawLine(QPointF(x0, y), QPointF(x0 + length_px, y))
        painter.setFont(QFont(theme.MONO_FONT_FAMILY, 9))
        painter.setPen(QColor(theme.TEXT))
        painter.drawText(QPointF(x0, y - 4), f"{metres:.0f} m")
