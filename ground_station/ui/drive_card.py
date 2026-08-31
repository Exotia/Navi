from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel

from ground_station import theme
from ground_station.models import DriveCommandTracker
from ground_station.ui.wheel_view import WheelView


class DriveCard(QWidget):
    """What the wheels are doing, as a picture.

    The vx/vy/wz/rate readouts that used to fill this card are gone: four
    lines of numbers said less than the steering picture says, and they
    cost the camera and the plan map the vertical space they occupied. The
    numbers still exist behind "numbers →" (DriveDetailPage), which is
    where a value you want to read digit by digit belongs.
    """

    details_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("driveCard")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet(f"QWidget#driveCard {{ {theme.card_style()} }}")

        title = QLabel("WHEELS")
        title.setStyleSheet(theme.section_title_style())

        self.wheel_view = WheelView()

        self.details_link = QLabel("numbers →")
        self.details_link.setStyleSheet(f"color: {theme.ACCENT}; border: none;")
        self.details_link.setToolTip(
            "The vx / vy / wz readouts and the raw /cmd_vel log.")
        self.details_link.mousePressEvent = self._on_details_clicked

        header = QHBoxLayout()
        header.addWidget(title)
        header.addStretch()
        header.addWidget(self.details_link)

        layout = QVBoxLayout(self)
        layout.setSpacing(4)
        layout.addLayout(header)
        layout.addWidget(self.wheel_view, stretch=1)

    def _on_details_clicked(self, event):
        self.details_requested.emit()

    def update_from(self, state: DriveCommandTracker) -> None:
        if state.latest is None:
            return
        self.wheel_view.set_twist(state.latest.linear_x, state.latest.linear_y,
                                  state.latest.angular_z)

    def mark_stale(self) -> None:
        """No fresh /cmd_vel sample. The wheels keep the geometry they had -
        that is still where the steering is - but the caption stops claiming
        the command is current."""
        self.wheel_view.mark_stale()
