from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel

from ground_station import theme
from ground_station.models import DriveCommandTracker
from ground_station.ui.wheel_view import WheelView


class DriveCard(QWidget):
    details_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("driveCard")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet(f"QWidget#driveCard {{ {theme.card_style()} }}")

        title = QLabel("DRIVE / TWIST")
        title.setStyleSheet(theme.section_title_style())

        self.vx_label = QLabel("vx (cmd)  --")
        self.vy_label = QLabel("vy (cmd)  --")
        self.wz_label = QLabel("wz (cmd)  --")
        self.rate_label = QLabel("/cmd_vel  --")
        for label in (self.vx_label, self.vy_label, self.wz_label, self.rate_label):
            label.setStyleSheet(f"color: {theme.TEXT}; font-family: {theme.MONO_FONT_FAMILY}; border: none;")

        self.details_link = QLabel("view details →")
        self.details_link.setStyleSheet(f"color: {theme.ACCENT}; border: none;")
        self.details_link.mousePressEvent = self._on_details_clicked

        # The numbers say what was commanded; the picture beside them says
        # what the chassis is about to look like, which is the thing an
        # operator can check against the rover in front of them.
        self.wheel_view = WheelView()

        numbers = QVBoxLayout()
        numbers.setSpacing(2)
        numbers.addWidget(title)
        for label in (self.vx_label, self.vy_label, self.wz_label, self.rate_label):
            numbers.addWidget(label)
        numbers.addStretch()
        footer = QHBoxLayout()
        footer.addStretch()
        footer.addWidget(self.details_link)
        numbers.addLayout(footer)

        layout = QHBoxLayout(self)
        layout.setSpacing(10)
        layout.addLayout(numbers, stretch=1)
        layout.addWidget(self.wheel_view)

    def _on_details_clicked(self, event):
        self.details_requested.emit()

    def update_from(self, state: DriveCommandTracker) -> None:
        if state.latest is None:
            return
        self.vx_label.setText(f"vx (cmd)  {state.latest.linear_x:.2f} m/s")
        self.vy_label.setText(f"vy (cmd)  {state.latest.linear_y:.2f} m/s")
        self.wz_label.setText(f"wz (cmd)  {state.latest.angular_z:.2f} rad/s")
        self.rate_label.setText(f"/cmd_vel  {state.rate_hz:.0f} Hz")
        self.wheel_view.set_twist(state.latest.linear_x, state.latest.linear_y,
                                  state.latest.angular_z)
        self.rate_label.setStyleSheet(
            f"color: {theme.TEXT}; font-family: {theme.MONO_FONT_FAMILY}; border: none;"
        )

    def mark_stale(self) -> None:
        """No new /cmd_vel sample has arrived recently — show 0 Hz / no-data
        rather than letting the last-computed rate sit there looking live."""
        self.rate_label.setText("/cmd_vel  0 Hz (no data)")
        self.rate_label.setStyleSheet(
            f"color: {theme.TEXT_DIM}; font-family: {theme.MONO_FONT_FAMILY}; border: none;"
        )
