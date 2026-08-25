from PySide6.QtCore import Signal
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel

from ground_station import theme
from ground_station.models import DriveState


class DriveCard(QWidget):
    details_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(
            f"background-color: {theme.PANEL}; border: 1px solid {theme.BORDER}; border-radius: 6px;"
        )

        title = QLabel("DRIVE / TWIST")
        title.setStyleSheet(f"color: {theme.TEXT_DIM}; font-weight: 600; border: none;")

        self.vx_label = QLabel("vx (cmd)  --")
        self.vy_label = QLabel("vy (cmd)  --")
        self.wz_label = QLabel("wz (cmd)  --")
        self.rate_label = QLabel("/cmd_vel  --")
        for label in (self.vx_label, self.vy_label, self.wz_label, self.rate_label):
            label.setStyleSheet(f"color: {theme.TEXT}; font-family: {theme.MONO_FONT_FAMILY}; border: none;")

        self.details_link = QLabel("view details →")
        self.details_link.setStyleSheet(f"color: {theme.ACCENT}; border: none;")
        self.details_link.mousePressEvent = self._on_details_clicked

        layout = QVBoxLayout(self)
        layout.addWidget(title)
        for label in (self.vx_label, self.vy_label, self.wz_label, self.rate_label):
            layout.addWidget(label)
        footer = QHBoxLayout()
        footer.addStretch()
        footer.addWidget(self.details_link)
        layout.addLayout(footer)

    def _on_details_clicked(self, event):
        self.details_requested.emit()

    def update_from(self, state: DriveState) -> None:
        if state.latest is None:
            return
        self.vx_label.setText(f"vx (cmd)  {state.latest.linear_x:.2f} m/s")
        self.vy_label.setText(f"vy (cmd)  {state.latest.linear_y:.2f} m/s")
        self.wz_label.setText(f"wz (cmd)  {state.latest.angular_z:.2f} rad/s")
        self.rate_label.setText(f"/cmd_vel  {state.rate_hz:.0f} Hz")
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
