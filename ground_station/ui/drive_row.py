"""The Drive row: STOP, Manual, Init and the reset/mode buttons, plus a
status line from /drive_status. Shown when a rover drive link is wanted.
No ROS in this file - it talks through signals the window routes."""

import html
from time import monotonic

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QHBoxLayout, QLabel, QMessageBox, QPushButton,
                               QWidget)

from ground_station import theme
from ground_station.models import DriveState


def _plain(text) -> str:
    return html.escape(str(text))


class DriveRow(QWidget):
    stop_requested = Signal()
    manual_requested = Signal()
    init_requested = Signal()
    reset_encoders_requested = Signal()
    reset_odometry_requested = Signal()
    drive_mode_requested = Signal()
    drive_state_requested = Signal()

    def __init__(self, parent=None, clock=monotonic):
        super().__init__(parent)
        self._state = None
        self._clock = clock
        self.confirm_init = self._confirm_init_dialog
        self.confirm_reset_encoders = self._confirm_reset_encoders_dialog

        self.stop_button = QPushButton("STOP")
        self.manual_button = QPushButton("Manual")
        self.init_button = QPushButton("Init drive")
        self.reset_enc_button = QPushButton("Reset encoders")
        self.reset_odom_button = QPushButton("Reset odometry")
        self.mode_button = QPushButton("Drive mode")
        self.state_button = QPushButton("Drive state")
        self.status_label = QLabel()
        self.status_label.setTextFormat(Qt.TextFormat.RichText)
        self.status_label.setStyleSheet(
            f"color: {theme.TEXT_DIM}; font-family: {theme.MONO_FONT_FAMILY};")
        self.stop_button.setStyleSheet(f"color: {theme.BAD};")

        layout = QHBoxLayout(self)
        layout.addWidget(QLabel("DRIVE"))
        for b in (self.stop_button, self.manual_button, self.init_button,
                  self.reset_enc_button, self.reset_odom_button,
                  self.mode_button, self.state_button):
            layout.addWidget(b)
        layout.addWidget(self.status_label, stretch=2)

        self.stop_button.clicked.connect(self.stop_requested.emit)
        self.manual_button.clicked.connect(self.manual_requested.emit)
        self.init_button.clicked.connect(self._on_init)
        self.reset_enc_button.clicked.connect(self._on_reset_encoders)
        self.reset_odom_button.clicked.connect(self.reset_odometry_requested.emit)
        self.mode_button.clicked.connect(self.drive_mode_requested.emit)
        self.state_button.clicked.connect(self.drive_state_requested.emit)
        self.set_state(None)

    def set_state(self, state) -> None:
        self._state = state
        movable = state is not None and state.connected
        for b in (self.manual_button, self.init_button, self.reset_enc_button,
                  self.reset_odom_button, self.mode_button, self.state_button):
            b.setEnabled(movable)
        self.stop_button.setEnabled(True)          # always
        self._refresh_status()

    def refresh(self, now=None) -> None:
        self._refresh_status(now)

    def _refresh_status(self, now=None) -> None:
        if self._state is None:
            self.status_label.setText("DRIVE: no status")
            return
        s = self._state
        parts = []
        if not s.connected:
            parts.append(f'<span style="color: {theme.BAD};">disconnected</span>')
        else:
            parts.append(_plain(f"lease {'held' if s.lease else 'none'}"))
        if s.coordinator_state == "PrepareManual":
            parts.append(_plain("arming (5 s)"))
        elif s.coordinator_state:
            parts.append(_plain(s.coordinator_state))
        if s.deadman_active:
            parts.append(f'<span style="color: {theme.ACCENT};">deadman</span>')
        if s.twist_age_s is not None:
            parts.append(_plain(f"twist {s.twist_age_s:.1f}s"))
        if s.last_error:
            parts.append(f'<span style="color: {theme.BAD};">'
                         f'{_plain(s.last_error)}</span>')
        self.status_label.setText(" | ".join(parts))

    def _on_init(self):
        if self.confirm_init():
            self.init_requested.emit()

    def _on_reset_encoders(self):
        if self.confirm_reset_encoders():
            self.reset_encoders_requested.emit()

    def _confirm_init_dialog(self) -> bool:
        answer = QMessageBox.question(
            self, "Init drive",
            "Initialise the drive? The wheels will move to zero the steering.")
        return answer == QMessageBox.StandardButton.Yes

    def _confirm_reset_encoders_dialog(self) -> bool:
        answer = QMessageBox.question(
            self, "Reset encoders",
            "Reset the steering encoders? The wheels will move.")
        return answer == QMessageBox.StandardButton.Yes
