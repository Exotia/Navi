"""The Drive row: STOP, Manual, Init and the reset/mode buttons, plus a
status area of small chips fed from /drive_status. Shown when a rover
drive link is wanted. No ROS in this file - it talks through signals the
window routes."""

from time import monotonic

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QHBoxLayout, QLabel, QMessageBox, QPushButton,
                               QWidget)

from ground_station import theme
from ground_station.models import DriveState


def _plain(text) -> str:
    """Text as itself. The pills below are set to Qt::PlainText explicitly
    (see set_state), so - unlike the old rich-text status line - nothing
    here is ever parsed as markup: a stray '<' from the rover's last_error
    is shown as a literal character, not swallowed or escaped-and-shown-
    literally."""
    return str(text)


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
        self._init_sent = False
        self.confirm_init = self._confirm_init_dialog
        self.confirm_reset_encoders = self._confirm_reset_encoders_dialog

        self.setObjectName("driveRow")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet(f"QWidget#driveRow {{ {theme.card_style()} }}")

        self.stop_button = QPushButton("STOP")
        self.manual_button = QPushButton("Manual")
        self.init_button = QPushButton("Init drive")
        self.reset_enc_button = QPushButton("Reset encoders")
        self.reset_odom_button = QPushButton("Reset odometry")
        self.mode_button = QPushButton("Drive mode")
        self.state_button = QPushButton("Drive state")

        # STOP must be unmissable: filled BAD, bold white text, generously
        # sized - this is the button that gets the rover to stop moving.
        self.stop_button.setStyleSheet(
            f"QPushButton {{ background-color: {theme.BAD}; color: white; "
            f"font-weight: 700; border: none; border-radius: 6px; "
            f"padding: 6px 14px; }} "
            f"QPushButton:hover {{ background-color: #ff6259; }} "
            f"QPushButton:pressed {{ background-color: #d9433a; }}"
        )
        self.stop_button.setMinimumHeight(34)
        self.stop_button.setMinimumWidth(90)

        # These move the rover's wheels - a danger hint, not a full danger
        # fill (the buttons stay usable at a glance, just bordered BAD).
        danger_border = (
            f"QPushButton {{ border: 1px solid {theme.BAD}; }}"
        )
        self.init_button.setStyleSheet(danger_border)
        self.reset_enc_button.setStyleSheet(danger_border)

        # --- status chips -------------------------------------------------
        # Small rounded pills instead of a " | "-joined string. Every
        # dynamic value is either plain setText (no rich text needed inside
        # a pill) or, where a value could contain anything (last_error, from
        # the rover verbatim), still passed through _plain first.
        self.no_status_label = QLabel("DRIVE: no status")
        self.no_status_label.setStyleSheet(f"color: {theme.TEXT_DIM}; border: none;")

        self.lease_pill = QLabel()
        self.state_pill = QLabel()
        self.deadman_pill = QLabel("DEADMAN")
        self.deadman_pill.setStyleSheet(theme.pill_style(theme.ACCENT, "#2a1600"))
        self.twist_age_label = QLabel()
        self.twist_age_label.setStyleSheet(
            f"color: {theme.TEXT_DIM}; font-family: {theme.MONO_FONT_FAMILY}; border: none;")
        self.last_action_label = QLabel()
        self.last_action_label.setStyleSheet(f"color: {theme.TEXT_DIM}; border: none;")
        self.error_pill = QLabel()
        self.error_pill.setStyleSheet(theme.pill_style(theme.BAD, "white"))
        # Every chip that ever carries rover-supplied text (coordinator
        # state, twist age, last action, last_error) is pinned to plain
        # text - never auto-detected as rich text - so a stray '<' from the
        # rover is shown as itself instead of being parsed as markup.
        for w in (self.lease_pill, self.state_pill, self.twist_age_label,
                  self.last_action_label, self.error_pill):
            w.setTextFormat(Qt.TextFormat.PlainText)

        self.status_layout = QHBoxLayout()
        self.status_layout.addWidget(self.no_status_label)
        for w in (self.lease_pill, self.state_pill, self.deadman_pill,
                  self.twist_age_label, self.last_action_label, self.error_pill):
            self.status_layout.addWidget(w)
        self.status_layout.addStretch()

        layout = QHBoxLayout(self)
        title = QLabel("DRIVE")
        title.setStyleSheet(theme.section_title_style())
        layout.addWidget(title)
        for b in (self.stop_button, self.manual_button, self.init_button,
                  self.reset_enc_button, self.reset_odom_button,
                  self.mode_button, self.state_button):
            layout.addWidget(b)
        layout.addLayout(self.status_layout, stretch=2)

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
        for b in (self.manual_button, self.reset_enc_button,
                  self.reset_odom_button, self.mode_button, self.state_button):
            b.setEnabled(movable)
        self.init_button.setEnabled(movable and not self._init_sent)
        self.stop_button.setEnabled(True)          # always
        self._refresh_status()

    def refresh(self, now=None) -> None:
        self._refresh_status(now)

    def _refresh_status(self, now=None) -> None:
        s = self._state
        if s is None:
            self.no_status_label.setVisible(True)
            for w in (self.lease_pill, self.state_pill, self.deadman_pill,
                      self.twist_age_label, self.last_action_label, self.error_pill):
                w.setVisible(False)
            return
        self.no_status_label.setVisible(False)

        # LEASE pill: green OK when held, grey OFF otherwise.
        if s.lease:
            self.lease_pill.setText("LEASE HELD")
            self.lease_pill.setStyleSheet(theme.pill_style(theme.OK, "#0c1a0e"))
        else:
            self.lease_pill.setText("LEASE OFF")
            self.lease_pill.setStyleSheet(theme.pill_style(theme.OFF, theme.TEXT))
        self.lease_pill.setVisible(True)

        # Coordinator-state pill: Manual -> OK green, PrepareManual ->
        # ACCENT orange showing "ARMING 5s", Idle -> OFF grey,
        # disconnected/no status -> BAD red.
        if not s.connected:
            self.state_pill.setText("DISCONNECTED")
            self.state_pill.setStyleSheet(theme.pill_style(theme.BAD, "white"))
        elif s.coordinator_state == "Manual":
            self.state_pill.setText("MANUAL")
            self.state_pill.setStyleSheet(theme.pill_style(theme.OK, "#0c1a0e"))
        elif s.coordinator_state == "PrepareManual":
            self.state_pill.setText("ARMING 5s")
            self.state_pill.setStyleSheet(theme.pill_style(theme.ACCENT, "#2a1600"))
        elif s.coordinator_state == "Idle":
            self.state_pill.setText("IDLE")
            self.state_pill.setStyleSheet(theme.pill_style(theme.OFF, theme.TEXT))
        elif s.coordinator_state:
            self.state_pill.setText(_plain(s.coordinator_state.upper()))
            self.state_pill.setStyleSheet(theme.pill_style(theme.OFF, theme.TEXT))
        else:
            self.state_pill.setText("NO STATE")
            self.state_pill.setStyleSheet(theme.pill_style(theme.BAD, "white"))
        self.state_pill.setVisible(True)

        # DEADMAN pill: shown only when active.
        self.deadman_pill.setVisible(bool(s.deadman_active))

        # Twist age, in mono - a numeric readout.
        if s.twist_age_s is not None:
            self.twist_age_label.setText(_plain(f"twist {s.twist_age_s:.1f}s"))
            self.twist_age_label.setVisible(True)
        else:
            self.twist_age_label.setVisible(False)

        # Last action, dimmed.
        if s.last_action:
            self.last_action_label.setText(_plain(f"last: {s.last_action}"))
            self.last_action_label.setVisible(True)
        else:
            self.last_action_label.setVisible(False)

        # Last error, in BAD when present.
        if s.last_error:
            self.error_pill.setText(_plain(s.last_error))
            self.error_pill.setVisible(True)
        else:
            self.error_pill.setVisible(False)

    def _on_init(self):
        if self.confirm_init():
            # BemaServer::init() is one-shot (a second F0 is ignored by the
            # rover), so the button follows suit for this GS session.
            self._init_sent = True
            self.init_button.setEnabled(False)
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
