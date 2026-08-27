"""Live camera view with its own on/off control.

Two independent facts are shown, never conflated: what the rover says
about the stream (/video_status) and whether frames are actually
arriving here. A rover reporting 'streaming' while nothing lands is the
signature of a blocked UDP port, and collapsing the two would hide
exactly that case.
"""

from time import monotonic

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QWidget

from ground_station import theme
from ground_station.video_receiver import VideoReceiver


class VideoPanel(QWidget):
    stream_requested = Signal(bool)

    def __init__(self, receiver=None, parent=None, poll_interval_ms: int = 33,
                 no_frame_after_seconds: float = 2.0):
        super().__init__(parent)
        self.receiver = receiver if receiver is not None else VideoReceiver()
        self.no_frame_after_seconds = no_frame_after_seconds
        self._streaming = False
        self._last_frame_at: float | None = None
        self._rover_state = "stopped"
        self._rover_detail = ""
        # Tracks the state the button has *requested*, separate from
        # self._streaming (which only moves once set_streaming confirms
        # it). Without this, two quick clicks before a round trip to the
        # rover completes would both request the same value.
        self._toggle_requested = False

        self.setStyleSheet(
            f"background-color: {theme.PANEL}; border: 1px solid {theme.BORDER}; "
            f"border-radius: 6px;"
        )

        title = QLabel("CAMERA / ZED FRONT LEFT")
        title.setStyleSheet(f"color: {theme.TEXT_DIM}; font-weight: 600; border: none;")

        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(self.receiver.width, self.receiver.height)
        self.image_label.setStyleSheet(f"background-color: {theme.BG}; border: none;")

        self.toggle_button = QPushButton("Start video")
        self.toggle_button.setStyleSheet(
            f"QPushButton {{ background-color: {theme.PANEL}; color: {theme.TEXT}; "
            f"border: 1px solid {theme.BORDER}; border-radius: 4px; padding: 4px 14px; }} "
            f"QPushButton:hover {{ border-color: {theme.ACCENT}; }}"
        )
        self.toggle_button.clicked.connect(self._on_toggle_clicked)

        self.status_label = QLabel("VIDEO OFF")
        self.status_label.setStyleSheet(
            f"color: {theme.TEXT_DIM}; font-family: {theme.MONO_FONT_FAMILY}; border: none;"
        )

        layout = QVBoxLayout(self)
        layout.addWidget(title)
        layout.addWidget(self.image_label, stretch=1)
        footer = QHBoxLayout()
        footer.addWidget(self.toggle_button)
        footer.addWidget(self.status_label)
        footer.addStretch()
        layout.addLayout(footer)

        self._poll_timer = QTimer(self)
        self._poll_timer.timeout.connect(self._poll_frame)
        self._poll_timer.start(poll_interval_ms)

    def _on_toggle_clicked(self) -> None:
        self._toggle_requested = not self._toggle_requested
        self.stream_requested.emit(self._toggle_requested)

    def set_streaming(self, enabled: bool) -> None:
        """Drives the local receiver. Called by the window after it has sent
        the request, and on disconnect - the receiver is stopped whether or
        not the rover ever answers."""
        self._streaming = enabled
        self._toggle_requested = enabled
        self.toggle_button.setText("Stop video" if enabled else "Start video")
        if enabled:
            self._last_frame_at = None
            self.receiver.start()
        else:
            self.receiver.stop()
            self.image_label.clear()
            self._rover_state = "stopped"
            self._rover_detail = ""
        self._refresh_status()

    def apply_status(self, status: dict) -> None:
        self._rover_state = status.get("state", "failed")
        self._rover_detail = status.get("detail", "")
        self._refresh_status()

    def _poll_frame(self, now: float | None = None) -> None:
        now = monotonic() if now is None else now
        if not self._streaming:
            return
        frame = self.receiver.read_frame()
        if frame is not None:
            self._last_frame_at = now
            image = QImage(frame, self.receiver.width, self.receiver.height,
                           self.receiver.width * 3, QImage.Format_RGB888)
            self.image_label.setPixmap(QPixmap.fromImage(image))
        elif self._last_frame_at is None:
            self._last_frame_at = now
        self._refresh_status(now)

    def _refresh_status(self, now: float | None = None) -> None:
        # The rover's reported state is shown even when this panel is not
        # locally streaming (e.g. right after apply_status arrives, before
        # set_streaming has been called) - only the "idle, nothing to
        # report" case collapses to a flat "VIDEO OFF".
        if self._rover_state == "stopped" and not self._streaming:
            self.status_label.setText("VIDEO OFF")
            self.status_label.setStyleSheet(
                f"color: {theme.TEXT_DIM}; font-family: {theme.MONO_FONT_FAMILY}; border: none;")
            return

        now = monotonic() if now is None else now
        starving = (self._last_frame_at is not None
                    and now - self._last_frame_at > self.no_frame_after_seconds)
        if starving and self._rover_state == "streaming":
            text = "NO FRAMES - rover streaming, nothing arriving (UDP blocked?)"
            color = theme.ACCENT
        elif self._rover_state == "failed":
            text = f"FAILED - {self._rover_detail}"
            color = theme.ACCENT
        elif self._rover_state == "streaming":
            text = f"STREAMING {self._rover_detail}"
            color = theme.OK
        else:
            text = self._rover_state.upper()
            color = theme.TEXT_DIM
        self.status_label.setText(text)
        self.status_label.setStyleSheet(
            f"color: {color}; font-family: {theme.MONO_FONT_FAMILY}; border: none;")
