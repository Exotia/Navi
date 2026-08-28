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
        the request - the receiver is stopped whether or not the rover ever
        answers. Disconnect/shutdown goes through stop_receiver() instead,
        so a previously reported failure reason can survive."""
        if not enabled:
            self.stop_receiver()
            return
        self._last_frame_at = None
        try:
            self.receiver.start()
        except Exception as exc:
            # A missing gst-launch-1.0 (or any other launch failure) must
            # not leave the panel half-updated: streaming=True, button
            # reading "Stop video", and _refresh_status never reached -
            # that would show the same misleading "UDP blocked?" verdict
            # 2s later instead of the real cause.
            self._streaming = False
            self._toggle_requested = False
            self.toggle_button.setText("Start video")
            self._rover_state = "failed"
            self._rover_detail = f"local receiver failed to start: {exc}"
            self._refresh_status()
            return
        self._streaming = True
        self._toggle_requested = True
        self.toggle_button.setText("Stop video")
        self._refresh_status()

    def stop_receiver(self, *, keep_failed_reason: bool = False) -> None:
        """Stops the local receiver. Used both for an operator-initiated
        stop (set_streaming(False)) and for rosbridge disconnect/window
        close (MainWindow), where keep_failed_reason=True preserves a
        previously reported 'failed' state instead of overwriting it with
        'stopped' - a shutdown didn't just happen because of a fresh
        failure, and erasing the reason would hide what did happen."""
        self._streaming = False
        self._toggle_requested = False
        self.toggle_button.setText("Start video")
        self.receiver.stop()
        self.image_label.clear()
        if not (keep_failed_reason and self._rover_state == "failed"):
            self._rover_state = "stopped"
            self._rover_detail = ""
        self._refresh_status()

    def apply_status(self, status: dict) -> None:
        self._rover_state = status.get("state", "failed")
        self._rover_detail = status.get("detail", "")
        if self._rover_state == "failed" and not self._streaming:
            # A rejection (e.g. "not connected to rosbridge", "no route to
            # the rover") happens before set_streaming is ever called, so
            # without this _toggle_requested would stay stuck at whatever
            # the triggering click just emitted. The operator's next click
            # is a retry, not a reversal - it must emit the same intent
            # again, not its opposite.
            self._toggle_requested = False
        self._refresh_status()

    def _poll_frame(self, now: float | None = None) -> None:
        now = monotonic() if now is None else now
        if not self._streaming:
            return
        # VideoReceiver hands back the newest frame it has and drops the
        # rest, so this loop normally runs once. It stays a loop because
        # painting the oldest of several available frames would hold a
        # backlog - and its latency - on screen, and this panel must not
        # depend on which receiver it was given to avoid that.
        #
        # This is not what keeps the stream live. The pipe is drained by
        # the receiver's own thread; draining it from this timer capped
        # the panel at 2.2 fps because a 758,016-byte frame is about
        # twelve times a pipe's capacity. See video_receiver's docstring.
        latest = None
        while (frame := self.receiver.read_frame()) is not None:
            latest = frame
        if latest is not None:
            self._last_frame_at = now
            image = QImage(latest, self.receiver.width, self.receiver.height,
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

        # A rover failure is worth surfacing immediately, even before this
        # panel starts polling locally - there is nothing to corroborate
        # and nothing to wait for.
        if self._rover_state == "failed":
            self.status_label.setText(f"FAILED - {self._rover_detail}")
            self.status_label.setStyleSheet(
                f"color: {theme.ACCENT}; font-family: {theme.MONO_FONT_FAMILY}; border: none;")
            return

        if not self._streaming:
            # The rover reports "streaming" or "starting" but this panel
            # has not (yet) been told to poll for frames - _poll_frame
            # never runs, so _last_frame_at can never advance and the
            # starving check below would never fire. Rendering the rover's
            # word alone here would be exactly the conflation this panel
            # exists to avoid: a claim with no local corroboration must
            # never read as plain success.
            self.status_label.setText(
                f"rover: {self._rover_state} (not receiving locally)")
            self.status_label.setStyleSheet(
                f"color: {theme.TEXT_DIM}; font-family: {theme.MONO_FONT_FAMILY}; border: none;")
            return

        if not self.receiver.is_running:
            # video_receiver.py's own module docstring names this exact
            # case: a caps mismatch, a missing decoder, a udpsrc bind
            # failure, or a decoder crash makes the pipeline die - and
            # without this check that read as "UDP blocked?", sending the
            # operator to the network firewall when the fault is on their
            # own laptop.
            text = "NO FRAMES - local receiver is not running (pipeline died - check gst-launch-1.0)"
            self.status_label.setText(text)
            self.status_label.setStyleSheet(
                f"color: {theme.ACCENT}; font-family: {theme.MONO_FONT_FAMILY}; border: none;")
            return

        now = monotonic() if now is None else now
        starving = (self._last_frame_at is not None
                    and now - self._last_frame_at > self.no_frame_after_seconds)
        if starving and self._rover_state in ("streaming", "starting"):
            text = "NO FRAMES - rover streaming, nothing arriving (UDP blocked?)"
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
