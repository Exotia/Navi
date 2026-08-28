"""Live camera view with its own on/off control.

Two independent facts are shown, never conflated, for a source that
makes a remote claim about itself: what it says over rosbridge
(/video_status) and whether frames are actually arriving here. A rover
reporting 'streaming' while nothing lands is the signature of a blocked
UDP port, and collapsing the two would hide exactly that case.

A source with no remote claim to corroborate - the simulation, which has
no /video_status of its own - only ever has the one fact: whether frames
are arriving locally. set_source's reports_remote_status flag says which
kind of source this is, and the status label shows only what is actually
known rather than a claim borrowed from a different source's protocol.
"""

from time import monotonic

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (QLabel, QPushButton, QSizePolicy, QVBoxLayout,
                               QHBoxLayout, QWidget)

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
        # Kept as bytes rather than a QImage: QImage does not copy the
        # buffer it is given, so holding one would mean holding the frame
        # alive by hand anyway. Re-wrapping on each render costs nothing.
        self._last_frame: bytes | None = None
        self._rover_state = "stopped"
        self._rover_detail = ""
        self._source_name = "zed front left"
        self._dead_reckoning = False
        # The rover has a remote self-report (/video_status); the
        # simulation does not. When False, _refresh_status shows only the
        # local fact instead of a rover claim that has nothing to do with
        # this source.
        self._reports_remote_status = True
        # Tracks the state the button has *requested*, separate from
        # self._streaming (which only moves once set_streaming confirms
        # it). Without this, two quick clicks before a round trip to the
        # rover completes would both request the same value.
        self._toggle_requested = False

        self.setStyleSheet(
            f"background-color: {theme.PANEL}; border: 1px solid {theme.BORDER}; "
            f"border-radius: 6px;"
        )

        self.title_label = QLabel()
        self.title_label.setStyleSheet(f"color: {theme.TEXT_DIM}; font-weight: 600; border: none;")

        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        # A small fixed minimum, not the stream's own size. Pinning the
        # label to 672x376 made that the floor for the whole window, and
        # the operator could not shrink the ground station below it. The
        # picture is scaled to whatever room the label ends up with.
        self.image_label.setMinimumSize(160, 90)
        self.image_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
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
        layout.addWidget(self.title_label)
        layout.addWidget(self.image_label, stretch=1)
        footer = QHBoxLayout()
        footer.addWidget(self.toggle_button)
        footer.addWidget(self.status_label)
        footer.addStretch()
        layout.addLayout(footer)

        self._poll_timer = QTimer(self)
        self._poll_timer.timeout.connect(self._poll_frame)
        self._poll_timer.start(poll_interval_ms)

        self._refresh_title()

    @property
    def dead_reckoning(self) -> bool:
        return self._dead_reckoning

    def set_source(self, name: str, port: int, *, dead_reckoning: bool = False,
                   reports_remote_status: bool = True) -> None:
        """Points the panel at a different sender.

        The receiver is stopped and re-pointed rather than a second one
        being created: two receivers would both be bound, and whichever
        the panel was not reading would silently fill its socket buffer.

        reports_remote_status says whether this source makes its own
        claim about the stream over rosbridge (the rover does; the
        simulation does not) - it decides whether _refresh_status has a
        remote claim to corroborate against, or only the local fact.
        """
        was_streaming = self._streaming
        self.stop_receiver()
        self.receiver.port = port
        self._source_name = name
        self._dead_reckoning = dead_reckoning
        self._reports_remote_status = reports_remote_status
        self._refresh_title()
        if was_streaming:
            self.set_streaming(True)

    def _refresh_title(self) -> None:
        title = f"CAMERA / {self._source_name.upper()}"
        if self._dead_reckoning:
            # The simulated pose is integrated from commanded twist, so it
            # drifts from the real rover and the picture cannot show it.
            title += "  -  DEAD RECKONING, NO LOCALISATION"
        self.title_label.setText(title)
        # This marker is the only defence an operator has against trusting
        # a synthetic view of the real machine they are driving - sitting
        # in the same muted colour as ordinary chrome ("CAMERA / ZED FRONT
        # LEFT") above a moving picture would never win the operator's
        # attention. theme.ACCENT is the same colour FAILED and the
        # blocked-port warning use, for the same reason: this is a warning.
        color = theme.ACCENT if self._dead_reckoning else theme.TEXT_DIM
        self.title_label.setStyleSheet(f"color: {color}; font-weight: 600; border: none;")

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
        self._last_frame = None
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
            self._last_frame = latest
            self._render_frame()
        elif self._last_frame_at is None:
            self._last_frame_at = now
        self._refresh_status(now)

    def _render_frame(self) -> None:
        """Draws the last frame received, scaled to the room the label has
        now. Separate from _poll_frame so a resize redraws immediately
        instead of waiting for the next frame - which on a stalled stream
        would be never."""
        if self._last_frame is None:
            return
        image = QImage(self._last_frame, self.receiver.width, self.receiver.height,
                       self.receiver.width * 3, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(image)
        target = self.image_label.size()
        if target.width() > 0 and target.height() > 0:
            # KeepAspectRatio, so the widest dimension fits and the picture
            # is letterboxed rather than stretched - a distorted camera view
            # misleads about what is in front of the rover.
            pixmap = pixmap.scaled(target, Qt.KeepAspectRatio,
                                   Qt.SmoothTransformation)
        self.image_label.setPixmap(pixmap)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._render_frame()

    def _refresh_status(self, now: float | None = None) -> None:
        if not self._reports_remote_status:
            self._refresh_local_only_status(now)
            return

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

    def _refresh_local_only_status(self, now: float | None = None) -> None:
        """The status line for a source with no remote self-report (the
        simulation). There is only one fact available - whether frames are
        actually arriving here - so this shows that alone, rather than
        borrowing the rover's /video_status vocabulary ("STREAMING",
        "rover: ...") for a source that never made any such claim."""
        if not self._streaming:
            self.status_label.setText("VIDEO OFF")
            self.status_label.setStyleSheet(
                f"color: {theme.TEXT_DIM}; font-family: {theme.MONO_FONT_FAMILY}; border: none;")
            return

        if not self.receiver.is_running:
            text = "NO FRAMES - local receiver is not running (pipeline died - check gst-launch-1.0)"
            self.status_label.setText(text)
            self.status_label.setStyleSheet(
                f"color: {theme.ACCENT}; font-family: {theme.MONO_FONT_FAMILY}; border: none;")
            return

        now = monotonic() if now is None else now
        starving = (self._last_frame_at is not None
                    and now - self._last_frame_at > self.no_frame_after_seconds)
        if starving:
            text = "NO FRAMES - nothing arriving"
            color = theme.ACCENT
        else:
            text = "RECEIVING"
            color = theme.OK
        self.status_label.setText(text)
        self.status_label.setStyleSheet(
            f"color: {color}; font-family: {theme.MONO_FONT_FAMILY}; border: none;")
