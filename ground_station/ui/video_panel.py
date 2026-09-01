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
from ground_station.video_receiver import VideoReceiver, parse_geometry


def localization_marker(status: dict | None) -> tuple[str, str]:
    """The marker text and colour for a /localization/status payload.

    Colours are the panel's existing two: theme.OK for the one state that
    means the picture can be trusted, theme.ACCENT for every state that
    should stop the operator - which is the same colour FAILED, NO FRAMES
    and the DEAD RECKONING title marker already use, for the same reason.

    None is not the same as OFF and is not shown as OFF: "the rover says
    localisation is off" and "we have heard nothing from the rover" point at
    different machines, and the second one is usually rosbridge.
    """
    if status is None:
        return "NO LOCALISATION STATUS", theme.ACCENT

    state = str(status.get("state", ""))
    if state == "OK":
        return "LOCALISED", theme.OK
    if state == "SEARCHING":
        seconds = status.get("seconds_since_ok")
        if isinstance(seconds, (int, float)):
            # The count is the content: "searching" alone does not say
            # whether this is a blip over a rut or the rover being lost.
            return f"SEARCHING … {seconds:.0f} s", theme.ACCENT
        return "SEARCHING", theme.ACCENT
    if state == "OFF":
        return "LOCALISATION OFF", theme.ACCENT
    # An unrecognised state is shown, not swallowed: a status this panel
    # cannot interpret is itself worth seeing.
    return f"LOCALISATION {state.upper()}", theme.ACCENT


class AspectLabel(QLabel):
    """A label whose height follows its width at the stream's aspect ratio.

    Without this the picture was letterboxed inside whatever rectangle the
    layout handed out - a 16:9 camera in a 3:1 slot is mostly black bars,
    and the bars are the operator's screen space. The panel now takes the
    shape of what the camera actually sends.
    """

    # (u, v, width, height): a click, in SOURCE-frame pixels - the stream's
    # own width/height, not the label's. See mousePressEvent below for why
    # a raw event position cannot be used directly.
    clicked = Signal(int, int, int, int)

    def __init__(self, text: str = "", parent=None):
        super().__init__(text, parent)
        self._aspect = 16.0 / 9.0
        # The dimensions of the frame currently painted (the receiver's
        # own width/height), kept so a click can be rescaled from label
        # pixels back to source pixels. None until a frame has actually
        # been rendered.
        self._source_width: int | None = None
        self._source_height: int | None = None

    def set_aspect(self, width: int, height: int) -> None:
        if width > 0 and height > 0:
            aspect = float(width) / float(height)
            if abs(aspect - self._aspect) > 1e-6:
                self._aspect = aspect
                self.updateGeometry()

    def set_source_size(self, width: int, height: int) -> None:
        """The stream's own dimensions, refreshed on every render - what
        `mousePressEvent` rescales a click into."""
        self._source_width = width
        self._source_height = height

    @property
    def aspect(self) -> float:
        return self._aspect

    def hasHeightForWidth(self) -> bool:
        return True

    def heightForWidth(self, width: int) -> int:
        return int(round(max(1, width) / self._aspect))

    def mousePressEvent(self, event) -> None:
        """Turns a click into a SOURCE-frame pixel.

        `_render_frame` scales the pixmap into this label with
        Qt.KeepAspectRatio and centres it (AlignCenter), so the picture is
        letterboxed inside the label - a raw event position is offset by
        the bars and scaled by an unknown factor, and a probe built from it
        would measure the wrong part of the world, quietly, with a
        plausible-looking number. So: take the rendered pixmap's size,
        subtract the letterbox origin, reject a click that landed in the
        bars, then rescale by source/pixmap to land on the actual pixel
        the operator pointed at.
        """
        pixmap = self.pixmap()
        if (pixmap is not None and not pixmap.isNull()
                and self._source_width is not None
                and self._source_height is not None):
            pw, ph = pixmap.width(), pixmap.height()
            if pw > 0 and ph > 0:
                origin_x = (self.width() - pw) / 2.0
                origin_y = (self.height() - ph) / 2.0
                pos = event.position()
                x = pos.x() - origin_x
                y = pos.y() - origin_y
                if 0 <= x < pw and 0 <= y < ph:
                    u = int(x * self._source_width / pw)
                    v = int(y * self._source_height / ph)
                    self.clicked.emit(u, v, self._source_width, self._source_height)
        super().mousePressEvent(event)


class VideoPanel(QWidget):
    stream_requested = Signal(bool)

    def __init__(self, receiver=None, parent=None, poll_interval_ms: int = 33,
                 no_frame_after_seconds: float = 2.0,
                 refusal_seconds: float = 5.0):
        super().__init__(parent)
        self.receiver = receiver if receiver is not None else VideoReceiver()
        # The geometry a fresh source is decoded at. The rover's status can
        # move the receiver to the size it actually streams (640x360), and
        # that must not leak into the next source: the simulation sends
        # 672x376 and a decoder left at 640x360 dies with 'not-negotiated'.
        self._default_geometry = (self.receiver.width, self.receiver.height)
        self.no_frame_after_seconds = no_frame_after_seconds
        self.refusal_seconds = refusal_seconds
        # A refusal is neither a rover claim nor a local fact, so it does not
        # fit either status path - it is an answer to a button press. Held
        # for refusal_seconds and then dropped, so the ordinary status line
        # comes back on its own.
        self._refusal: str | None = None
        self._refusal_until = 0.0
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
        self._show_localization = False
        self._localization_status: dict | None = None
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

        self.setObjectName("videoPanel")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet(f"QWidget#videoPanel {{ {theme.card_style()} }}")

        self.title_label = QLabel()
        self.title_label.setStyleSheet(theme.section_title_style())

        self.image_label = AspectLabel("NO SIGNAL")
        self.image_label.setAlignment(Qt.AlignCenter)
        # A small fixed minimum, not the stream's own size. Pinning the
        # label to 672x376 made that the floor for the whole window, and
        # the operator could not shrink the ground station below it. The
        # picture is scaled to whatever room the label ends up with.
        self.image_label.setMinimumSize(160, 90)
        # Preferred (not Ignored) with heightForWidth honoured: the label
        # asks the layout for the stream's own shape instead of accepting
        # any rectangle and letterboxing inside it.
        policy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        policy.setHeightForWidth(True)
        self.image_label.setSizePolicy(policy)
        # The empty state (no pixmap set yet) shows dimmed, centred text
        # rather than a flat, unexplained rectangle; setPixmap later draws
        # over this the moment a frame arrives.
        self.image_label.setStyleSheet(
            f"background-color: {theme.BG}; color: {theme.TEXT_DIM}; border: none;"
        )

        self.toggle_button = QPushButton("Start video")
        self.toggle_button.setStyleSheet(theme.button_style())
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

    @property
    def streaming(self) -> bool:
        """Whether the local receiver is running - which is also the answer
        to "is anything on this laptop listening for a stream". MainWindow
        needs it to decide whether asking the rover to (re)start streaming
        would send frames to a port with no listener."""
        return self._streaming

    def set_source(self, name: str, port: int, *, dead_reckoning: bool = False,
                   reports_remote_status: bool = True,
                   show_localization: bool = False) -> None:
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
        self.receiver.width, self.receiver.height = self._default_geometry
        self._source_name = name
        self._dead_reckoning = dead_reckoning
        self._reports_remote_status = reports_remote_status
        self._show_localization = show_localization
        self._refusal = None
        self._refresh_title()
        if was_streaming:
            self.set_streaming(True)

    def set_localization_status(self, status: dict | None) -> None:
        """The rover's localisation health, as parsed from
        /localization/status. Stored whatever the mode: the panel decides
        whether it is on screen (set_source's show_localization), so the
        window can forward every message without knowing the mode."""
        self._localization_status = status
        self._refresh_title()

    def _refresh_title(self) -> None:
        title = f"CAMERA / {self._source_name.upper()}"
        colour = theme.TEXT_DIM
        if self._dead_reckoning:
            # The simulated pose is integrated from commanded twist, so it
            # drifts from the real rover and the picture cannot show it.
            title += "  -  DEAD RECKONING, NO LOCALISATION"
            colour = theme.ACCENT
        elif self._show_localization:
            marker, colour = localization_marker(self._localization_status)
            title += f"  -  {marker}"
        self.title_label.setText(title)
        # This marker is the only defence an operator has against trusting a
        # synthetic view of the real machine they are driving - sitting in
        # the same muted colour as ordinary chrome ("CAMERA / ZED FRONT
        # LEFT") above a moving picture would never win the operator's
        # attention. The failure colours are the same ones FAILED and the
        # blocked-port warning use, for the same reason: these are warnings.
        self.title_label.setStyleSheet(
            f"color: {colour}; font-size: {theme.FONT_SIZE_SMALL}px; font-weight: 600; "
            f"letter-spacing: 1px; border: none;"
        )

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
        self.image_label.setText("NO SIGNAL")
        if not (keep_failed_reason and self._rover_state == "failed"):
            self._rover_state = "stopped"
            self._rover_detail = ""
        self._refresh_status()

    def _follow_reported_geometry(self) -> None:
        """Restarts the local receiver at the size the rover says it is
        streaming. The rover's zed_topic source sends whatever the ZED
        wrapper publishes (640x360 today, not the 672x376 the UVC path
        produced) and names it in the status detail; a decode pipeline
        pinned to the wrong size fails caps negotiation and never emits a
        frame, which looks exactly like a blocked UDP port. Only while the
        operator has the stream on: a stale detail from before a stop must
        not start a receiver that was switched off."""
        # The simulation source reports no remote status: a late rover
        # status (published before the rover processed its stop) landing
        # after the switch must not re-point the receiver at the rover's
        # 640x360 and break the 672x376 sim stream.
        if not self._reports_remote_status:
            return
        if self._rover_state != "streaming" or not self._streaming:
            return
        geometry = parse_geometry(self._rover_detail)
        if geometry is None or geometry == (self.receiver.width, self.receiver.height):
            return
        self.receiver.stop()
        self.receiver.width, self.receiver.height = geometry
        self._last_frame_at = None
        self._last_frame = None
        try:
            self.receiver.start()
        except Exception as exc:
            # Same contract as set_streaming: a launch failure is shown as
            # the cause, not left as a silent "Stop video" with no receiver.
            self._streaming = False
            self._toggle_requested = False
            self.toggle_button.setText("Start video")
            self._rover_state = "failed"
            self._rover_detail = f"local receiver failed to start: {exc}"
            self._refresh_status()

    def apply_status(self, status: dict) -> None:
        self._rover_state = status.get("state", "failed")
        self._rover_detail = status.get("detail", "")
        self._follow_reported_geometry()
        if self._rover_state == "failed" and not self._streaming:
            # A rejection (e.g. "not connected to rosbridge", "no route to
            # the rover") happens before set_streaming is ever called, so
            # without this _toggle_requested would stay stuck at whatever
            # the triggering click just emitted. The operator's next click
            # is a retry, not a reversal - it must emit the same intent
            # again, not its opposite.
            self._toggle_requested = False
        self._refresh_status()

    def refuse_stream(self, reason: str, now: float | None = None) -> None:
        """Answers a stream request that will not be honoured.

        Nothing about the view changes: the source on screen was not what
        was requested, and stopping it would punish the operator for
        asking. Only the button is put back where reality is - the click
        already flipped _toggle_requested, and leaving it flipped would make
        the next click send the opposite of what the operator means.
        """
        now = monotonic() if now is None else now
        self._refusal = reason
        self._refusal_until = now + self.refusal_seconds
        self._toggle_requested = self._streaming
        self.toggle_button.setText("Stop video" if self._streaming else "Start video")
        self._refresh_status(now)

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
        # The stream's real dimensions drive the panel's shape, so a
        # source that is not 16:9 (or a resolution change) reshapes the
        # card rather than growing bars inside it.
        self.image_label.set_aspect(self.receiver.width, self.receiver.height)
        self.image_label.set_source_size(self.receiver.width, self.receiver.height)
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
        now = monotonic() if now is None else now

        if self._refusal is not None:
            if now < self._refusal_until:
                self.status_label.setText(self._refusal)
                self.status_label.setStyleSheet(
                    f"color: {theme.ACCENT}; font-family: {theme.MONO_FONT_FAMILY}; border: none;")
                return
            self._refusal = None

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

    def _refresh_local_only_status(self, now: float) -> None:
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
