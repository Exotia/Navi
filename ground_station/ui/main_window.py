import socket
import sys

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
                                QStackedWidget, QLineEdit, QPushButton)

from ground_station import theme
from ground_station.gamepad_input import GamepadReader
from ground_station.models import DriveState, NodeRegistry
from ground_station.ros_client import RosBridgeClient
from ground_station.ui.dashboard_page import DashboardPage
from ground_station.ui.drive_detail_page import DriveDetailPage

# The simulation streams to a different UDP port than the rover, on
# purpose: two senders can never contend and decode each other's late
# packets as garbage.
SIM_VIDEO_PORT = 5601


class MainWindow(QMainWindow):
    def __init__(self, ros_client_factory=RosBridgeClient, initial_host: str | None = None,
                 initial_port: int = 9090, node_poll_interval_ms: int = 2000,
                 staleness_check_interval_ms: int = 500, stale_after_seconds: float = 1.0,
                 gamepad_reader=None, gamepad_poll_interval_ms: int = 50,
                 video_receiver=None, video_port: int = 5600):
        super().__init__()
        self.stale_after_seconds = stale_after_seconds
        self.video_port = video_port
        self.setWindowTitle("Asterope Ground Station")
        self.setStyleSheet(f"QMainWindow {{ background-color: {theme.BG}; }}")
        self.ros_client_factory = ros_client_factory
        self.ros_client: RosBridgeClient | None = None
        self.drive_state = DriveState()
        self.node_registry = NodeRegistry()
        self.gamepad_reader = gamepad_reader if gamepad_reader is not None else GamepadReader()
        self._gamepad_was_connected = False
        # Which source the panel is showing. Several decisions depend on it:
        # the video toggle must act on the source actually on screen, and a
        # rosbridge drop must not tear down a view that does not come from
        # rosbridge. Kept here rather than read back off the radio buttons
        # so the window does not depend on the dashboard's widget layout.
        self._mode = "manual"
        # The last /localization/status seen, or None if none has arrived.
        # Kept here as well as on the panel so entering semi-autonomous can
        # show the current state immediately instead of a blank marker until
        # the next 2 Hz status message.
        self._localization_status: dict | None = None

        input_style = (
            f"background-color: {theme.PANEL}; color: {theme.TEXT}; "
            f"border: 1px solid {theme.BORDER}; border-radius: 4px; padding: 4px 8px; "
            f"font-family: {theme.MONO_FONT_FAMILY};"
        )
        button_style = (
            f"QPushButton {{ background-color: {theme.PANEL}; color: {theme.TEXT}; "
            f"border: 1px solid {theme.BORDER}; border-radius: 4px; padding: 4px 14px; }} "
            f"QPushButton:hover {{ border-color: {theme.ACCENT}; }} "
            f"QPushButton:pressed {{ background-color: {theme.BORDER}; }}"
        )

        self.connection_label = QLabel("ROSBRIDGE: DISCONNECTED")
        self.connection_label.setStyleSheet(
            f"color: {theme.TEXT_DIM}; background-color: {theme.PANEL}; "
            f"border: 1px solid {theme.BORDER}; border-radius: 4px; padding: 6px 12px; "
            f"font-family: {theme.MONO_FONT_FAMILY};"
        )

        self.host_input = QLineEdit(initial_host or "")
        self.host_input.setPlaceholderText("rosbridge host, e.g. 192.168.1.50")
        self.host_input.setStyleSheet(input_style)
        self.port_input = QLineEdit(str(initial_port))
        self.port_input.setFixedWidth(60)
        self.port_input.setStyleSheet(input_style)
        self.connect_button = QPushButton("Connect")
        self.connect_button.setStyleSheet(button_style)
        self.connect_button.clicked.connect(self._on_connect_clicked)

        title_label = QLabel("ASTEROPE GROUND STATION")
        title_label.setStyleSheet(f"color: {theme.TEXT}; font-weight: 600; font-size: 16px;")

        header = QWidget()
        header.setStyleSheet(f"background-color: {theme.BG};")
        header_layout = QHBoxLayout(header)
        header_layout.addWidget(title_label)
        header_layout.addStretch()
        header_layout.addWidget(self.host_input)
        header_layout.addWidget(self.port_input)
        header_layout.addWidget(self.connect_button)
        header_layout.addWidget(self.connection_label)

        self.dashboard_page = DashboardPage(video_receiver=video_receiver)
        self.drive_detail_page = DriveDetailPage()
        self.stacked_widget = QStackedWidget()
        self.stacked_widget.addWidget(self.dashboard_page)
        self.stacked_widget.addWidget(self.drive_detail_page)

        central = QWidget()
        central.setStyleSheet(f"background-color: {theme.BG};")
        layout = QVBoxLayout(central)
        layout.addWidget(header)
        layout.addWidget(self.stacked_widget)
        self.setCentralWidget(central)

        self.dashboard_page.drive_details_requested.connect(
            lambda: self.stacked_widget.setCurrentWidget(self.drive_detail_page)
        )
        self.drive_detail_page.back_requested.connect(
            lambda: self.stacked_widget.setCurrentWidget(self.dashboard_page)
        )
        self.dashboard_page.video_panel.stream_requested.connect(self._on_stream_requested)
        self.dashboard_page.mode_changed.connect(self._on_mode_changed)

        self._node_poll_timer = QTimer(self)
        self._node_poll_timer.timeout.connect(self._poll_nodes)
        self._node_poll_timer.start(node_poll_interval_ms)

        self._staleness_timer = QTimer(self)
        self._staleness_timer.timeout.connect(self._check_staleness)
        self._staleness_timer.start(staleness_check_interval_ms)

        self._gamepad_timer = QTimer(self)
        self._gamepad_timer.timeout.connect(self._poll_gamepad)
        self._gamepad_timer.start(gamepad_poll_interval_ms)

    def _poll_nodes(self) -> None:
        if self.ros_client is not None:
            self.ros_client.poll_nodes()

    def _poll_gamepad(self) -> None:
        """Shows the gamepad's current stick-derived Twist on the Drive
        card/detail page unconditionally - this display never depends on a
        rosbridge connection. Separately, once both a gamepad and a
        rosbridge connection are present, also publishes it on
        /manual_twist automatically (no manual "enable driving" step) - a
        raw stream nothing subscribes to yet; deciding whether this becomes
        the rover's actual /cmd_vel is a later mode-supervisor module's
        job, not this one's. On disconnect, publishes one zero-velocity
        Twist as a fail-safe stop rather than leaving the rover at its last
        command forever, then stops publishing until the gamepad returns."""
        connected = self.gamepad_reader.poll()
        rosbridge_ready = self.ros_client is not None and self.ros_client.is_connected

        if connected:
            self._gamepad_was_connected = True
            linear_x, linear_y, angular_z = self.gamepad_reader.read_twist()
            self._update_drive_display(linear_x, linear_y, angular_z)
            if rosbridge_ready:
                self.ros_client.publish_manual_twist(linear_x, linear_y, angular_z)
        elif self._gamepad_was_connected:
            self._gamepad_was_connected = False
            self._update_drive_display(0.0, 0.0, 0.0)
            if rosbridge_ready:
                self.ros_client.publish_manual_twist(0.0, 0.0, 0.0)

    def _on_connect_clicked(self) -> None:
        host = self.host_input.text().strip()
        if not host:
            return
        self._connect_to(host, self._current_rosbridge_port())

    def _current_rosbridge_port(self) -> int:
        try:
            return int(self.port_input.text().strip())
        except ValueError:
            return 9090

    def _connect_to(self, host: str, port: int) -> None:
        """(Re)connect to a rosbridge server at host:port, discarding any
        previous connection. Safe to call synchronously (from a button
        click, after the event loop is already running) or deferred via
        QTimer.singleShot(0, ...) for an initial connect at startup, so a
        slow/failed connection attempt doesn't block the window from
        painting first."""
        if self.ros_client is not None:
            try:
                self.ros_client.close()
            except Exception:
                pass

        self.ros_client = self.ros_client_factory(host, port)
        self.ros_client.signals.twist_received.connect(self._on_twist)
        self.ros_client.signals.nodes_received.connect(self._on_nodes)
        self.ros_client.signals.connection_changed.connect(self._on_connection_changed)
        self.ros_client.signals.video_status_received.connect(self._on_video_status)

        try:
            self.ros_client.connect()
            self.ros_client.subscribe_manual_twist()
            self.ros_client.subscribe_video_status()
        except Exception as exc:
            print(f"ground_station: failed to connect to rosbridge: {exc}", file=sys.stderr)

    def _update_drive_display(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        self.drive_state.ingest(linear_x, linear_y, angular_z)
        self.dashboard_page.drive_card.update_from(self.drive_state)
        self.drive_detail_page.update_from(self.drive_state)
        self.drive_detail_page.append_raw_message(
            f"linear.x={linear_x:.2f} linear.y={linear_y:.2f} angular.z={angular_z:.2f}"
        )

    def _on_twist(self, msg: dict) -> None:
        """Fires when our own /manual_twist publish loops back through
        rosbridge - a wire-level integration check, not the primary display
        path (that's _poll_gamepad calling _update_drive_display directly,
        which works even without a connection)."""
        linear = msg.get("linear", {})
        angular = msg.get("angular", {})
        self._update_drive_display(linear.get("x", 0.0), linear.get("y", 0.0), angular.get("z", 0.0))

    def _on_nodes(self, names: list) -> None:
        self.node_registry.update(names)
        self.dashboard_page.node_list.update_from(self.node_registry.snapshot())

    def _on_connection_changed(self, connected: bool) -> None:
        text = "ROSBRIDGE: CONNECTED" if connected else "ROSBRIDGE: DISCONNECTED"
        color = theme.OK if connected else theme.TEXT_DIM
        self.connection_label.setText(text)
        self.connection_label.setStyleSheet(
            f"color: {color}; background-color: {theme.PANEL}; "
            f"border: 1px solid {theme.BORDER}; border-radius: 4px; padding: 6px 12px; "
            f"font-family: {theme.MONO_FONT_FAMILY};"
        )
        if not connected:
            # A dropped rosbridge connection must not leave gst-launch-1.0
            # running: Python doesn't kill Popen children at exit, and
            # udpsrc's default reuse=true means an orphaned process keeps
            # udp/video_port bound and can swallow the *next* session's RTP
            # stream. keep_failed_reason=True: this is a disconnect, not an
            # operator stop, so a previously reported failure must survive.
            #
            # Only for the rover's stream. That reason is a rover-path
            # reason - the rover stops being asked for video and its
            # receiver has nothing left to receive - and it does not apply
            # to the simulation, whose sender is a local process on this
            # same laptop that has never heard of rosbridge. Tearing the
            # sim view down here meant a two-second rosbridge blip over
            # exactly the lossy field link this project exists for blacked
            # out a still-running, still-streaming simulation permanently,
            # under the label "VIDEO OFF" - the wording for the operator
            # having switched it off. The spec's answer to an unreachable
            # rover is that the simulation freezes *visibly*, and it does
            # that on its own: /manual_twist stops arriving and the IK node
            # zeroes the command, so the picture stops moving while frames
            # keep coming.
            if self._mode not in ("semi_auto", "simulation"):
                self.dashboard_page.video_panel.stop_receiver(keep_failed_reason=True)

    def closeEvent(self, event) -> None:
        """Stops the local video receiver on window close for the same
        reason as the rosbridge-disconnect path above - nothing else tears
        down the gst-launch-1.0 subprocess on quit."""
        self.dashboard_page.video_panel.stop_receiver(keep_failed_reason=True)
        super().closeEvent(event)

    def _check_staleness(self) -> None:
        elapsed = self.drive_state.seconds_since_last()
        if elapsed is not None and elapsed > self.stale_after_seconds:
            self.dashboard_page.drive_card.mark_stale()
            self.drive_detail_page.mark_stale()

    def local_address_for(self, host: str, port: int) -> str:
        """Our own address on the interface that reaches the rover. The
        rover cannot discover this itself - it is the server side of
        rosbridge - and a hardcoded laptop address breaks on every network
        change, so it is read back from a UDP socket connected to the
        rover (connect() on a UDP socket just assigns a local address/route
        without sending anything)."""
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect((host, port))
            return probe.getsockname()[0]
        except OSError:
            return ""
        finally:
            probe.close()

    def _request_rover_video(self, enable: bool) -> bool:
        """Asks the rover to start or stop streaming to us.

        The one place that request is built, so every caller (the panel's
        toggle, and the mode switch in both directions) sends the same
        seven arguments, discovers our own address the same way, and
        handles an undiscoverable route the same way. Returns False - with
        the reason already on the panel, never silently - if the request
        could not be sent at all.
        """
        panel = self.dashboard_page.video_panel
        if self.ros_client is None or not self.ros_client.is_connected:
            panel.apply_status({"state": "failed", "detail": "not connected to rosbridge"})
            return False

        address = ""
        if enable:
            # Probe toward the rosbridge port, not video_port: nothing is
            # actually sent on a connect()'d UDP socket, and the route to
            # the rover's rosbridge port is the same route as to its video
            # port, so this is harmless either way - but probing video_port
            # here reads as copy/paste from publish_video_request below it.
            address = self.local_address_for(self.host_input.text().strip() or "127.0.0.1",
                                             self._current_rosbridge_port())
            if not address:
                panel.apply_status({"state": "failed", "detail": "no route to the rover"})
                return False

        self.ros_client.publish_video_request(
            enable=enable, host=address, port=self.video_port,
            width=1344, height=376, fps=30, bitrate_kbps=800)
        return True

    def _on_stream_requested(self, enable: bool) -> None:
        panel = self.dashboard_page.video_panel
        if self._mode in ("simulation", "semi_auto"):
            # The toggle acts on whichever source the panel is actually
            # showing, and in both simulation modes that is the simulation -
            # a local process with no control plane at all, which streams
            # whenever it runs (by design: it is on this same laptop, so
            # there is nothing to ask). Commanding the rover's camera from
            # here would push 800 kbps of H.264 across the field link to
            # port 5600 where nothing is listening, for as long as the mode
            # lasts, undoing the only reason this mode stops it - and say
            # nothing about it.
            panel.set_streaming(enable)
            return

        if not self._request_rover_video(enable):
            return
        # The local receiver follows our own intent, not the rover's answer:
        # on disable it must stop regardless of whether the rover ever
        # replies, so a dead link cannot leave a stream pointed at us.
        panel.set_streaming(enable)

    def _on_video_status(self, status: dict) -> None:
        self.dashboard_page.video_panel.apply_status(status)

    def _on_mode_changed(self, mode: str) -> None:
        """Switches the panel's view source only. The twist keeps reaching
        the rover in every mode - driving stays on the gamepad/rosbridge
        path (_poll_gamepad), untouched here - because a mode switch that
        quietly changed what is being driven would be a control change
        wearing a view change's clothes.

        The two simulation modes show the same stream on the same port and
        differ in one thing: where the rover in the picture comes from.
        `simulation` integrates the commanded twist and says so in orange;
        `semi_auto` is placed by the rover's own /localization/pose, so the
        DEAD RECKONING warning would be false and the localisation marker
        takes its place.
        """
        panel = self.dashboard_page.video_panel
        self._mode = mode

        if mode == "autonomous":
            # Unreachable from the UI (the radio is disabled). Reached only
            # if something emits the signal directly, and then the honest
            # thing is to leave the view exactly as it is rather than
            # half-configure a panel for a mode with nothing behind it.
            print("ground_station: autonomous mode is not implemented",
                  file=sys.stderr)
            return

        if mode in ("simulation", "semi_auto"):
            # Stop the rover's camera: nobody is looking at it, and the
            # field link is the scarce resource. The rover keeps being
            # driven.
            self._request_rover_video(False)
            panel.set_source("simulation", SIM_VIDEO_PORT,
                             dead_reckoning=(mode == "simulation"),
                             reports_remote_status=False,
                             show_localization=(mode == "semi_auto"))
            if mode == "semi_auto":
                panel.set_localization_status(self._localization_status)
            return

        panel.set_source("zed front left", self.video_port)
        # Entering a simulation mode told the rover to stop streaming.
        # Leaving must tell it to start again, or the mode is a one-way door
        # for the live camera: set_source restarts the local receiver on 5600
        # so this laptop listens, but the rover was told to stop and never
        # told otherwise, so no frames ever come. What the operator then sees
        # is worse than nothing - stop_receiver has reset the rover state to
        # "stopped", so the panel shows a dim, permanent STOPPED over a black
        # picture, and the "rover says streaming but nothing arrives" branch
        # cannot fire to explain it.
        #
        # Only when the panel is actually receiving: asking the rover to
        # start streaming to a port nothing is listening on is the same waste
        # of the field link that _on_stream_requested guards against. A
        # failed request reports itself on the panel via _request_rover_video
        # rather than doing nothing quietly.
        if panel.streaming:
            self._request_rover_video(True)
