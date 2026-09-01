import math
import socket
import sys
from time import monotonic, time

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
                                QStackedWidget, QLineEdit, QPushButton)

from ground_station import theme
from ground_station.gamepad_input import GamepadReader
from ground_station.landmark_table import LandmarkTableError, load_landmark_table
from ground_station.models import (NAV_ACTIVE_STATES, DriveCommandTracker,
                                   NodeRegistry, Waypoint, is_stick_deflected,
                                   may_publish_manual_twist,
                                   may_publish_takeover_twist, new_probe_id,
                                   new_run_id)
from ground_station.ros_client import RosBridgeClient
from ground_station.site_frame import (reexpress_at_lock_pose, site_to_map,
                                       site_yaw_to_map_yaw)
from ground_station.ui.dashboard_page import DashboardPage
from ground_station.ui.drive_detail_page import DriveDetailPage
from ground_station.ui.mission_timer import MissionTimer
from ground_station.ui.star_logo import StarLogo

# The simulation streams to a different UDP port than the rover, on
# purpose: two senders can never contend and decode each other's late
# packets as garbage.
SIM_VIDEO_PORT = 5601

# The exact words the panel shows when the operator asks for the rover's
# camera in semi-autonomous mode. One constant, because the test asserts on
# it and the spec fixes the wording.
SEMI_AUTO_REFUSAL = "no camera stream in semi-autonomous mode"

# /localization/status arrives at 2 Hz, so three seconds is six missed
# messages: the rover is gone, not hiccuping. After that the marker stops
# asserting a health nobody has confirmed since - it reads NO LOCALISATION
# STATUS, which is what is actually true.
LOCALIZATION_STATUS_STALE_AFTER_SECONDS = 3.0

# What we ask the rover's camera for, and it must match what the camera
# actually delivers: with localisation running, the frames come from the
# ZED wrapper, which grabs at `general.grab_frame_rate: 15` (see
# rover/src/navi_localization/config/zed_front.yaml). The rover's
# video_sender puts this number straight into its pipeline as
# `rawvideoparse framerate=<fps>/1`, so asking for 30 would tell GStreamer
# that frames arrive twice as fast as they do - every timestamp in the
# stream wrong by a factor of two. Change one of the two and change this.
ROVER_VIDEO_FPS = 15


class MainWindow(QMainWindow):
    SEMI_AUTO_REFUSAL = SEMI_AUTO_REFUSAL

    def __init__(self, ros_client_factory=RosBridgeClient, initial_host: str | None = None,
                 initial_port: int = 9090, node_poll_interval_ms: int = 2000,
                 staleness_check_interval_ms: int = 500, stale_after_seconds: float = 1.0,
                 gamepad_reader=None, gamepad_poll_interval_ms: int = 50,
                 video_receiver=None, video_port: int = 5600):
        super().__init__()
        self.stale_after_seconds = stale_after_seconds
        self.video_port = video_port
        self.setWindowTitle("Asterope Ground Station")
        self.ros_client_factory = ros_client_factory
        self.ros_client: RosBridgeClient | None = None
        self.drive_state = DriveCommandTracker()
        self.node_registry = NodeRegistry()
        self.gamepad_reader = gamepad_reader if gamepad_reader is not None else GamepadReader()
        self._gamepad_was_connected = False
        # Which source the panel is showing. Several decisions depend on it:
        # the video toggle must act on the source actually on screen, and a
        # rosbridge drop must not tear down a view that does not come from
        # rosbridge. Kept here rather than read back off the radio buttons
        # so the window does not depend on the dashboard's widget layout.
        self._mode = "manual"
        self._rover_video_before_simulation = False
        # The last /localization/status seen, or None if none has arrived.
        # Kept here as well as on the panel so entering semi-autonomous can
        # show the current state immediately instead of a blank marker until
        # the next 2 Hz status message.
        self._localization_status: dict | None = None
        # The last /localization/pose, kept so the header chip can show
        # tracking state and pose together whichever of the two arrives.
        self._localization_pose: dict | None = None
        # monotonic() when that status arrived, or None if none has.
        self._localization_status_at: float | None = None
        # monotonic() when the last /localization/map_status arrived, or
        # None if none has (or it has gone stale) - mirrors
        # _localization_status_at for the same reason.
        self._map_status_at: float | None = None
        # Same, for /drive_status.
        self._drive_status_at: float | None = None
        # The last /mode_status seen, or None if none has. Deliberately
        # never expired and never cleared on a rosbridge drop: the mode is
        # the rover's, not this link's, and forgetting it would reopen the
        # /manual_twist stream into an autonomous run the moment the link
        # hiccuped - which rule 1 would then read as a takeover.
        self._mode_state = None
        # monotonic() when the last /nav_status arrived, or None if none has
        # (or it has gone stale) - mirrors _localization_status_at for the
        # same reason.
        self._nav_status_at: float | None = None
        # The run id the rover last reported on /nav_status, or the one we
        # minted for our own last Go - whichever is freshest. pause/resume/
        # abort act on this, not on a value the ground station invented
        # unilaterally, so a reconnect mid-run still targets the run that is
        # actually happening.
        self._nav_run_id: str | None = None
        # The last /nav_status state seen, so a run's END is printed to the
        # terminal exactly once with its reason - the status republishes at
        # 2 Hz, so printing on every message would scroll the reason away.
        self._last_nav_state: str | None = None

        # The locked site->map transform, or None - the only two things
        # _on_go_requested (§3.8) and the mid-run refusal (§3.10) need.
        # Kept separately from site_card.transform/locked: the card is the
        # operator's view of its own state, this is the window's copy of
        # what is actually in effect for the wire.
        self._site_transform = None
        self._site_locked = False
        # The last /localization/pose seen at the moment lock_changed
        # delivered a transform (review round 3, §3.10) - what
        # reexpress_at_lock_pose re-expresses the transform against when
        # the SITE card says the camera was restarted with the rover
        # unmoved since Lock.
        self._site_lock_pose = None
        # The last pixel the operator clicked in the camera view, as
        # (u, v, width, height) in SOURCE-frame pixels - what a probe
        # request is built from. None until the panel's own `clicked`
        # signal has fired at least once.
        self._last_video_click = None
        # A per-probe counter so two requests issued in the same
        # monotonic tick still mint distinct ids (see new_probe_id).
        self._probe_counter = 0

        # Global button style, applied at the MainWindow level so every
        # QPushButton in the app (including ones on child widgets) gets the
        # same filled/hover/pressed/disabled look unless overridden locally.
        self.setStyleSheet(
            f"QMainWindow {{ background-color: {theme.BG}; }} " + theme.button_style()
        )

        input_style = (
            f"background-color: {theme.PANEL}; color: {theme.TEXT}; "
            f"border: 1px solid {theme.BORDER}; border-radius: 4px; padding: 4px 8px; "
            f"font-family: {theme.MONO_FONT_FAMILY};"
        )
        connect_button_style = (
            f"QPushButton {{ background-color: {theme.ACCENT}; color: #1c1200; "
            f"border: 1px solid {theme.ACCENT}; border-radius: 6px; padding: 6px 14px; "
            f"font-weight: 600; }} "
            f"QPushButton:hover {{ background-color: #f0985c; }} "
            f"QPushButton:pressed {{ background-color: #cf7333; }}"
        )
        # "LINK OK" / "LINK DOWN", not "ROSBRIDGE: CONNECTED": the long
        # form cost 200 px of a header that also carries rover mode,
        # localisation, the clock and STOP - and the header's fixed contents
        # were overflowing 1600 px, clipping the wordmark mid-word. What
        # the link IS lives in the tooltip; what matters at a glance is
        # whether it is up.
        self.connection_label = QLabel("LINK DOWN")
        self.connection_label.setToolTip(
            "The rosbridge websocket to the rover.")
        self.connection_label.setStyleSheet(self._header_pill_style(False))

        # The rover's OWN mode, in the header rather than buried in the DRIVE
        # row's chips: it is the answer to "what is the rover doing right
        # now", it is what gates Go, and it is emphatically not the VIEW
        # radios below (which only choose a picture). An operator who
        # confuses the two cannot start a run, so the authoritative one gets
        # the authoritative position.
        self.rover_mode_pill = QLabel("ROVER: NO STATUS")
        self.rover_mode_pill.setStyleSheet(theme.pill_style(theme.OFF, theme.TEXT))
        self.rover_mode_pill.setTextFormat(Qt.TextFormat.PlainText)
        self.rover_mode_pill.setToolTip(
            "The rover's actual mode, from /mode_status. Changed with the "
            "Manual and Autonomous buttons - never by the VIEW selector.")

        # Tracking health, not just coordinates: a SEARCHING ZED halts every
        # run (supervisor rule 3), and before this the operator had to ask
        # where that was visible. State first, coordinates after.
        self.localization_label = QLabel("LOC: NO POSE")
        self.localization_label.setStyleSheet(self._header_pill_style(False))
        self.localization_label.setTextFormat(Qt.TextFormat.PlainText)
        self.localization_label.setToolTip(
            "ZED tracking state and the rover's pose. SEARCHING for longer "
            "than the supervisor's grace halts an autonomous run.")

        # The one control that must be in the same place in every view, at a
        # size nobody has to aim for. It was a same-sized button among seven
        # on the DRIVE row; here nothing competes with it.
        self.stop_button = QPushButton("STOP")
        self.stop_button.setStyleSheet(theme.stop_button_style())
        self.stop_button.setMinimumHeight(40)
        self.stop_button.setMinimumWidth(120)
        self.stop_button.setToolTip(
            "Stop all movement immediately (emergency stop).  [Esc]\n"
            "Latches: the rover stays stopped until you move a stick again.")
        self.stop_button.clicked.connect(self._on_stop_requested)
        # Esc stops the rover from anywhere in the window, without finding
        # the button first. A stop pressed by accident costs a re-arm; a
        # stop the operator could not reach costs the rover.
        self._stop_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Escape), self)
        self._stop_shortcut.activated.connect(self.stop_button.animateClick)

        self.host_input = QLineEdit(initial_host or "")
        self.host_input.setPlaceholderText("rosbridge host")
        self.host_input.setStyleSheet(input_style)
        # Fixed, not elastic: the header carries rover state now, and an
        # elastic host field was the first thing to eat the room - it
        # collapsed to "host" while the rest of the row stayed comfortable.
        self.host_input.setFixedWidth(150)
        self.host_input.setToolTip("rosbridge host, e.g. 192.168.178.33")
        self.port_input = QLineEdit(str(initial_port))
        self.port_input.setFixedWidth(56)
        self.port_input.setStyleSheet(input_style)
        self.connect_button = QPushButton("Connect")
        self.connect_button.setStyleSheet(connect_button_style)
        self.connect_button.clicked.connect(self._on_connect_clicked)

        # The team mark, then the product name over the owning org - a
        # two-line wordmark reads as a masthead instead of one long shouty
        # string, and leaves the horizontal room for rover state.
        self.logo = StarLogo(size=32)
        title_label = QLabel("ASTEROPE GROUND STATION")
        title_label.setStyleSheet(
            f"color: {theme.TEXT}; font-weight: 700; "
            f"font-size: {theme.FONT_SIZE_TITLE}px; letter-spacing: 0.5px;")
        org_label = QLabel("STAR Dresden e.V.")
        org_label.setStyleSheet(
            f"color: {theme.TEXT_DIM}; font-size: {theme.FONT_SIZE_SMALL}px; "
            f"letter-spacing: 1px;")
        wordmark = QVBoxLayout()
        wordmark.setSpacing(0)
        wordmark.setContentsMargins(0, 0, 0, 0)
        wordmark.addWidget(title_label)
        wordmark.addWidget(org_label)

        self.mission_timer = MissionTimer()

        # The node list is a drawer, not a permanent column: it is a
        # diagnostic read a few times a session, and as a fixed 240 px
        # column it took width from the map and the camera in every view.
        # Host/port/Connect are touched once a session, and they were
        # holding ~290 px of a header that has rover state to show. They
        # collapse behind this, and manage themselves: open while the link
        # is down (which is exactly when you need them), closed once it is
        # up. The button stays, so a reconnect elsewhere is one click away.
        self.link_button = QPushButton("Link ▸")
        self.link_button.setCheckable(True)
        self.link_button.setChecked(True)
        self.link_button.setToolTip("Show or hide the rosbridge host and port.")
        self.link_button.toggled.connect(self._on_link_toggled)

        self.site_button = QPushButton("Site ▸")
        self.site_button.setCheckable(True)
        self.site_button.setToolTip(
            "Show or hide the site anchor: landmarks, the fit, and the lock.")
        self.site_button.toggled.connect(self._on_site_toggled)

        self.nodes_button = QPushButton("Nodes ▸")
        self.nodes_button.setCheckable(True)
        self.nodes_button.setToolTip("Show or hide the system-node list.")
        self.nodes_button.toggled.connect(self._on_nodes_toggled)

        # Visible only once a transform is locked, so a locked site frame
        # is legible without opening the drawer - text set by
        # _refresh_site_pill, empty (and neutrally styled) until then.
        self.site_status_pill = QLabel("")
        self.site_status_pill.setStyleSheet(self._header_pill_style(False))
        self.site_status_pill.setTextFormat(Qt.TextFormat.PlainText)
        self.site_status_pill.setToolTip(
            "The locked site->map transform, if any, and its RMS residual.")

        # Header reading order, left to right: who am I, what is the rover
        # doing, can it see where it is | how do I reach it | STOP.
        # Rover state sits left (read constantly), link plumbing right
        # (touched once per session), STOP hard right on its own.
        header = QWidget()
        header.setStyleSheet(f"background-color: {theme.BG};")
        header_layout = QHBoxLayout(header)
        header_layout.setSpacing(8)
        header_layout.addWidget(self.logo)
        header_layout.addLayout(wordmark)
        header_layout.addSpacing(14)
        header_layout.addWidget(self.rover_mode_pill)
        header_layout.addWidget(self.localization_label)
        header_layout.addWidget(self.site_status_pill)
        header_layout.addStretch()
        header_layout.addWidget(self.mission_timer)
        header_layout.addSpacing(14)
        header_layout.addWidget(self.connection_label)
        self.link_panel = QWidget()
        link_layout = QHBoxLayout(self.link_panel)
        link_layout.setContentsMargins(0, 0, 0, 0)
        link_layout.setSpacing(6)
        link_layout.addWidget(self.host_input)
        link_layout.addWidget(self.port_input)
        link_layout.addWidget(self.connect_button)
        header_layout.addWidget(self.link_panel)
        header_layout.addWidget(self.link_button)
        header_layout.addWidget(self.site_button)
        header_layout.addWidget(self.nodes_button)
        header_layout.addSpacing(16)
        header_layout.addWidget(self.stop_button)

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

        row = self.dashboard_page.map_row
        row.save_requested.connect(lambda name: self._send_map_command("save", name))
        row.load_requested.connect(lambda name: self._send_map_command("load", name))
        row.clear_requested.connect(lambda: self._send_map_command("clear"))

        drive_row = self.dashboard_page.drive_row
        drive_row.manual_requested.connect(self._on_manual_requested)
        drive_row.init_requested.connect(lambda: self._send_drive_command("init"))
        drive_row.reset_encoders_requested.connect(
            lambda: self._send_drive_command("reset_encoders"))
        drive_row.reset_odometry_requested.connect(
            lambda: self._send_drive_command("reset_odometry"))
        drive_row.drive_mode_requested.connect(lambda: self._send_drive_command("drive_mode"))
        drive_row.drive_state_requested.connect(lambda: self._send_drive_command("drive_state"))

        nav_row = self.dashboard_page.nav_row
        nav_row.autonomous_requested.connect(self._on_autonomous_requested)
        nav_row.go_requested.connect(self._on_go_requested)
        nav_row.pause_requested.connect(lambda: self._send_nav_request("pause"))
        nav_row.resume_requested.connect(lambda: self._send_nav_request("resume"))
        nav_row.abort_requested.connect(lambda: self._send_nav_request("abort"))

        site_card = self.dashboard_page.site_card
        site_card.table_load_requested.connect(self._on_site_table_load_requested)
        site_card.probe_requested.connect(self._on_probe_requested)
        site_card.lock_changed.connect(self._on_site_lock_changed)
        site_card.camera_restarted.connect(self._on_camera_restarted)

        # The operator's last click in the camera view - not gated on a
        # rosbridge connection, since it is purely local until a probe is
        # actually sent.
        self.dashboard_page.video_panel.image_label.clicked.connect(
            self._on_video_clicked)

        # The gamepad publishes /manual_twist in every mode (see
        # _poll_gamepad), so the STOP button and the deadman/lease status
        # line must be visible in every mode too - never mode-gated like the
        # map row, which only makes sense in semi_auto.
        self.dashboard_page.drive_row.setVisible(True)

        self._node_poll_timer = QTimer(self)
        self._node_poll_timer.timeout.connect(self._poll_nodes)
        self._node_poll_timer.start(node_poll_interval_ms)

        self._staleness_timer = QTimer(self)
        self._staleness_timer.timeout.connect(self._check_staleness)
        self._staleness_timer.start(staleness_check_interval_ms)

        self._gamepad_timer = QTimer(self)
        self._gamepad_timer.timeout.connect(self._poll_gamepad)
        self._gamepad_timer.start(gamepad_poll_interval_ms)

    @staticmethod
    def _header_pill_style(ok: bool) -> str:
        """The ROSBRIDGE/LOC header chips as pills: green fill when the
        thing they report is up/ok, the same neutral panel chip as before
        otherwise. Only the colour changes here - the text callers set is
        untouched, since tests assert on it."""
        if ok:
            return (
                f"color: #0c1a0e; background-color: {theme.OK}; "
                f"border: 1px solid {theme.OK}; border-radius: 10px; padding: 6px 12px; "
                f"font-family: {theme.MONO_FONT_FAMILY}; font-weight: 600;"
            )
        return (
            f"color: {theme.TEXT_DIM}; background-color: {theme.PANEL}; "
            f"border: 1px solid {theme.BORDER}; border-radius: 10px; padding: 6px 12px; "
            f"font-family: {theme.MONO_FONT_FAMILY};"
        )

    def _poll_nodes(self) -> None:
        if self.ros_client is not None:
            self.ros_client.poll_nodes()

    def _poll_gamepad(self) -> None:
        """Shows the gamepad's current stick-derived Twist on the Drive
        card/detail page unconditionally - this display never depends on a
        rosbridge connection, or on the mode.

        Publishing is gated. The continuous stream goes out only when a
        gamepad and a rosbridge connection are both present AND the rover's
        own /mode_status says manual or semi_auto (spec rule 5).

        In autonomous the stream stops but the operator is not locked out:
        a stick past the gamepad deadzone is still published, a centred one
        is not. That is what rule 5's own justification asks for - rule 1
        becomes "a real signal rather than a constant stream" - and it is
        what makes the supervisor's takeover reachable at all, since this
        is the only publisher of /manual_twist. No new threshold is
        invented: read_twist() has already applied gamepad_input.DEADZONE
        (0.1) per axis, so a non-zero component is past it by construction.

        On gamepad disconnect it publishes one zero-velocity Twist as a
        fail-safe stop - subject to the *stream* gate only, because a zero
        is never a takeover and a zero stream is exactly what the gate
        exists to silence - then stops publishing until the gamepad
        returns."""
        connected = self.gamepad_reader.poll()
        rosbridge_ready = self.ros_client is not None and self.ros_client.is_connected
        may_stream = rosbridge_ready and may_publish_manual_twist(self._mode_state)
        may_take_over = rosbridge_ready and may_publish_takeover_twist(self._mode_state)

        if connected:
            self._gamepad_was_connected = True
            linear_x, linear_y, angular_z = self.gamepad_reader.read_twist()
            self._update_drive_display(linear_x, linear_y, angular_z)
            if may_stream or (may_take_over and is_stick_deflected(
                    (linear_x, linear_y, angular_z))):
                self.ros_client.publish_manual_twist(linear_x, linear_y, angular_z)
        elif self._gamepad_was_connected:
            self._gamepad_was_connected = False
            self._update_drive_display(0.0, 0.0, 0.0)
            if may_stream:
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
        self.ros_client.signals.localization_status_received.connect(
            self._on_localization_status)
        self.ros_client.signals.localization_pose_received.connect(
            self._on_localization_pose)
        self.ros_client.signals.map_status_received.connect(self._on_map_status)
        self.ros_client.signals.drive_status_received.connect(self._on_drive_status)
        self.ros_client.signals.mode_status_received.connect(self._on_mode_status)
        self.ros_client.signals.nav_status_received.connect(self._on_nav_status)
        self.ros_client.signals.nav_path_summary_received.connect(
            self._on_nav_path_summary)
        self.ros_client.signals.probe_result_received.connect(self._on_probe_result)
        self.ros_client.signals.sightings_received.connect(self._on_sightings)

        try:
            # Subscribe BEFORE connecting: roslibpy's Ros.run() raises
            # RosTimeoutError after 10 s if rosbridge doesn't answer, but
            # it keeps reconnecting in the background and Topic.subscribe()
            # registers its "send once ready" callback against the Ros
            # instance regardless of whether a connection exists yet
            # (roslibpy.core.Topic._connect_topic -> Ros.send_on_ready ->
            # factory.on_ready, which queues via a one-shot "ready"
            # listener when not yet connected). So calling connect() last
            # means a slow/initially-down rosbridge still ends up with
            # every subscription wired up once it does go ready, instead
            # of silently skipping all of them.
            self.ros_client.subscribe_manual_twist()
            self.ros_client.subscribe_video_status()
            self.ros_client.subscribe_localization_status()
            self.ros_client.subscribe_localization_pose()
            self.ros_client.subscribe_map_status()
            self.ros_client.subscribe_drive_status()
            self.ros_client.subscribe_mode_status()
            self.ros_client.subscribe_nav_status()
            self.ros_client.subscribe_nav_path_summary()
            self.ros_client.subscribe_probe_result()
            self.ros_client.subscribe_landmark_sightings()
            self.ros_client.connect()
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
        text = "LINK OK" if connected else "LINK DOWN"
        # Out of the way once the link is up; back the moment it drops.
        self.link_button.setChecked(not connected)
        self.connection_label.setText(text)
        self.connection_label.setStyleSheet(self._header_pill_style(connected))
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
            self._on_map_status(None)
            self._on_drive_status(None)

    def closeEvent(self, event) -> None:
        """Stops the local video receiver on window close for the same
        reason as the rosbridge-disconnect path above - nothing else tears
        down the gst-launch-1.0 subprocess on quit."""
        self.dashboard_page.video_panel.stop_receiver(keep_failed_reason=True)
        super().closeEvent(event)

    def _check_staleness(self, now: float | None = None) -> None:
        now = monotonic() if now is None else now
        elapsed = self.drive_state.seconds_since_last(now)
        if elapsed is not None and elapsed > self.stale_after_seconds:
            self.dashboard_page.drive_card.mark_stale()
            self.drive_detail_page.mark_stale()

        # The rover unreachable is the case this catches: sim_bridge stops
        # receiving, the Gazebo rover holds still on its own, and this is
        # the panel saying so rather than leaving LOCALISED on screen
        # because that is what the rover said before the link died.
        if (self._localization_status_at is not None
                and now - self._localization_status_at
                > LOCALIZATION_STATUS_STALE_AFTER_SECONDS):
            self._localization_status = None
            self._localization_status_at = None
            # The pose goes with it: coordinates from a rover that has
            # stopped reporting its tracking health are not a position.
            self._localization_pose = None
            self._refresh_localization_label()
            self.dashboard_page.video_panel.set_localization_status(None)

        # Same staleness rule, for the map row: a rover that has gone quiet
        # must not leave stale map controls (load/save/clear) enabled.
        if (self._map_status_at is not None
                and now - self._map_status_at > LOCALIZATION_STATUS_STALE_AFTER_SECONDS):
            self._map_status_at = None
            self.dashboard_page.map_row.set_state(None)
        else:
            # The rover repeats last_command in every status message, so
            # only the clock can retire it: the row shows an outcome for
            # ten seconds after it changed and this is what ages it off
            # when the line is otherwise unchanged.
            self.dashboard_page.map_row.refresh(now)

        # Same staleness rule, for the drive row: a rover that has gone
        # quiet must not leave stale drive controls (STOP excepted, which
        # is always enabled) looking live.
        if (self._drive_status_at is not None
                and now - self._drive_status_at > LOCALIZATION_STATUS_STALE_AFTER_SECONDS):
            self._drive_status_at = None
            self.dashboard_page.drive_row.set_state(None)
        else:
            self.dashboard_page.drive_row.refresh(now)

        # Same staleness rule, for the NAV row: a rover that has gone quiet
        # must not leave a stale run status looking live.
        if (self._nav_status_at is not None
                and now - self._nav_status_at > LOCALIZATION_STATUS_STALE_AFTER_SECONDS):
            self._nav_status_at = None
            self.dashboard_page.nav_row.set_state(None)
        else:
            self.dashboard_page.nav_row.refresh(now)

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
            width=1344, height=376, fps=ROVER_VIDEO_FPS, bitrate_kbps=800)
        return True

    def _on_stream_requested(self, enable: bool) -> None:
        panel = self.dashboard_page.video_panel

        if self._mode == "semi_auto":
            # The whole point of this mode is that the field link carries no
            # video: the operator drives on the Gazebo view, placed by
            # localisation. Commanding the rover's camera would undo that;
            # toggling the local simulation receiver would blank the only
            # picture the operator has. So: nothing moves, and the panel
            # says why rather than swallowing the press.
            panel.refuse_stream(SEMI_AUTO_REFUSAL)
            return

        if self._mode == "simulation":
            # The toggle acts on whichever source the panel is actually
            # showing, and here that is the simulation - a local process
            # with no control plane at all, which streams whenever it runs
            # (by design: it is on this same laptop, so there is nothing to
            # ask). Commanding the rover's camera from here would push
            # 800 kbps of H.264 across the field link to port 5600 where
            # nothing is listening, for as long as the mode lasts.
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

    def _on_localization_status(self, status: dict) -> None:
        """Kept here as well as handed to the panel: entering semi-autonomous
        must show the current state at once rather than a blank marker until
        the next 2 Hz message."""
        self._localization_status = status
        self._localization_status_at = monotonic()
        self._refresh_localization_label()
        self.dashboard_page.video_panel.set_localization_status(status)

    def _on_localization_pose(self, pose: dict) -> None:
        """Three numbers in the header, at 5 Hz. This is the whole of what
        the ground station does with the pose - the rover in the Gazebo view
        is placed over DDS by sim_ik_node, not from here, because this
        process has no ROS and is not in that path."""
        self._localization_pose = pose
        self._refresh_localization_label()
        self.dashboard_page.nav_row.set_pose(pose)

    def _refresh_localization_label(self) -> None:
        """Tracking state first, pose second. A pose with no tracking behind
        it is a number that stopped being true a moment ago, so the state is
        what the chip is coloured by - green only for OK."""
        status = self._localization_status
        pose = self._localization_pose
        state = None
        if isinstance(status, dict):
            value = status.get("state")
            state = str(value) if value is not None else None

        if pose is None:
            text = f"LOC: {state}" if state else "LOC: NO POSE"
        else:
            # Compact on purpose: this chip shares the header with the
            # rover mode, the clock and the link controls, and the word
            # "yaw" costs more room than the number it introduces.
            coords = (f"x {pose['x']:.2f}  y {pose['y']:.2f}  "
                      f"{math.degrees(pose['yaw']):.0f}°")
            text = f"LOC: {state}  {coords}" if state else f"LOC: {coords}"
        self._set_elided(self.localization_label, text, 260)

        if state == "OK":
            self.localization_label.setStyleSheet(theme.pill_style(theme.OK, "#0c1a0e"))
        elif state in ("SEARCHING", "OFF"):
            # The state that halts autonomous runs must not read as healthy
            # just because coordinates are still arriving.
            self.localization_label.setStyleSheet(theme.pill_style(theme.BAD, "white"))
        else:
            self.localization_label.setStyleSheet(
                self._header_pill_style(pose is not None))

    def _send_map_command(self, action: str, name: str | None = None) -> None:
        if self.ros_client is None:
            return
        self.ros_client.send_map_command(action, name)

    def _on_map_status(self, state) -> None:
        self._map_status_at = monotonic() if state is not None else None
        self.dashboard_page.map_row.set_state(state)

    def _send_drive_command(self, action: str) -> None:
        if self.ros_client is None:
            return
        self.ros_client.send_drive_command(action)

    def _on_drive_status(self, state) -> None:
        self._drive_status_at = monotonic() if state is not None else None
        self.dashboard_page.drive_row.set_state(state)

    def _on_mode_status(self, state) -> None:
        # A status that would not parse says nothing about the rover's
        # mode. Keeping the last known one is the safe direction:
        # forgetting it would reopen the /manual_twist stream into a
        # running autonomous mission for the same reason a rosbridge blip
        # must not (see __init__).
        if state is None:
            return
        self._mode_state = state
        self._refresh_rover_mode_pill(state)
        self.dashboard_page.drive_row.set_mode_state(state)
        self.dashboard_page.nav_row.set_mode_state(state)

    @staticmethod
    def _set_elided(label, text: str, max_px: int) -> None:
        """Header chips carry rover text of unknown length (a mode reason, a
        pose). Left to grow they squeezed the wordmark and each other until
        Qt clipped them mid-word - "ASTEROPE GROUND S", "ROSBRIDGE: CONNECTEI".
        Bounded and elided instead, with the full text in the tooltip."""
        label.setMaximumWidth(max_px)
        metrics = label.fontMetrics()
        label.setText(metrics.elidedText(str(text), Qt.ElideRight, max_px - 20))
        label.setToolTip(str(text))

    def _on_link_toggled(self, shown: bool) -> None:
        self.link_panel.setVisible(shown)
        self.link_button.setText("Link ▾" if shown else "Link ▸")

    def _on_nodes_toggled(self, shown: bool) -> None:
        self.dashboard_page.node_list.setVisible(shown)
        self.nodes_button.setText("Nodes ▾" if shown else "Nodes ▸")

    def _on_site_toggled(self, shown: bool) -> None:
        self.dashboard_page.site_card.setVisible(shown)
        self.site_button.setText("Site ▾" if shown else "Site ▸")

    def _refresh_rover_mode_pill(self, state) -> None:
        """The header's rover-mode chip. The reason rides along with the
        mode: "MANUAL (localisation SEARCHING)" is the difference between
        an operator who knows why their run stopped and one who does not."""
        if state is None:
            self.rover_mode_pill.setText("ROVER: NO STATUS")
            self.rover_mode_pill.setStyleSheet(theme.pill_style(theme.OFF, theme.TEXT))
            return
        if state.estop_latched or state.mode == "estop":
            text, bg, fg = "E-STOP LATCHED", theme.BAD, "white"
        elif state.mode == "autonomous":
            text, bg, fg = "AUTONOMOUS", theme.ACCENT, "#2a1600"
        elif state.mode in ("manual", "semi_auto"):
            text, bg, fg = state.mode.upper(), theme.OK, "#0c1a0e"
        else:
            text, bg, fg = f"MODE {state.mode}", theme.OFF, theme.TEXT
        reason = getattr(state, "reason", "") or ""
        # Only reasons that explain a state the operator did not choose;
        # "mode request" is just an echo of their own last press.
        if reason and reason not in ("mode request", "startup"):
            text = f"{text} - {reason}"
        self._set_elided(self.rover_mode_pill, text, 300)
        self.rover_mode_pill.setStyleSheet(theme.pill_style(bg, fg))

    def _on_autonomous_requested(self) -> None:
        """The NAV row's Autonomous button. Only a /mode_request: the
        supervisor is the single authority on mode, and goal_relay
        refuses Go until it says autonomous."""
        if self.ros_client is None:
            return
        self.ros_client.send_mode_request("autonomous")

    def _on_go_requested(self, waypoints) -> None:
        """§3.8: the one place in the entire codebase where a site
        coordinate becomes a map coordinate. With no locked transform this
        is byte-for-byte today's behaviour - there is a regression test
        that says so."""
        if self.ros_client is None:
            return
        if self._site_transform is not None and self._site_locked:
            t = self._site_transform
            waypoints = [Waypoint(*site_to_map(t, w.x, w.y),
                                  None if w.yaw is None else site_yaw_to_map_yaw(t, w.yaw))
                        for w in waypoints]
        self._nav_run_id = new_run_id(time())
        self.ros_client.send_nav_request("go", waypoints, self._nav_run_id)

    def _send_nav_request(self, action: str) -> None:
        """pause / resume / abort, always against the run the rover
        reported. A ground station that reconnected mid-run adopts the
        rover's run id from /nav_status rather than its own, so the
        buttons act on the run that is actually happening."""
        if self.ros_client is None or self._nav_run_id is None:
            return
        self.ros_client.send_nav_request(action, run_id=self._nav_run_id)

    def _on_nav_status(self, state) -> None:
        if state is None:
            return
        self._nav_status_at = monotonic()
        if state.run_id:
            self._nav_run_id = state.run_id
        if (state.state != self._last_nav_state
                and self._last_nav_state in NAV_ACTIVE_STATES
                and state.state not in NAV_ACTIVE_STATES):
            reason = state.error or "destination reached"
            print(f"ground_station: autonomy run {state.run_id or '?'} "
                  f"ended: {state.state} - {reason}", file=sys.stderr)
        self._last_nav_state = state.state
        self.dashboard_page.nav_row.set_state(state)

    def _on_nav_path_summary(self, summary) -> None:
        if summary is None:
            return
        self.dashboard_page.nav_row.set_path_summary(summary)

    # --- site anchor (site-anchor plan, Task 9) ---------------------------

    def _on_site_table_load_requested(self, path: str) -> None:
        """The SITE card's own file dialog only picks a path - it never
        touches disk itself (D1: the loader is a stdlib-only sibling of
        this window, not the card's job). A malformed table is reported to
        the terminal and otherwise ignored: the card keeps whatever it had
        before."""
        try:
            table = load_landmark_table(path)
        except LandmarkTableError as exc:
            print(f"ground_station: failed to load site table: {exc}", file=sys.stderr)
            return
        self.dashboard_page.site_card.set_table(table)

    def _on_video_clicked(self, u: int, v: int, width: int, height: int) -> None:
        """The camera panel's own click, in source-frame pixels
        (video_panel.AspectLabel.clicked). Remembered, not sent - a probe
        is only ever built at the moment the operator presses Probe."""
        self._last_video_click = (u, v, width, height)

    def _on_probe_requested(self, landmark_id: str, target: str) -> None:
        """site_card.probe_requested: the operator picked a landmark and
        pressed Probe. The card owns which landmark and which target; this
        window owns the pixel, because that is where the camera view - and
        its last click - actually lives."""
        if self.ros_client is None:
            return
        if self._last_video_click is None:
            self.dashboard_page.site_card.state_pill.setText(
                "click the landmark in the camera view first")
            return
        u, v, width, height = self._last_video_click
        self._probe_counter += 1
        request_id = new_probe_id(monotonic(), self._probe_counter)
        self.ros_client.send_probe_request(request_id, landmark_id, u, v, width, height,
                                           target=target)

    def _on_probe_result(self, result) -> None:
        if result is None:
            return
        self.dashboard_page.site_card.apply_probe_result(result)

    def _on_sightings(self, report) -> None:
        if report is None:
            return
        self.dashboard_page.site_card.apply_sightings(report)

    def _on_site_lock_changed(self, transform) -> None:
        """site_card.lock_changed: a solved transform to lock, or None to
        clear a lock. D2 forbids changing the transform under an active
        run - a converted goal would move a rover already driving toward
        it - so a lock/unlock while a run is active is refused: the card's
        own button and state are put back where they were and nothing here
        changes. Otherwise, this window's copy of the transform (what
        _on_go_requested actually uses) is updated, the row's drawing
        follows (NavRow.set_site_transform), the header pill reflects it,
        and - review round 3, §3.10 - the pose in effect at this exact
        moment is captured as the lock pose."""
        site_card = self.dashboard_page.site_card
        if self._last_nav_state in NAV_ACTIVE_STATES:
            print("ground_station: refusing to change the site transform "
                  "mid-run", file=sys.stderr)
            site_card.locked = self._site_locked
            site_card.lock_button.blockSignals(True)
            site_card.lock_button.setChecked(self._site_locked)
            site_card.lock_button.blockSignals(False)
            return

        self._site_transform = transform
        self._site_locked = transform is not None
        self._site_lock_pose = self._localization_pose if transform is not None else None
        self.dashboard_page.nav_row.set_site_transform(transform)
        self._refresh_site_pill()

    def _on_camera_restarted(self) -> None:
        """site_card.camera_restarted (§3.10, review round 3): the ZED
        wrapper was just relaunched with the rover NOT moved since Lock.
        The wrapper keeps no area memory, so the restart bears a brand new
        map frame at the rover's pose at that moment - which, with the
        rover unmoved, IS the lock pose captured in _on_site_lock_changed.
        T1 owns the arithmetic (reexpress_at_lock_pose); this is the swap,
        reflected back to the card exactly as the plan asks: transform
        updated, state pill reads LOCKED (re-expressed)."""
        if (not self._site_locked or self._site_transform is None
                or self._site_lock_pose is None):
            return
        pose = self._site_lock_pose
        new_transform = reexpress_at_lock_pose(
            self._site_transform, pose["x"], pose["y"], pose["yaw"])
        self._site_transform = new_transform

        site_card = self.dashboard_page.site_card
        site_card.transform = new_transform
        site_card.state_pill.setText("LOCKED (re-expressed)")
        site_card.state_pill.setStyleSheet(theme.pill_style(theme.OK, theme.BG))

        self.dashboard_page.nav_row.set_site_transform(new_transform)
        self._refresh_site_pill()

    def _refresh_site_pill(self) -> None:
        """The header's SITE chip: a locked transform is visible without
        opening the drawer, and it is the only thing this pill ever shows -
        an unlocked/no-table state says nothing here, because the SITE
        drawer's own state_pill already covers that in detail."""
        if self._site_locked and self._site_transform is not None:
            text = f"SITE: LOCKED  RMS {self._site_transform.rms_m:.2f} m"
            self._set_elided(self.site_status_pill, text, 220)
            self.site_status_pill.setStyleSheet(self._header_pill_style(True))
        else:
            self.site_status_pill.setText("")
            self.site_status_pill.setToolTip(
                "The locked site->map transform, if any, and its RMS residual.")
            self.site_status_pill.setStyleSheet(self._header_pill_style(False))

    def _on_stop_requested(self) -> None:
        """STOP does two different things, in this order.

        /estop_request latches on the Orin and survives this link dying -
        it is the one that must go first, because it is the one that keeps
        working when the next message does not arrive. The chassis stop on
        /drive_command is what puts F2 on the wire through the primary
        right now. Neither replaces the other, and both are sent in every
        mode."""
        if self.ros_client is None:
            return
        self.ros_client.send_estop_request("ground station STOP")
        self.ros_client.send_drive_command("stop")

    def _on_manual_requested(self) -> None:
        """The DRIVE row's Manual button, which is also the only way back
        from a latched e-stop: /mode_request manual clears the supervisor's
        latch, then the coordinator is asked for Manual as before. In that
        order - asking the coordinator to arm while the supervisor is still
        latched would arm a rover that is still being held at zero."""
        if self.ros_client is None:
            return
        self.ros_client.send_mode_request("manual")
        self.ros_client.send_drive_command("manual")

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
        previous_mode = self._mode
        self._mode = mode
        self.dashboard_page.map_row.setVisible(mode == "semi_auto")
        self.dashboard_page.nav_row.setVisible(mode == "autonomous")
        # The waypoint editor lives in the bottom card (see DashboardPage),
        # so it is no longer hidden by the nav row's own visibility and has
        # to be switched with it.
        self.dashboard_page.waypoint_panel.setVisible(mode == "autonomous")
        # The drive row is not mode-gated - see the setVisible(True) call in
        # __init__: the gamepad drives in every mode, so STOP and the
        # deadman/lease line must stay visible in every mode too.
        if previous_mode not in ("simulation", "semi_auto") and mode in ("simulation", "semi_auto"):
            # What the operator had before the simulation modes forced the
            # local receiver on; restored on the way back, so a mode
            # round-trip never switches the rover's camera on by itself.
            self._rover_video_before_simulation = panel.streaming

        if mode == "autonomous":
            # Autonomous is a semi-autonomous view with a NAV row on top: the
            # operator watches the Gazebo mirror with the plan drawn in it -
            # the whole point of the map view - while the rover's own mode
            # changes only when Autonomous is pressed on that row, not by
            # switching this radio.
            self._request_rover_video(False)
            panel.set_source("simulation", SIM_VIDEO_PORT,
                             dead_reckoning=False,
                             reports_remote_status=False,
                             show_localization=True)
            panel.set_localization_status(self._localization_status)
            if not panel.streaming:
                panel.set_streaming(True)
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
                # The Gazebo stream is local and has no control plane, and
                # the toggle is refused in this mode - so the receiver
                # cannot be started by hand here. Start it now, whatever
                # the toggle was before the switch; otherwise entering with
                # video off leaves the panel deaf on 5601 with no way in.
                if not panel.streaming:
                    panel.set_streaming(True)
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
        #
        # And only if it was on before the simulation mode: semi mode forces
        # the local receiver on (the toggle is refused there), and set_source
        # carries that "on" back - which is not the operator's choice.
        if not self._rover_video_before_simulation:
            if panel.streaming:
                panel.set_streaming(False)
            return
        if panel.streaming:
            self._request_rover_video(True)
