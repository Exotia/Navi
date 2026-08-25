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


class MainWindow(QMainWindow):
    def __init__(self, ros_client_factory=RosBridgeClient, initial_host: str | None = None,
                 initial_port: int = 9090, node_poll_interval_ms: int = 2000,
                 staleness_check_interval_ms: int = 500, stale_after_seconds: float = 1.0,
                 gamepad_reader=None, gamepad_poll_interval_ms: int = 50):
        super().__init__()
        self.stale_after_seconds = stale_after_seconds
        self.setWindowTitle("Asterope Ground Station")
        self.setStyleSheet(f"QMainWindow {{ background-color: {theme.BG}; }}")
        self.ros_client_factory = ros_client_factory
        self.ros_client: RosBridgeClient | None = None
        self.drive_state = DriveState()
        self.node_registry = NodeRegistry()
        self.gamepad_reader = gamepad_reader if gamepad_reader is not None else GamepadReader()
        self._gamepad_was_connected = False

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

        self.dashboard_page = DashboardPage()
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
        """Publishes gamepad stick position as /cmd_vel automatically once
        both a gamepad and a rosbridge connection are present - no manual
        "enable driving" step. On disconnect, publishes one zero-velocity
        Twist as a fail-safe stop rather than leaving the rover at its last
        command forever, then stops publishing until the gamepad returns."""
        connected = self.gamepad_reader.poll()
        rosbridge_ready = self.ros_client is not None and self.ros_client.is_connected

        if connected:
            self._gamepad_was_connected = True
            if rosbridge_ready:
                linear_x, linear_y, angular_z = self.gamepad_reader.read_twist()
                self.ros_client.publish_cmd_vel(linear_x, linear_y, angular_z)
        elif self._gamepad_was_connected:
            self._gamepad_was_connected = False
            if rosbridge_ready:
                self.ros_client.publish_cmd_vel(0.0, 0.0, 0.0)

    def _on_connect_clicked(self) -> None:
        host = self.host_input.text().strip()
        if not host:
            return
        try:
            port = int(self.port_input.text().strip())
        except ValueError:
            port = 9090
        self._connect_to(host, port)

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

        try:
            self.ros_client.connect()
            self.ros_client.subscribe_cmd_vel()
        except Exception as exc:
            print(f"ground_station: failed to connect to rosbridge: {exc}", file=sys.stderr)

    def _on_twist(self, msg: dict) -> None:
        linear = msg.get("linear", {})
        angular = msg.get("angular", {})
        self.drive_state.ingest(linear.get("x", 0.0), linear.get("y", 0.0), angular.get("z", 0.0))
        self.dashboard_page.drive_card.update_from(self.drive_state)
        self.drive_detail_page.update_from(self.drive_state)
        self.drive_detail_page.append_raw_message(
            f"linear.x={linear.get('x', 0.0):.2f} linear.y={linear.get('y', 0.0):.2f} "
            f"angular.z={angular.get('z', 0.0):.2f}"
        )

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

    def _check_staleness(self) -> None:
        elapsed = self.drive_state.seconds_since_last()
        if elapsed is not None and elapsed > self.stale_after_seconds:
            self.dashboard_page.drive_card.mark_stale()
            self.drive_detail_page.mark_stale()
