from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QStackedWidget

from ground_station import theme
from ground_station.models import DriveState, NodeRegistry
from ground_station.ros_client import RosBridgeClient
from ground_station.ui.dashboard_page import DashboardPage
from ground_station.ui.drive_detail_page import DriveDetailPage


class MainWindow(QMainWindow):
    def __init__(self, ros_client: RosBridgeClient, node_poll_interval_ms: int = 2000,
                 staleness_check_interval_ms: int = 500, stale_after_seconds: float = 1.0):
        super().__init__()
        self.stale_after_seconds = stale_after_seconds
        self.setWindowTitle("Asterope Ground Station")
        self.ros_client = ros_client
        self.drive_state = DriveState()
        self.node_registry = NodeRegistry()

        self.connection_label = QLabel("ROSBRIDGE: DISCONNECTED")
        self.connection_label.setStyleSheet(f"color: {theme.TEXT_DIM};")

        header = QWidget()
        header_layout = QHBoxLayout(header)
        header_layout.addWidget(QLabel("ASTEROPE GROUND STATION"))
        header_layout.addStretch()
        header_layout.addWidget(self.connection_label)

        self.dashboard_page = DashboardPage()
        self.drive_detail_page = DriveDetailPage()
        self.stacked_widget = QStackedWidget()
        self.stacked_widget.addWidget(self.dashboard_page)
        self.stacked_widget.addWidget(self.drive_detail_page)

        central = QWidget()
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

        self.ros_client.signals.twist_received.connect(self._on_twist)
        self.ros_client.signals.nodes_received.connect(self._on_nodes)
        self.ros_client.signals.connection_changed.connect(self._on_connection_changed)

        self._node_poll_timer = QTimer(self)
        self._node_poll_timer.timeout.connect(self.ros_client.poll_nodes)
        self._node_poll_timer.start(node_poll_interval_ms)

        self._staleness_timer = QTimer(self)
        self._staleness_timer.timeout.connect(self._check_staleness)
        self._staleness_timer.start(staleness_check_interval_ms)

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
        self.connection_label.setStyleSheet(f"color: {color};")

    def _check_staleness(self) -> None:
        elapsed = self.drive_state.seconds_since_last()
        if elapsed is not None and elapsed > self.stale_after_seconds:
            self.dashboard_page.drive_card.mark_stale()
            self.drive_detail_page.mark_stale()
