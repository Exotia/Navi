from ground_station.ros_client import RosBridgeClient
from ground_station.ui.main_window import MainWindow


class FakeTopic:
    def __init__(self, ros, name, msg_type):
        self.callback = None

    def subscribe(self, callback):
        self.callback = callback


class FakeRos:
    def __init__(self, host, port):
        self.is_connected = False
        self.ready_callback = None

    def on_ready(self, callback):
        self.ready_callback = callback

    def run(self):
        self.is_connected = True
        self.ready_callback()

    def close(self):
        self.is_connected = False

    def get_nodes(self, callback, errback=None):
        pass


def make_window(qtbot):
    client = RosBridgeClient(host="localhost", ros_factory=FakeRos, topic_factory=FakeTopic)
    window = MainWindow(client)
    qtbot.addWidget(window)
    return window, client


def test_main_window_starts_on_dashboard_page(qtbot):
    window, _ = make_window(qtbot)
    assert window.stacked_widget.currentWidget() is window.dashboard_page


def test_clicking_drive_card_details_shows_detail_page(qtbot):
    window, _ = make_window(qtbot)
    window.dashboard_page.drive_card.details_requested.emit()

    assert window.stacked_widget.currentWidget() is window.drive_detail_page


def test_back_from_detail_returns_to_dashboard(qtbot):
    window, _ = make_window(qtbot)
    window.dashboard_page.drive_card.details_requested.emit()
    window.drive_detail_page.back_requested.emit()

    assert window.stacked_widget.currentWidget() is window.dashboard_page


def test_twist_message_updates_drive_card(qtbot):
    window, client = make_window(qtbot)
    client.connect()
    client.subscribe_cmd_vel()

    # calling the slot directly here: it's a plain method, not a signal to
    # wait on — the signal->slot wiring itself is covered by
    # test_ros_client.py's test_twist_message_emits_twist_received_signal
    msg = {"linear": {"x": 0.4, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.1}}
    window._on_twist(msg)

    assert "0.40" in window.dashboard_page.drive_card.vx_label.text()


def test_connection_changed_updates_label(qtbot):
    window, client = make_window(qtbot)
    client.connect()

    assert "CONNECTED" in window.connection_label.text()
