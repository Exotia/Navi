from ground_station.ros_client import RosBridgeClient
from ground_station.ui.main_window import MainWindow


class FakeTopic:
    instances = []

    def __init__(self, ros, name, msg_type):
        self.callback = None
        self.published_messages = []
        FakeTopic.instances.append(self)

    def subscribe(self, callback):
        self.callback = callback

    def publish(self, message):
        self.published_messages.append(message)


class FakeRos:
    instances = []

    def __init__(self, host, port):
        self.is_connected = False
        self.ready_callback = None
        self._event_callbacks = {}
        FakeRos.instances.append(self)

    def on_ready(self, callback):
        self.ready_callback = callback

    def on(self, event_name, callback):
        self._event_callbacks[event_name] = callback

    def trigger_event(self, event_name, *args):
        callback = self._event_callbacks.get(event_name)
        if callback:
            callback(*args)

    def run(self):
        self.is_connected = True
        self.ready_callback()

    def close(self):
        self.is_connected = False

    def get_nodes(self, callback, errback=None):
        pass


class FakeGamepadReader:
    """Never reports a gamepad connected unless a test explicitly says so -
    this is the default injected into every MainWindow test so none of them
    trigger a real pygame.init() as a side effect of just building a window."""

    def __init__(self, connected: bool = False, twist: tuple = (0.0, 0.0, 0.0)):
        self._connected = connected
        self._twist = twist

    def poll(self) -> bool:
        return self._connected

    def read_twist(self) -> tuple:
        return self._twist

    def set_connected(self, connected: bool) -> None:
        self._connected = connected

    def set_twist(self, twist: tuple) -> None:
        self._twist = twist


def make_fake_client_factory():
    def factory(host, port=9090):
        return RosBridgeClient(host=host, port=port, ros_factory=FakeRos, topic_factory=FakeTopic)
    return factory


def make_window(qtbot, initial_host="localhost", gamepad_reader=None):
    FakeRos.instances.clear()
    FakeTopic.instances.clear()
    window = MainWindow(
        ros_client_factory=make_fake_client_factory(),
        initial_host=initial_host,
        gamepad_reader=gamepad_reader if gamepad_reader is not None else FakeGamepadReader(),
    )
    qtbot.addWidget(window)
    if initial_host:
        # MainWindow itself never auto-connects from its constructor (see
        # test_initial_host_prefills_but_does_not_auto_connect below) - only
        # ground_station.main defers this call via QTimer.singleShot after
        # the event loop starts. Tests call it directly and synchronously.
        window._connect_to(initial_host, 9090)
    return window, window.ros_client


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
    window, _ = make_window(qtbot, initial_host=None)

    assert window.connection_label.text() == "ROSBRIDGE: DISCONNECTED"

    window._connect_to("localhost", 9090)

    assert window.connection_label.text() == "ROSBRIDGE: CONNECTED"


def test_mid_session_disconnect_updates_label_back_to_disconnected(qtbot):
    window, client = make_window(qtbot)
    client.connect()
    assert window.connection_label.text() == "ROSBRIDGE: CONNECTED"

    ros = FakeRos.instances[-1]
    ros.trigger_event("close", None)

    assert window.connection_label.text() == "ROSBRIDGE: DISCONNECTED"


def test_check_staleness_marks_drive_displays_stale_after_threshold(qtbot):
    window, _ = make_window(qtbot)
    window.stale_after_seconds = 1.0
    # ingest "now" far in the past (relative to real monotonic time) so
    # seconds_since_last() is certain to exceed the staleness threshold
    window.drive_state.ingest(0.4, 0.0, 0.1, now=0.0)
    window.dashboard_page.drive_card.update_from(window.drive_state)
    window.drive_detail_page.update_from(window.drive_state)

    window._check_staleness()

    assert "0 Hz" in window.dashboard_page.drive_card.rate_label.text()
    assert "no data" in window.dashboard_page.drive_card.rate_label.text()
    assert "no data" in window.drive_detail_page.link_label.text()


def test_check_staleness_does_not_mark_fresh_data_stale(qtbot):
    window, _ = make_window(qtbot)
    window.stale_after_seconds = 1.0
    # ingest with the real current time (now=None default) - this sample is
    # fresh, so staleness must not fire
    window.drive_state.ingest(0.4, 0.0, 0.1)
    window.dashboard_page.drive_card.update_from(window.drive_state)
    window.drive_detail_page.update_from(window.drive_state)

    window._check_staleness()

    assert "no data" not in window.dashboard_page.drive_card.rate_label.text()
    assert "no data" not in window.drive_detail_page.link_label.text()


def test_new_twist_after_stale_clears_stale_indication(qtbot):
    window, _ = make_window(qtbot)
    window.stale_after_seconds = 1.0
    window.drive_state.ingest(0.4, 0.0, 0.1, now=0.0)
    window._check_staleness()
    assert "no data" in window.dashboard_page.drive_card.rate_label.text()

    msg = {"linear": {"x": 0.4, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.1}}
    window._on_twist(msg)

    assert "no data" not in window.dashboard_page.drive_card.rate_label.text()
    assert "no data" not in window.drive_detail_page.link_label.text()


def test_nodes_received_updates_node_list(qtbot):
    window, _ = make_window(qtbot)

    # calling the slot directly here, same pattern as
    # test_twist_message_updates_drive_card: the signal->slot wiring itself
    # is covered by test_ros_client.py's
    # test_poll_nodes_emits_nodes_received_signal.
    window._on_nodes(["/cmd_vel_bridge", "/rosbridge_websocket"])

    assert window.dashboard_page.node_list.row_count() == 2
    row_texts = {window.dashboard_page.node_list.row_text(i) for i in range(2)}
    assert row_texts == {"/cmd_vel_bridge  (up)", "/rosbridge_websocket  (up)"}


def test_no_initial_host_leaves_ros_client_unset(qtbot):
    window, client = make_window(qtbot, initial_host=None)

    assert client is None
    assert window.ros_client is None
    assert window.host_input.text() == ""
    assert window.connection_label.text() == "ROSBRIDGE: DISCONNECTED"


def test_initial_host_prefills_but_does_not_auto_connect(qtbot):
    FakeRos.instances.clear()
    window = MainWindow(ros_client_factory=make_fake_client_factory(), initial_host="orin.local",
                         gamepad_reader=FakeGamepadReader())
    qtbot.addWidget(window)

    # MainWindow's constructor only pre-fills the field - it never connects
    # on its own. Only ground_station.main's deferred singleShot call (or a
    # user clicking Connect) actually triggers a connection attempt.
    assert window.host_input.text() == "orin.local"
    assert window.ros_client is None
    assert FakeRos.instances == []


def test_connect_button_with_typed_host_connects(qtbot):
    window, _ = make_window(qtbot, initial_host=None)

    window.host_input.setText("192.168.1.50")
    window.port_input.setText("9090")
    window._on_connect_clicked()

    assert window.ros_client is not None
    assert window.connection_label.text() == "ROSBRIDGE: CONNECTED"


def test_connect_button_with_blank_host_does_nothing(qtbot):
    window, _ = make_window(qtbot, initial_host=None)

    window.host_input.setText("   ")
    window._on_connect_clicked()

    assert window.ros_client is None


def test_reconnecting_to_a_new_host_closes_the_previous_client(qtbot):
    window, first_client = make_window(qtbot, initial_host="192.168.1.50")
    first_ros = FakeRos.instances[-1]
    assert first_ros.is_connected is True

    window.host_input.setText("192.168.1.60")
    window._on_connect_clicked()

    assert first_ros.is_connected is False
    assert window.ros_client is not first_client
    assert len(FakeRos.instances) == 2
    assert FakeRos.instances[-1].is_connected is True


def test_gamepad_publishes_cmd_vel_when_gamepad_and_rosbridge_both_present(qtbot):
    gamepad = FakeGamepadReader(connected=True, twist=(0.4, -0.05, 0.1))
    window, _ = make_window(qtbot, initial_host="localhost", gamepad_reader=gamepad)

    window._poll_gamepad()

    topic = FakeTopic.instances[-1]
    assert topic.published_messages == [{
        "linear": {"x": 0.4, "y": -0.05, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.1},
    }]


def test_gamepad_disconnected_does_not_publish(qtbot):
    gamepad = FakeGamepadReader(connected=False)
    window, _ = make_window(qtbot, initial_host="localhost", gamepad_reader=gamepad)

    window._poll_gamepad()

    assert FakeTopic.instances[-1].published_messages == []


def test_gamepad_connected_but_rosbridge_not_connected_does_not_publish(qtbot):
    gamepad = FakeGamepadReader(connected=True, twist=(0.4, 0.0, 0.0))
    window, client = make_window(qtbot, initial_host=None, gamepad_reader=gamepad)
    assert client is None

    # must not raise even though there's no ros_client at all yet
    window._poll_gamepad()

    assert FakeTopic.instances == []


def test_gamepad_disconnect_sends_one_zero_velocity_stop_then_stays_quiet(qtbot):
    gamepad = FakeGamepadReader(connected=True, twist=(0.4, 0.0, 0.0))
    window, _ = make_window(qtbot, initial_host="localhost", gamepad_reader=gamepad)
    topic = FakeTopic.instances[-1]

    window._poll_gamepad()
    assert len(topic.published_messages) == 1

    gamepad.set_connected(False)
    window._poll_gamepad()

    assert len(topic.published_messages) == 2
    assert topic.published_messages[-1] == {
        "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
    }

    # still disconnected on a later poll - must not publish again
    window._poll_gamepad()

    assert len(topic.published_messages) == 2
