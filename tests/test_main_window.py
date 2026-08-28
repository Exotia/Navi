import json

from ground_station.ros_client import RosBridgeClient
from ground_station.ui.main_window import MainWindow
from ground_station.video_receiver import VideoReceiver
from tests.test_video_panel import FakeReceiver


class FakeTopic:
    instances = []

    def __init__(self, ros, name, msg_type):
        self.name = name
        self.msg_type = msg_type
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


def make_window(qtbot, initial_host="localhost", gamepad_reader=None, video_receiver=None):
    FakeRos.instances.clear()
    FakeTopic.instances.clear()
    window = MainWindow(
        ros_client_factory=make_fake_client_factory(),
        initial_host=initial_host,
        gamepad_reader=gamepad_reader if gamepad_reader is not None else FakeGamepadReader(),
        # A fake receiver by default, same reasoning as the fake gamepad
        # above: no test should spawn a real gst-launch-1.0 subprocess just
        # from constructing a window.
        video_receiver=video_receiver if video_receiver is not None else FakeReceiver(),
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
    client.subscribe_manual_twist()

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


def test_gamepad_publishes_manual_twist_when_gamepad_and_rosbridge_both_present(qtbot):
    gamepad = FakeGamepadReader(connected=True, twist=(0.4, -0.05, 0.1))
    window, _ = make_window(qtbot, initial_host="localhost", gamepad_reader=gamepad)

    window._poll_gamepad()

    # subscribe_video_status() (added in _connect_to alongside
    # subscribe_manual_twist()) creates its own topic, so instances[-1] is
    # no longer reliably the manual_twist topic - select it by name instead.
    topic = next(t for t in FakeTopic.instances if t.name == "/manual_twist")
    assert topic.published_messages == [{
        "linear": {"x": 0.4, "y": -0.05, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.1},
    }]
    # the local display path is independent of the connection, but should
    # of course also reflect the same reading
    assert "0.40" in window.dashboard_page.drive_card.vx_label.text()


def test_gamepad_disconnected_does_not_publish(qtbot):
    gamepad = FakeGamepadReader(connected=False)
    window, _ = make_window(qtbot, initial_host="localhost", gamepad_reader=gamepad)

    window._poll_gamepad()

    manual_twist_topic = next(t for t in FakeTopic.instances if t.name == "/manual_twist")
    assert manual_twist_topic.published_messages == []


def test_gamepad_updates_local_display_even_without_a_rosbridge_connection(qtbot):
    # per the design: gamepad input + the Twist it produces should always
    # be visible in the GS, regardless of whether anything is connected -
    # publishing to the rover is a separate concern for a later module.
    gamepad = FakeGamepadReader(connected=True, twist=(0.4, 0.0, 0.2))
    window, client = make_window(qtbot, initial_host=None, gamepad_reader=gamepad)
    assert client is None

    window._poll_gamepad()

    assert "0.40" in window.dashboard_page.drive_card.vx_label.text()
    assert "0.20" in window.dashboard_page.drive_card.wz_label.text()
    # no rosbridge connection at all yet, so nothing to publish on
    assert FakeTopic.instances == []


def test_gamepad_disconnect_sends_one_zero_velocity_stop_then_stays_quiet(qtbot):
    gamepad = FakeGamepadReader(connected=True, twist=(0.4, 0.0, 0.0))
    window, _ = make_window(qtbot, initial_host="localhost", gamepad_reader=gamepad)
    topic = next(t for t in FakeTopic.instances if t.name == "/manual_twist")

    window._poll_gamepad()
    assert len(topic.published_messages) == 1

    gamepad.set_connected(False)
    window._poll_gamepad()

    assert len(topic.published_messages) == 2
    assert topic.published_messages[-1] == {
        "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
    }
    assert "0.00" in window.dashboard_page.drive_card.vx_label.text()

    # still disconnected on a later poll - must not publish (or re-display) again
    window._poll_gamepad()

    assert len(topic.published_messages) == 2


def test_disconnect_stops_the_local_video_receiver(qtbot):
    window, _ = make_window(qtbot, initial_host="192.168.178.33")
    window._on_stream_requested(True)
    assert window.dashboard_page.video_panel.receiver.started

    ros = FakeRos.instances[-1]
    ros.trigger_event("close", None)

    assert window.dashboard_page.video_panel.receiver.stopped


def test_disconnect_preserves_a_previously_reported_failure_reason(qtbot):
    window, _ = make_window(qtbot, initial_host="192.168.178.33")
    window.dashboard_page.video_panel.apply_status({"state": "failed", "detail": "camera busy"})

    ros = FakeRos.instances[-1]
    ros.trigger_event("close", None)

    text = window.dashboard_page.video_panel.status_label.text()
    assert "camera busy" in text
    assert "FAILED" in text.upper()


def test_closing_the_window_stops_the_local_video_receiver(qtbot):
    window, _ = make_window(qtbot, initial_host="192.168.178.33")
    window._on_stream_requested(True)
    assert window.dashboard_page.video_panel.receiver.started

    window.close()

    assert window.dashboard_page.video_panel.receiver.stopped


def _last_video_request():
    """Finds the /video_request topic's most recent published payload,
    decoded from the JSON string the real RosBridgeClient sends - there is
    no fake client here (make_fake_client_factory builds a real
    RosBridgeClient over FakeRos/FakeTopic), so requests are observed the
    same way tests/test_ros_client.py observes them: through the topic."""
    topic = next(t for t in FakeTopic.instances if t.name == "/video_request")
    return json.loads(topic.published_messages[-1]["data"])


def test_dashboard_has_a_video_panel(qtbot):
    window, _ = make_window(qtbot)

    assert window.dashboard_page.video_panel is not None


def test_enabling_video_publishes_a_request_with_our_own_address(qtbot, monkeypatch):
    # local_address_for makes a real UDP connect() to discover our route to
    # the rover - on a host with no route to 192.168.178.33 (CI, a
    # container, any other network) that legitimately returns "", which
    # would silently take the "no route" branch below and publish nothing,
    # making _last_video_request() below fail with a bare StopIteration.
    # Stub it so this test doesn't depend on the machine's route table.
    window, _ = make_window(qtbot, initial_host="192.168.178.33")
    monkeypatch.setattr(window, "local_address_for", lambda host, port: "10.20.30.40")

    window._on_stream_requested(True)

    request = _last_video_request()
    assert request["enable"] is True
    assert request["port"] == 5600
    assert request["host"] == "10.20.30.40"


def test_video_request_width_matches_double_the_receiver_default_width(qtbot):
    # Important 5: MainWindow hardcodes the requested capture width/height
    # (1344x376) here while VideoReceiver independently defaults to
    # 672x376 (post-crop, since the rover crops the capture width in half) -
    # two literals in different modules, tied only by convention and
    # documented nowhere. Pin the invariant so they can't silently drift
    # apart (the symptom of drift is Important 1's misleading message).
    window, _ = make_window(qtbot, initial_host="192.168.178.33")

    window._on_stream_requested(True)
    request = _last_video_request()

    default_receiver = VideoReceiver()
    assert default_receiver.width == request["width"] // 2
    assert default_receiver.height == request["height"]


def test_disabling_video_stops_the_receiver_even_if_the_rover_never_answers(qtbot):
    window, _ = make_window(qtbot, initial_host="192.168.178.33")
    window._on_stream_requested(True)

    window._on_stream_requested(False)

    assert window.dashboard_page.video_panel.receiver.stopped


def test_video_status_reaches_the_panel(qtbot):
    # Tightened per the final review: "STREAMING" in text.upper() alone
    # also passes on "rover: streaming (not receiving locally)" - the exact
    # text commit 20c7c0f introduced for "reported before local polling
    # starts" - so it would not catch a regression of that behavior. This
    # window never calls _on_stream_requested/set_streaming, so the panel
    # is not locally streaming and must show the qualified text, not plain
    # success.
    window, _ = make_window(qtbot, initial_host="192.168.178.33")

    window._on_video_status({"state": "streaming", "detail": "10.0.0.5:5600"})

    text = window.dashboard_page.video_panel.status_label.text()
    assert "not receiving locally" in text.lower()
    assert not text.upper().startswith("STREAMING ")


def test_requesting_video_without_a_connection_is_ignored(qtbot):
    window, client = make_window(qtbot, initial_host="")
    assert client is None

    window._on_stream_requested(True)

    assert window.dashboard_page.video_panel._streaming is False


def test_local_address_is_the_interface_that_reaches_the_rover(qtbot):
    window, _ = make_window(qtbot)

    address = window.local_address_for("192.168.178.33", 9090)

    # A real, unmocked UDP connect() - tolerant of a host with no route to
    # this address (CI, a container, any other network), where
    # local_address_for legitimately returns "". Only assert the shape when
    # a route actually exists.
    assert address == "" or address.count(".") == 3
