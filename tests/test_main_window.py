import json

from ground_station.ros_client import RosBridgeClient
from ground_station.ui import main_window
from ground_station.ui.main_window import MainWindow
from ground_station.video_receiver import VideoReceiver
from tests.test_video_panel import FakeReceiver


class FakeTopic:
    instances = []

    def __init__(self, ros, name, msg_type, **options):
        self.name = name
        self.msg_type = msg_type
        self.callback = None
        self.published_messages = []
        self.options = options
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


class FakeRosThatTimesOutOnRun(FakeRos):
    """Mirrors roslibpy: Ros.run() raises RosTimeoutError when rosbridge
    doesn't answer within the connection timeout, while auto-reconnect
    keeps running in the background and Topic subscriptions registered
    beforehand still get wired up once a later attempt succeeds."""

    def run(self):
        raise TimeoutError("Failed to connect to ROS")


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


def make_timeout_client_factory():
    def factory(host, port=9090):
        return RosBridgeClient(host=host, port=port, ros_factory=FakeRosThatTimesOutOnRun,
                               topic_factory=FakeTopic)
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


def test_disconnect_stops_the_local_video_receiver(qtbot, monkeypatch):
    window, _ = make_window(qtbot, initial_host="192.168.178.33")
    monkeypatch.setattr(window, "local_address_for", lambda host, port: "10.20.30.40")
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


def test_closing_the_window_stops_the_local_video_receiver(qtbot, monkeypatch):
    window, _ = make_window(qtbot, initial_host="192.168.178.33")
    monkeypatch.setattr(window, "local_address_for", lambda host, port: "10.20.30.40")
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


def test_video_request_width_matches_double_the_receiver_default_width(qtbot, monkeypatch):
    # Important 5: MainWindow hardcodes the requested capture width/height
    # (1344x376) here while VideoReceiver independently defaults to
    # 672x376 (post-crop, since the rover crops the capture width in half) -
    # two literals in different modules, tied only by convention and
    # documented nowhere. Pin the invariant so they can't silently drift
    # apart (the symptom of drift is Important 1's misleading message).
    window, _ = make_window(qtbot, initial_host="192.168.178.33")
    monkeypatch.setattr(window, "local_address_for", lambda host, port: "10.20.30.40")

    window._on_stream_requested(True)
    request = _last_video_request()

    default_receiver = VideoReceiver()
    assert default_receiver.width == request["width"] // 2
    assert default_receiver.height == request["height"]


def test_the_requested_geometry_is_advisory_for_the_rovers_zed_topic_source(qtbot,
                                                                             monkeypatch):
    # Critical, final review: with localisation on, the rover's video_sender
    # takes its frames from the ZED wrapper's topic, and the wrapper - which
    # owns the camera - publishes 640x360, not the 1344x376 UVC capture size
    # this window asks for. Nothing the ground station sends can change that,
    # so the contract is: our width/height are advisory for that source, the
    # rover adopts the published geometry, and it reports the geometry it is
    # actually sending in /video_status's detail. This test pins the ground
    # station's half of it - the request is still sent (the v4l2 source does
    # use it), and a status whose detail names a different geometry is a
    # success, not a failure to be surfaced as one.
    window, _ = make_window(qtbot, initial_host="192.168.178.33")
    monkeypatch.setattr(window, "local_address_for", lambda host, port: "10.20.30.40")

    window._on_stream_requested(True)
    request = _last_video_request()
    assert (request["width"], request["height"]) == (1344, 376)

    window._on_video_status({"state": "streaming", "detail": "10.20.30.40:5600 640x360"})

    text = window.dashboard_page.video_panel.status_label.text()
    assert "640x360" in text
    assert "failed" not in text.lower()


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


def test_entering_simulation_stops_rover_video_and_switches_port(qtbot):
    window, _ = make_window(qtbot)

    window.dashboard_page.mode_changed.emit("simulation")

    assert window._mode == "simulation"
    assert window.dashboard_page.video_panel.receiver.port == 5601
    assert window.dashboard_page.video_panel.dead_reckoning is True
    # The rover keeps being driven - only its camera is turned off, to spare
    # the link while nobody is looking at it.
    request = _last_video_request()
    assert request["enable"] is False


def test_leaving_simulation_returns_to_the_rover_camera(qtbot, monkeypatch):
    # Asserting the port and the marker alone passed for as long as the
    # resume request was missing entirely: the local receiver comes back on
    # 5600 by itself, so the port flipping back proves only that this laptop
    # is listening - not that anything is sending.
    window, _ = make_window(qtbot)
    monkeypatch.setattr(window, "local_address_for", lambda host, port: "10.20.30.40")
    window._on_stream_requested(True)
    window.dashboard_page.mode_changed.emit("simulation")

    window.dashboard_page.mode_changed.emit("manual")

    assert window.dashboard_page.video_panel.receiver.port == 5600
    assert window.dashboard_page.video_panel.dead_reckoning is False
    request = _last_video_request()
    assert request["enable"] is True
    assert request["port"] == 5600
    assert request["host"] == "10.20.30.40"


def test_semi_auto_shows_the_simulation_without_the_dead_reckoning_marker(qtbot):
    # Semi-autonomous shows the same Gazebo stream on the same port, but the
    # rover in it is placed by localisation, so the DEAD RECKONING warning
    # would be a lie. What replaces it is the localisation marker (Task 3).
    window, _ = make_window(qtbot)

    window.dashboard_page.mode_changed.emit("semi_auto")

    assert window._mode == "semi_auto"
    assert window.dashboard_page.video_panel.receiver.port == 5601
    assert window.dashboard_page.video_panel.dead_reckoning is False
    assert _last_video_request()["enable"] is False


def test_autonomous_changes_nothing_because_nothing_is_built(qtbot):
    # The radio is disabled so this cannot happen from the UI. If the signal
    # is emitted anyway (a test, a future caller), the window must not put
    # the panel into a half-configured state - it leaves the view alone and
    # says so on stderr.
    window, _ = make_window(qtbot)
    port_before = window.dashboard_page.video_panel.receiver.port

    window.dashboard_page.mode_changed.emit("autonomous")

    assert window._mode == "autonomous"
    assert window.dashboard_page.video_panel.receiver.port == port_before


def test_the_video_toggle_does_not_command_the_rover_in_simulation(qtbot, monkeypatch):
    # The toggle acts on the source on screen. In simulation that is the
    # simulation, which has no control plane; sending enable=True to the
    # rover here would push 800 kbps over the field link to port 5600 with
    # nothing listening, for as long as the mode lasts - undoing the reason
    # the mode stops the rover's camera at all.
    window, _ = make_window(qtbot)
    monkeypatch.setattr(window, "local_address_for", lambda host, port: "10.20.30.40")
    window.dashboard_page.mode_changed.emit("simulation")
    topic = next(t for t in FakeTopic.instances if t.name == "/video_request")
    requests_before = len(topic.published_messages)

    window.dashboard_page.video_panel.toggle_button.click()

    assert len(topic.published_messages) == requests_before
    # ...and the local simulation receiver did start, so the toggle is not
    # simply dead in this mode.
    assert window.dashboard_page.video_panel.streaming is True


def test_a_rosbridge_drop_does_not_tear_down_the_simulation_view(qtbot, monkeypatch):
    # The simulation's sender is a local process on this laptop and does not
    # depend on rosbridge, so a blip on the field link must not black out a
    # running simulation - and certainly must not label it with the wording
    # for an operator switching video off.
    window, _ = make_window(qtbot, video_receiver=FakeReceiver(frame=bytes(4 * 2 * 3)))
    monkeypatch.setattr(window, "local_address_for", lambda host, port: "10.20.30.40")
    window._on_stream_requested(True)
    window.dashboard_page.mode_changed.emit("simulation")
    panel = window.dashboard_page.video_panel
    assert panel.streaming is True

    ros = FakeRos.instances[-1]
    ros.trigger_event("close", None)

    assert panel.streaming is True
    assert "OFF" not in panel.status_label.text().upper()


def test_entering_simulation_shows_receiving_not_a_stale_rover_word(qtbot):
    # Regression for the bug the reviewer found: set_source's
    # stop_receiver() resets _rover_state to "stopped" on the way in, and
    # the sim has no /video_status of its own to overwrite it afterwards -
    # so the panel must show the local fact (frames are arriving) rather
    # than that leftover rover word.
    receiver = FakeReceiver(frame=bytes(4 * 2 * 3))
    window, _ = make_window(qtbot, video_receiver=receiver)
    window._on_stream_requested(True)

    window.dashboard_page.mode_changed.emit("simulation")
    window.dashboard_page.video_panel._poll_frame(now=100.0)

    text = window.dashboard_page.video_panel.status_label.text().upper()
    assert "STOPPED" not in text
    assert "RECEIVING" in text


def test_the_twist_still_reaches_the_rover_in_semi_auto(qtbot):
    # The rover is driven in both modes. Anything else would make the mode
    # switch a control change disguised as a view change.
    gamepad = FakeGamepadReader(connected=True, twist=(0.4, 0.0, 0.2))
    window, _ = make_window(qtbot, initial_host="localhost", gamepad_reader=gamepad)

    window.dashboard_page.mode_changed.emit("semi_auto")
    window._poll_gamepad()

    topic = next(t for t in FakeTopic.instances if t.name == "/manual_twist")
    assert topic.published_messages[-1] == {
        "linear": {"x": 0.4, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.2},
    }


def test_the_twist_still_reaches_the_rover_in_simulation(qtbot):
    gamepad = FakeGamepadReader(connected=True, twist=(0.4, 0.0, 0.2))
    window, _ = make_window(qtbot, initial_host="localhost", gamepad_reader=gamepad)

    window.dashboard_page.mode_changed.emit("simulation")
    window._poll_gamepad()

    topic = next(t for t in FakeTopic.instances if t.name == "/manual_twist")
    assert topic.published_messages[-1] == {
        "linear": {"x": 0.4, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.2},
    }


def test_the_video_toggle_is_refused_in_semi_auto_with_the_reason_on_the_panel(qtbot,
                                                                               monkeypatch):
    # Semi-autonomous exists so the operator drives on the Gazebo view and
    # the field link carries no video at all. A toggle here must not command
    # the rover's camera, must not start a local receiver pointed at a
    # camera that is off, and must not be silent about either.
    window, _ = make_window(qtbot)
    monkeypatch.setattr(window, "local_address_for", lambda host, port: "10.20.30.40")
    window.dashboard_page.mode_changed.emit("semi_auto")
    topic = next(t for t in FakeTopic.instances if t.name == "/video_request")
    requests_before = len(topic.published_messages)

    window.dashboard_page.video_panel.toggle_button.click()

    assert len(topic.published_messages) == requests_before
    assert window.dashboard_page.video_panel.status_label.text() == (
        "no camera stream in semi-autonomous mode")


def test_the_semi_auto_refusal_does_not_start_a_local_receiver(qtbot):
    # The simulation's own stream is started by the mode switch, not by this
    # button - and in semi-auto the button must move nothing at all.
    window, _ = make_window(qtbot)
    window.dashboard_page.mode_changed.emit("semi_auto")
    panel = window.dashboard_page.video_panel
    streaming_before = panel.streaming

    window._on_stream_requested(True)

    assert panel.streaming is streaming_before


def test_connecting_subscribes_to_both_localisation_topics(qtbot):
    window, _ = make_window(qtbot)

    names = [t.name for t in FakeTopic.instances]
    assert "/localization/status" in names
    assert "/localization/pose" in names


def test_a_localisation_status_reaches_the_panel(qtbot):
    window, client = make_window(qtbot)
    window.dashboard_page.mode_changed.emit("semi_auto")
    topic = next(t for t in FakeTopic.instances if t.name == "/localization/status")

    topic.callback({"data": json.dumps({
        "state": "SEARCHING", "seconds_since_ok": 4.2, "source": "zed_vio",
        "distance_travelled": 12.5, "mount_offset_verified": True})})

    assert window.dashboard_page.video_panel.title_label.text() == (
        "CAMERA / SIMULATION  -  SEARCHING … 4 s")


def test_the_last_status_is_on_screen_the_moment_semi_auto_is_entered(qtbot):
    # /localization/status arrives at 2 Hz. Waiting half a second with a
    # blank marker after a mode switch is half a second of the operator not
    # knowing whether to trust the picture they just switched to.
    window, _ = make_window(qtbot)
    topic = next(t for t in FakeTopic.instances if t.name == "/localization/status")
    topic.callback({"data": json.dumps({
        "state": "OK", "seconds_since_ok": 0.0, "source": "zed_vio",
        "distance_travelled": 1.0, "mount_offset_verified": True})})

    window.dashboard_page.mode_changed.emit("semi_auto")

    assert window.dashboard_page.video_panel.title_label.text() == (
        "CAMERA / SIMULATION  -  LOCALISED")


def test_the_header_reads_out_the_pose(qtbot):
    import math

    window, _ = make_window(qtbot)
    topic = next(t for t in FakeTopic.instances if t.name == "/localization/pose")

    topic.callback({"pose": {"pose": {
        "position": {"x": 1.5, "y": -2.25, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0,
                        "z": math.sin(math.pi / 4), "w": math.cos(math.pi / 4)}}}})

    assert window.localization_label.text() == "LOC: x 1.50  y -2.25  yaw 90.0°"


def test_the_header_says_so_before_any_pose_arrives(qtbot):
    window, _ = make_window(qtbot)

    assert window.localization_label.text() == "LOC: NO POSE"


def test_a_localisation_status_that_stops_arriving_stops_being_asserted(qtbot):
    # A marker reading LOCALISED because that is what the rover said before
    # the link died is the worst failure this panel has: the operator is
    # driving on a picture, and the marker is the one thing telling them
    # whether to trust it.
    window, _ = make_window(qtbot)
    window.dashboard_page.mode_changed.emit("semi_auto")
    topic = next(t for t in FakeTopic.instances if t.name == "/localization/status")
    topic.callback({"data": json.dumps({
        "state": "OK", "seconds_since_ok": 0.0, "source": "zed_vio",
        "distance_travelled": 1.0, "mount_offset_verified": True})})
    assert window.dashboard_page.video_panel.title_label.text().endswith("LOCALISED")

    window._check_staleness(now=window._localization_status_at + 3.5)

    assert window.dashboard_page.video_panel.title_label.text() == (
        "CAMERA / SIMULATION  -  NO LOCALISATION STATUS")
    assert window._localization_status is None


def test_a_fresh_status_is_not_called_stale(qtbot):
    window, _ = make_window(qtbot)
    window.dashboard_page.mode_changed.emit("semi_auto")
    topic = next(t for t in FakeTopic.instances if t.name == "/localization/status")
    topic.callback({"data": json.dumps({
        "state": "OK", "seconds_since_ok": 0.0, "source": "zed_vio",
        "distance_travelled": 1.0, "mount_offset_verified": True})})

    window._check_staleness(now=window._localization_status_at + 1.0)

    assert window.dashboard_page.video_panel.title_label.text().endswith("LOCALISED")


def test_entering_semi_auto_with_video_off_still_listens_for_the_simulation(qtbot):
    # The toggle is refused in semi-autonomous mode, so if the switch did
    # not start the receiver there would be no way to ever see the view.
    window, _ = make_window(qtbot)
    assert window.dashboard_page.video_panel.streaming is False

    window.dashboard_page.mode_changed.emit("semi_auto")

    panel = window.dashboard_page.video_panel
    assert panel.streaming is True
    assert panel.receiver.port == 5601
    assert panel.receiver.started is True


def test_semi_auto_mode_shows_the_map_row_and_other_modes_hide_it(qtbot):
    window, _ = make_window(qtbot)
    row = window.dashboard_page.map_row
    assert not row.isVisibleTo(window)
    window.dashboard_page.mode_changed.emit("semi_auto")
    assert row.isVisibleTo(window)
    window.dashboard_page.mode_changed.emit("manual")
    assert not row.isVisibleTo(window)


def test_drive_row_stays_visible_in_every_mode(qtbot):
    # Unlike the map row, the drive row must never disappear: the gamepad
    # publishes /manual_twist in every mode, so STOP and the deadman/lease
    # line have to stay reachable regardless of what the mode radio shows.
    window, _ = make_window(qtbot)
    row = window.dashboard_page.drive_row
    assert row.isVisibleTo(window)
    for mode in ("semi_auto", "manual", "simulation"):
        window.dashboard_page.mode_changed.emit(mode)
        assert row.isVisibleTo(window)


def test_map_status_reaches_the_row_and_goes_stale(qtbot):
    window, client = make_window(qtbot)
    topic = next(t for t in FakeTopic.instances if t.name == "/localization/map_status")
    topic.callback({"data": '{"cells_seen": 5, "maps": ["m"]}'})
    assert window.dashboard_page.map_row.load_button.isEnabled()
    window._check_staleness(now=window._map_status_at + 10.0)
    assert not window.dashboard_page.map_row.load_button.isEnabled()


def test_drive_status_clears_instantly_on_disconnect(qtbot):
    window, client = make_window(qtbot)
    topic = next(t for t in FakeTopic.instances if t.name == "/drive_status")
    topic.callback({"data": json.dumps({"connected": True, "lease": True,
                                        "coordinator_state": 3,
                                        "deadman_active": False,
                                        "twist_age_s": 0.1})})
    assert window._drive_status_at is not None
    assert window.dashboard_page.drive_row.manual_button.isEnabled()
    # A dropped rosbridge connection must blank the drive row at once,
    # rather than leaving stale drive controls looking live for the full
    # 3 s staleness window.
    window._on_connection_changed(False)
    assert window._drive_status_at is None
    assert not window.dashboard_page.drive_row.manual_button.isEnabled()


def test_map_row_buttons_send_commands(qtbot):
    window, client = make_window(qtbot)
    topic = next(t for t in FakeTopic.instances if t.name == "/localization/map_status")
    topic.callback({"data": '{"cells_seen": 5, "maps": ["m"]}'})
    row = window.dashboard_page.map_row
    row.load_button.click()
    command_topic = next(t for t in FakeTopic.instances if t.name == "/localization/map_command")
    assert json.loads(command_topic.published_messages[-1]["data"]) == {"action": "load", "name": "m"}


def test_the_requested_frame_rate_matches_the_zeds_grab_rate(qtbot, monkeypatch):
    # The ZED wrapper grabs at 15 fps (grab_frame_rate: 15 in
    # rover/src/navi_localization/config/zed_front.yaml), and the rover's
    # video_sender stamps the request's fps straight into
    # `rawvideoparse framerate=<fps>/1`. Asking for 30 tells the pipeline
    # that frames arrive twice as fast as they do, which is a lie about
    # every timestamp in the stream.
    window, _ = make_window(qtbot, initial_host="192.168.178.33")
    monkeypatch.setattr(window, "local_address_for", lambda host, port: "10.20.30.40")

    window._on_stream_requested(True)

    assert _last_video_request()["fps"] == 15
    assert main_window.ROVER_VIDEO_FPS == 15


def test_the_staleness_timer_ages_the_map_command_outcome_off_the_row(qtbot):
    # The rover repeats last_command in every status message for as long as
    # it stands, so only the clock retires it - and the row cannot age it
    # off on its own if no further status arrives to redraw the line.
    window, _ = make_window(qtbot)
    row = window.dashboard_page.map_row
    topic = next(t for t in FakeTopic.instances if t.name == "/localization/map_status")
    topic.callback({"data": json.dumps(
        {"cells_seen": 5, "maps": ["m"],
         "last_command": {"action": "save", "name": "m", "ok": True}})})
    assert "save" in row.status_label.text()

    # The rover is still publishing - _map_status_at keeps up - the row is
    # simply not being redrawn by a new state between those messages.
    window._map_status_at = row._last_command_at + 5.0
    window._check_staleness(now=row._last_command_at + 5.0)
    assert "save" in row.status_label.text()

    window._map_status_at = row._last_command_at + 11.0
    window._check_staleness(now=row._last_command_at + 11.0)
    assert "save" not in row.status_label.text()
    assert row.load_button.isEnabled()              # the row itself is not stale


def test_a_mode_round_trip_does_not_switch_the_rover_camera_on(qtbot, monkeypatch):
    # Video OFF in manual; semi mode forces the local receiver on for the
    # Gazebo stream. Coming back must not turn that into a /video_request
    # enable the operator never asked for.
    window, _ = make_window(qtbot, initial_host="192.168.178.33")
    monkeypatch.setattr(window, "local_address_for", lambda host, port: "10.20.30.40")
    panel = window.dashboard_page.video_panel
    assert panel.streaming is False

    window.dashboard_page.mode_changed.emit("semi_auto")
    assert panel.streaming is True
    window.dashboard_page.mode_changed.emit("manual")

    assert panel.streaming is False
    requests = [json.loads(m["data"]) for t in FakeTopic.instances
                if t.name == "/video_request" for m in t.published_messages]
    assert all(r["enable"] is False for r in requests), requests


def test_a_mode_round_trip_restores_the_rover_camera_that_was_on(qtbot, monkeypatch):
    window, _ = make_window(qtbot, initial_host="192.168.178.33")
    monkeypatch.setattr(window, "local_address_for", lambda host, port: "10.20.30.40")
    window._on_stream_requested(True)
    panel = window.dashboard_page.video_panel
    assert panel.streaming is True

    window.dashboard_page.mode_changed.emit("semi_auto")
    window.dashboard_page.mode_changed.emit("manual")

    assert panel.streaming is True
    assert _last_video_request()["enable"] is True


def test_a_timed_out_first_connect_still_registers_every_subscription(qtbot):
    # Regression: _connect_to used to call ros_client.connect() (which
    # raises when roslibpy's Ros.run() times out) BEFORE any of the
    # subscribe_* calls, so a slow/down rosbridge on first connect meant
    # every subscription was skipped - even though roslibpy keeps
    # reconnecting in the background and will eventually go ready.
    # subscribe_* must run regardless of whether connect() raises.
    window = MainWindow(
        ros_client_factory=make_timeout_client_factory(),
        initial_host=None,
        gamepad_reader=FakeGamepadReader(),
        video_receiver=FakeReceiver(),
    )
    qtbot.addWidget(window)
    FakeTopic.instances.clear()

    window._connect_to("localhost", 9090)

    names = {t.name for t in FakeTopic.instances}
    assert names == {
        "/manual_twist", "/video_status", "/localization/status",
        "/localization/pose", "/localization/map_status", "/drive_status",
    }
    client = window.ros_client
    assert client._manual_twist_topic is not None
    assert client._video_status_topic is not None
    assert client._localization_status_topic is not None
    assert client._localization_pose_topic is not None
    assert client._map_status_topic is not None
    assert client._drive_status_topic is not None
