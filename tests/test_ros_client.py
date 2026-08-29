from ground_station.ros_client import RosBridgeClient


class FakeTopic:
    instances = []

    def __init__(self, ros, name, msg_type, **options):
        self.ros = ros
        self.name = name
        self.msg_type = msg_type
        self.callback = None
        self.published_messages = []
        # Whatever roslibpy keywords the client passed - throttle_rate today.
        # Kept rather than swallowed: a subscription's throttle is part of
        # what the client promises, so a test has to be able to assert on it.
        self.options = options
        FakeTopic.instances.append(self)

    def subscribe(self, callback):
        self.callback = callback

    def publish(self, message):
        self.published_messages.append(message)


def fake_message_factory(data):
    return data


class FakeRos:
    instances = []

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.is_connected = False
        self.ready_callback = None
        self.get_nodes_callback = None
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
        if self.ready_callback:
            self.ready_callback()

    def close(self):
        self.is_connected = False

    def get_nodes(self, callback, errback=None):
        self.get_nodes_callback = callback


def make_client(qtbot):
    FakeRos.instances.clear()
    FakeTopic.instances.clear()
    return RosBridgeClient(host="localhost", port=9090,
                            ros_factory=FakeRos, topic_factory=FakeTopic,
                            message_factory=fake_message_factory)


def test_connect_starts_ros_and_emits_connection_changed(qtbot):
    client = make_client(qtbot)
    with qtbot.waitSignal(client.signals.connection_changed, timeout=1000) as blocker:
        client.connect()

    assert blocker.args == [True]
    assert client.is_connected is True


def test_subscribe_manual_twist_creates_twist_topic(qtbot):
    client = make_client(qtbot)
    client.connect()
    client.subscribe_manual_twist()

    topic = FakeTopic.instances[-1]
    assert topic.name == "/manual_twist"
    assert topic.msg_type == "geometry_msgs/Twist"


def test_twist_message_emits_twist_received_signal(qtbot):
    client = make_client(qtbot)
    client.connect()
    client.subscribe_manual_twist()
    topic = FakeTopic.instances[-1]

    sample_msg = {"linear": {"x": 0.4, "y": 0.0, "z": 0.0},
                  "angular": {"x": 0.0, "y": 0.0, "z": 0.1}}
    with qtbot.waitSignal(client.signals.twist_received, timeout=1000) as blocker:
        topic.callback(sample_msg)

    assert blocker.args == [sample_msg]


def test_mid_session_close_event_emits_connection_changed_false(qtbot):
    client = make_client(qtbot)
    client.connect()
    ros = FakeRos.instances[-1]

    with qtbot.waitSignal(client.signals.connection_changed, timeout=1000) as blocker:
        # roslibpy's factory emits a "close" event (with the closed proto
        # as the argument) when the underlying websocket connection drops
        # mid-session — this is the real hook, distinct from on_ready.
        ros.trigger_event("close", None)

    assert blocker.args == [False]


def test_poll_nodes_emits_nodes_received_signal(qtbot):
    client = make_client(qtbot)
    client.connect()
    client.poll_nodes()
    ros = FakeRos.instances[-1]

    # roslibpy's real get_nodes callback receives a ServiceResponse - a
    # dict-like wrapper of the rosapi/Nodes response, i.e. {"nodes": [...]}
    # - not a bare list. A plain dict here is enough to exercise the same
    # __getitem__ access poll_nodes() actually does.
    with qtbot.waitSignal(client.signals.nodes_received, timeout=1000) as blocker:
        ros.get_nodes_callback({"nodes": ["/cmd_vel_bridge", "/rosbridge_websocket"]})

    assert blocker.args == [["/cmd_vel_bridge", "/rosbridge_websocket"]]


def test_publish_manual_twist_before_subscribe_does_nothing(qtbot):
    client = make_client(qtbot)
    client.connect()

    # subscribe_manual_twist() was never called, so there's no topic to
    # publish on yet - this must not raise.
    client.publish_manual_twist(0.4, -0.05, 0.1)

    assert FakeTopic.instances == []


def test_publish_manual_twist_publishes_twist_on_the_subscribed_topic(qtbot):
    client = make_client(qtbot)
    client.connect()
    client.subscribe_manual_twist()
    topic = FakeTopic.instances[-1]

    client.publish_manual_twist(0.4, -0.05, 0.1)

    assert topic.published_messages == [{
        "linear": {"x": 0.4, "y": -0.05, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.1},
    }]


def test_subscribe_video_status_emits_parsed_json():
    FakeTopic.instances.clear()
    client = RosBridgeClient("host", 9090, ros_factory=FakeRos,
                             topic_factory=FakeTopic,
                             message_factory=fake_message_factory)
    received = []
    client.signals.video_status_received.connect(received.append)

    client.subscribe_video_status()
    topic = FakeTopic.instances[-1]
    topic.callback({"data": '{"state": "streaming", "detail": "10.0.0.5:5600"}'})

    assert received == [{"state": "streaming", "detail": "10.0.0.5:5600"}]


def test_subscribe_video_status_reports_malformed_payloads_instead_of_raising():
    FakeTopic.instances.clear()
    client = RosBridgeClient("host", 9090, ros_factory=FakeRos,
                             topic_factory=FakeTopic,
                             message_factory=fake_message_factory)
    received = []
    client.signals.video_status_received.connect(received.append)

    client.subscribe_video_status()
    FakeTopic.instances[-1].callback({"data": "{not json"})

    assert received[0]["state"] == "failed"
    assert "JSON" in received[0]["detail"]


def test_subscribe_video_status_reports_non_string_data_instead_of_raising():
    # rosbridge delivers whatever the wire carries; a peer (or a bug) could
    # send a std_msgs/String with a non-string "data" field. json.loads on a
    # non-string raises TypeError, not JSONDecodeError - this must not
    # propagate out of the roslibpy subscription callback.
    FakeTopic.instances.clear()
    client = RosBridgeClient("host", 9090, ros_factory=FakeRos,
                             topic_factory=FakeTopic,
                             message_factory=fake_message_factory)
    received = []
    client.signals.video_status_received.connect(received.append)

    client.subscribe_video_status()
    FakeTopic.instances[-1].callback({"data": 5})

    assert received[0]["state"] == "failed"
    assert received[0]["detail"]


def test_publish_video_request_sends_json_on_the_request_topic():
    import json

    FakeTopic.instances.clear()
    client = RosBridgeClient("host", 9090, ros_factory=FakeRos,
                             topic_factory=FakeTopic,
                             message_factory=fake_message_factory)

    client.publish_video_request(enable=True, host="192.168.178.101", port=5600,
                                 width=1344, height=376, fps=30, bitrate_kbps=800)

    topic = FakeTopic.instances[-1]
    assert topic.name == "/video_request"
    assert topic.msg_type == "std_msgs/String"
    payload = json.loads(topic.published_messages[-1]["data"])
    assert payload == {"enable": True, "host": "192.168.178.101", "port": 5600,
                       "width": 1344, "height": 376, "fps": 30, "bitrate_kbps": 800}


def test_publish_video_request_reuses_one_topic_across_calls():
    FakeTopic.instances.clear()
    client = RosBridgeClient("host", 9090, ros_factory=FakeRos,
                             topic_factory=FakeTopic,
                             message_factory=fake_message_factory)

    client.publish_video_request(enable=True, host="10.0.0.5", port=5600,
                                 width=1344, height=376, fps=30, bitrate_kbps=800)
    client.publish_video_request(enable=False, host="10.0.0.5", port=5600,
                                 width=1344, height=376, fps=30, bitrate_kbps=800)

    request_topics = [t for t in FakeTopic.instances if t.name == "/video_request"]
    assert len(request_topics) == 1
    assert len(request_topics[0].published_messages) == 2


import json

from ground_station.ros_client import LOCALIZATION_POSE_THROTTLE_MS


def test_subscribe_localization_status_emits_the_parsed_json(qtbot):
    client = make_client(qtbot)
    client.connect()
    client.subscribe_localization_status()
    topic = next(t for t in FakeTopic.instances if t.name == "/localization/status")
    assert topic.msg_type == "std_msgs/String"

    with qtbot.waitSignal(client.signals.localization_status_received,
                          timeout=1000) as blocker:
        topic.callback({"data": json.dumps({
            "state": "SEARCHING", "seconds_since_ok": 4.2, "source": "zed_vio",
            "distance_travelled": 12.5, "mount_offset_verified": True})})

    assert blocker.args[0] == {
        "state": "SEARCHING", "seconds_since_ok": 4.2, "source": "zed_vio",
        "distance_travelled": 12.5, "mount_offset_verified": True, "detail": ""}


def test_a_malformed_localization_status_reads_as_off_with_the_reason(qtbot):
    # Same reasoning as the video status: a bad payload must not raise
    # inside roslibpy's background thread. OFF is the right fallback state -
    # it is what the panel shows when nothing can be trusted - and the
    # reason is carried so it is not lost.
    client = make_client(qtbot)
    client.connect()
    client.subscribe_localization_status()
    topic = next(t for t in FakeTopic.instances if t.name == "/localization/status")

    with qtbot.waitSignal(client.signals.localization_status_received,
                          timeout=1000) as blocker:
        topic.callback({"data": "{not json"})

    assert blocker.args[0]["state"] == "OFF"
    assert "bad status JSON" in blocker.args[0]["detail"]


def test_subscribe_localization_pose_throttles_at_five_hertz(qtbot):
    # The pose is published at ~30 Hz and is wanted here for one header
    # readout. Throttling at the rosbridge server rather than on this laptop
    # is 25 messages a second that never cross the field link.
    client = make_client(qtbot)
    client.connect()
    client.subscribe_localization_pose()
    topic = next(t for t in FakeTopic.instances if t.name == "/localization/pose")

    assert topic.msg_type == "nav_msgs/Odometry"
    assert topic.options["throttle_rate"] == LOCALIZATION_POSE_THROTTLE_MS
    assert LOCALIZATION_POSE_THROTTLE_MS == 200


def test_subscribe_localization_pose_emits_x_y_and_yaw(qtbot):
    import math

    client = make_client(qtbot)
    client.connect()
    client.subscribe_localization_pose()
    topic = next(t for t in FakeTopic.instances if t.name == "/localization/pose")

    with qtbot.waitSignal(client.signals.localization_pose_received,
                          timeout=1000) as blocker:
        topic.callback({"pose": {"pose": {
            "position": {"x": 1.5, "y": 2.5, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0,
                            "z": math.sin(math.pi / 4), "w": math.cos(math.pi / 4)}}}})

    assert blocker.args[0]["x"] == 1.5
    assert blocker.args[0]["y"] == 2.5
    assert abs(blocker.args[0]["yaw"] - math.pi / 2) < 1e-9


def test_map_status_is_parsed_and_emitted(qtbot):
    client = make_client(qtbot)
    received = []
    client.signals.map_status_received.connect(received.append)
    client.subscribe_map_status()
    topic = next(t for t in FakeTopic.instances if t.name == "/localization/map_status")
    assert topic.msg_type == "std_msgs/String"
    topic.callback({"data": '{"cells_seen": 3, "maps": ["x"]}'})
    assert received[-1].cells_seen == 3 and received[-1].maps == ["x"]
    topic.callback({"data": "garbage"})
    assert received[-1] is None


def test_send_map_command_publishes_a_string_on_the_command_topic(qtbot):
    client = make_client(qtbot)
    client.connect()
    client.send_map_command("save", "yard")
    topic = next(t for t in FakeTopic.instances if t.name == "/localization/map_command")
    assert topic.msg_type == "std_msgs/String"
    assert json.loads(topic.published_messages[-1]["data"]) == {"action": "save", "name": "yard"}


def test_send_map_command_without_a_connection_publishes_nothing(qtbot):
    client = make_client(qtbot)
    client.send_map_command("clear")
    assert not any(t.name == "/localization/map_command" for t in FakeTopic.instances)
