from ground_station.ros_client import RosBridgeClient


class FakeTopic:
    instances = []

    def __init__(self, ros, name, msg_type):
        self.ros = ros
        self.name = name
        self.msg_type = msg_type
        self.callback = None
        FakeTopic.instances.append(self)

    def subscribe(self, callback):
        self.callback = callback


class FakeRos:
    instances = []

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.is_connected = False
        self.ready_callback = None
        self.get_nodes_callback = None
        FakeRos.instances.append(self)

    def on_ready(self, callback):
        self.ready_callback = callback

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
                            ros_factory=FakeRos, topic_factory=FakeTopic)


def test_connect_starts_ros_and_emits_connection_changed(qtbot):
    client = make_client(qtbot)
    with qtbot.waitSignal(client.signals.connection_changed, timeout=1000) as blocker:
        client.connect()

    assert blocker.args == [True]
    assert client.is_connected is True


def test_subscribe_cmd_vel_creates_twist_topic(qtbot):
    client = make_client(qtbot)
    client.connect()
    client.subscribe_cmd_vel()

    topic = FakeTopic.instances[-1]
    assert topic.name == "/cmd_vel"
    assert topic.msg_type == "geometry_msgs/Twist"


def test_twist_message_emits_twist_received_signal(qtbot):
    client = make_client(qtbot)
    client.connect()
    client.subscribe_cmd_vel()
    topic = FakeTopic.instances[-1]

    sample_msg = {"linear": {"x": 0.4, "y": 0.0, "z": 0.0},
                  "angular": {"x": 0.0, "y": 0.0, "z": 0.1}}
    with qtbot.waitSignal(client.signals.twist_received, timeout=1000) as blocker:
        topic.callback(sample_msg)

    assert blocker.args == [sample_msg]


def test_poll_nodes_emits_nodes_received_signal(qtbot):
    client = make_client(qtbot)
    client.connect()
    client.poll_nodes()
    ros = FakeRos.instances[-1]

    with qtbot.waitSignal(client.signals.nodes_received, timeout=1000) as blocker:
        ros.get_nodes_callback(["/cmd_vel_bridge", "/rosbridge_websocket"])

    assert blocker.args == [["/cmd_vel_bridge", "/rosbridge_websocket"]]
