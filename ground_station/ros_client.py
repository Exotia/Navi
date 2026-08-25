"""Wraps roslibpy for connecting to rosbridge; re-emits everything as Qt
signals so roslibpy's background-thread callbacks are safely marshaled to
the Qt GUI thread by Qt's own queued-connection mechanism."""

import roslibpy
from PySide6.QtCore import QObject, Signal


class RosSignals(QObject):
    twist_received = Signal(dict)
    nodes_received = Signal(list)
    connection_changed = Signal(bool)


class RosBridgeClient:
    def __init__(self, host: str, port: int = 9090,
                 ros_factory=roslibpy.Ros, topic_factory=roslibpy.Topic,
                 message_factory=roslibpy.Message):
        self.signals = RosSignals()
        self._topic_factory = topic_factory
        self._message_factory = message_factory
        self._ros = ros_factory(host=host, port=port)
        self._cmd_vel_topic = None

    def connect(self) -> None:
        self._ros.on_ready(lambda: self.signals.connection_changed.emit(True))
        self._ros.on("close", lambda *args: self.signals.connection_changed.emit(False))
        self._ros.run()

    def close(self) -> None:
        self._ros.close()
        self.signals.connection_changed.emit(False)

    @property
    def is_connected(self) -> bool:
        return bool(self._ros.is_connected)

    def subscribe_cmd_vel(self, topic_name: str = "/cmd_vel") -> None:
        self._cmd_vel_topic = self._topic_factory(self._ros, topic_name, "geometry_msgs/Twist")
        self._cmd_vel_topic.subscribe(lambda msg: self.signals.twist_received.emit(msg))

    def poll_nodes(self) -> None:
        self._ros.get_nodes(lambda nodes: self.signals.nodes_received.emit(nodes))

    def publish_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        """Publishes a Twist on the same /cmd_vel topic subscribe_cmd_vel()
        set up - requires subscribe_cmd_vel() to have been called first.
        Because this app is also subscribed to that topic, a published
        message loops back through twist_received, which is why the Drive
        card shows gamepad-driven commands without any separate display
        path."""
        if self._cmd_vel_topic is None:
            return
        self._cmd_vel_topic.publish(self._message_factory({
            "linear": {"x": linear_x, "y": linear_y, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular_z},
        }))
