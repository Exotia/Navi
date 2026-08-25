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
        self._manual_twist_topic = None

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

    def subscribe_manual_twist(self, topic_name: str = "/manual_twist") -> None:
        """/manual_twist is the raw gamepad-derived Twist - not /cmd_vel.
        Nothing downstream subscribes to it yet; a later mode-supervisor
        module will be the one that decides whether this (vs. an autonomy
        source) becomes the rover's actual /cmd_vel. Subscribing to our own
        publish here is just a wire-level integration check (proves publish
        really reaches rosbridge and comes back) - the Drive card's primary
        display path is local (see MainWindow._poll_gamepad), not this
        loopback."""
        self._manual_twist_topic = self._topic_factory(self._ros, topic_name, "geometry_msgs/Twist")
        self._manual_twist_topic.subscribe(lambda msg: self.signals.twist_received.emit(msg))

    def poll_nodes(self) -> None:
        self._ros.get_nodes(lambda nodes: self.signals.nodes_received.emit(nodes))

    def publish_manual_twist(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        """Publishes on /manual_twist - requires subscribe_manual_twist() to
        have been called first (it creates the Topic object this reuses)."""
        if self._manual_twist_topic is None:
            return
        self._manual_twist_topic.publish(self._message_factory({
            "linear": {"x": linear_x, "y": linear_y, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular_z},
        }))
