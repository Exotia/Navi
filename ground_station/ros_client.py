"""Wraps roslibpy for connecting to rosbridge; re-emits everything as Qt
signals so roslibpy's background-thread callbacks are safely marshaled to
the Qt GUI thread by Qt's own queued-connection mechanism."""

import json

import roslibpy
from PySide6.QtCore import QObject, Signal


class RosSignals(QObject):
    twist_received = Signal(dict)
    nodes_received = Signal(list)
    connection_changed = Signal(bool)
    video_status_received = Signal(dict)


class RosBridgeClient:
    def __init__(self, host: str, port: int = 9090,
                 ros_factory=roslibpy.Ros, topic_factory=roslibpy.Topic,
                 message_factory=roslibpy.Message):
        self.signals = RosSignals()
        self._topic_factory = topic_factory
        self._message_factory = message_factory
        self._ros = ros_factory(host=host, port=port)
        self._manual_twist_topic = None
        self._video_request_topic = None
        self._video_status_topic = None

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
        # roslibpy's get_nodes callback receives a ServiceResponse (a
        # dict-like wrapper around the rosapi/Nodes service response, i.e.
        # {"nodes": [...]}) - NOT the node list itself. Unwrap it here so
        # nodes_received always carries a plain list, matching its Signal(list)
        # declaration.
        self._ros.get_nodes(lambda response: self.signals.nodes_received.emit(list(response["nodes"])))

    def publish_manual_twist(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        """Publishes on /manual_twist - requires subscribe_manual_twist() to
        have been called first (it creates the Topic object this reuses)."""
        if self._manual_twist_topic is None:
            return
        self._manual_twist_topic.publish(self._message_factory({
            "linear": {"x": linear_x, "y": linear_y, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular_z},
        }))

    def subscribe_video_status(self, topic_name: str = "/video_status") -> None:
        """The rover's own account of the stream: stopped, starting,
        streaming, or failed. Distinct from whether frames are actually
        arriving, which only the receiver can tell - a rover reporting
        'streaming' while no packets land is the signature of a blocked
        UDP port."""
        topic = self._topic_factory(self._ros, topic_name, "std_msgs/String")
        topic.subscribe(lambda msg: self.signals.video_status_received.emit(
            self._parse_status(msg.get("data", ""))))
        self._video_status_topic = topic

    @staticmethod
    def _parse_status(payload: str) -> dict:
        try:
            status = json.loads(payload)
        except (json.JSONDecodeError, TypeError) as exc:
            return {"state": "failed", "detail": f"bad status JSON: {exc}"}
        if not isinstance(status, dict):
            return {"state": "failed", "detail": "status was not a JSON object"}
        return {"state": status.get("state", "failed"),
                "detail": status.get("detail", "")}

    def publish_video_request(self, enable: bool, host: str, port: int, width: int,
                              height: int, fps: int, bitrate_kbps: int) -> None:
        """Asks the rover to start or stop streaming to host:port. The host
        is ours, not the rover's: the rover is the server side of rosbridge
        and has no other way to learn where we are."""
        if self._video_request_topic is None:
            self._video_request_topic = self._topic_factory(
                self._ros, "/video_request", "std_msgs/String")
        self._video_request_topic.publish(self._message_factory({
            "data": json.dumps({
                "enable": enable, "host": host, "port": port, "width": width,
                "height": height, "fps": fps, "bitrate_kbps": bitrate_kbps,
            }),
        }))
