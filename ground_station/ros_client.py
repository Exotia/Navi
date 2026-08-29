"""Wraps roslibpy for connecting to rosbridge; re-emits everything as Qt
signals so roslibpy's background-thread callbacks are safely marshaled to
the Qt GUI thread by Qt's own queued-connection mechanism."""

import json

import roslibpy
from PySide6.QtCore import QObject, Signal

from ground_station.models import pose_readout_from_odometry

# /localization/pose is published at the ZED wrapper's ~30 Hz and is wanted
# here for one header readout. rosbridge's own throttle_rate (milliseconds)
# drops the rest at the rover, before they cross the field link.
LOCALIZATION_POSE_THROTTLE_MS = 200      # 5 Hz


class RosSignals(QObject):
    twist_received = Signal(dict)
    nodes_received = Signal(list)
    connection_changed = Signal(bool)
    video_status_received = Signal(dict)
    localization_status_received = Signal(dict)
    localization_pose_received = Signal(dict)


def _localization_status_failure(detail: str) -> dict:
    """The same shape as a parsed status, so every consumer can read the
    same keys without asking whether this one came off the wire intact."""
    return {"state": "OFF", "seconds_since_ok": None, "source": "",
            "distance_travelled": 0.0, "mount_offset_verified": False,
            "detail": detail}


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
        self._localization_status_topic = None
        self._localization_pose_topic = None

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
    def _parse_status(payload: object) -> dict:
        try:
            status = json.loads(payload)
        except (json.JSONDecodeError, TypeError) as exc:
            return {"state": "failed", "detail": f"bad status JSON: {exc}"}
        if not isinstance(status, dict):
            return {"state": "failed", "detail": "status was not a JSON object"}
        return {"state": status.get("state", "failed"),
                "detail": status.get("detail", "")}

    def subscribe_localization_status(self, topic_name: str = "/localization/status") -> None:
        """The rover's account of its own localisation: OK, SEARCHING or OFF,
        with the seconds since it was last OK.

        JSON in a std_msgs/String, the same convention as /video_status, so
        rosbridge needs no custom message type to discover. Published at 2 Hz
        and on every state change."""
        topic = self._topic_factory(self._ros, topic_name, "std_msgs/String")
        topic.subscribe(lambda msg: self.signals.localization_status_received.emit(
            self._parse_localization_status(msg.get("data", ""))))
        self._localization_status_topic = topic

    def subscribe_localization_pose(
            self, topic_name: str = "/localization/pose",
            throttle_ms: int = LOCALIZATION_POSE_THROTTLE_MS) -> None:
        """The rover's pose, for the header readout only.

        The Gazebo view gets its pose over DDS through sim_bridge, not
        through here - the ground station has no ROS and is not in that
        path. All this subscription feeds is three numbers in the header, so
        it is throttled server-side to 5 Hz."""
        topic = self._topic_factory(self._ros, topic_name, "nav_msgs/Odometry",
                                    throttle_rate=throttle_ms)
        topic.subscribe(lambda msg: self.signals.localization_pose_received.emit(
            pose_readout_from_odometry(msg)))
        self._localization_pose_topic = topic

    @staticmethod
    def _parse_localization_status(payload: object) -> dict:
        """Every field the status JSON carries, with defaults for all of
        them. A payload that will not parse becomes OFF with the reason
        attached: OFF is what the panel shows when nothing can be trusted,
        and losing the reason would send the operator looking at the ZED
        when the fault is in the link."""
        try:
            status = json.loads(payload)
        except (json.JSONDecodeError, TypeError) as exc:
            return _localization_status_failure(f"bad status JSON: {exc}")
        if not isinstance(status, dict):
            return _localization_status_failure("status was not a JSON object")
        seconds = status.get("seconds_since_ok")
        return {
            "state": str(status.get("state", "OFF")),
            "seconds_since_ok": seconds if isinstance(seconds, (int, float)) else None,
            "source": str(status.get("source", "")),
            "distance_travelled": float(status.get("distance_travelled", 0.0) or 0.0),
            "mount_offset_verified": bool(status.get("mount_offset_verified", False)),
            "detail": "",
        }

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
