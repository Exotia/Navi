#!/usr/bin/env python3
"""Mock rosbridge server for local ground-station testing without ROS2,
rosbridge_suite, or the Jetson.

Speaks just enough of the rosbridge v2 JSON protocol for the ground
station to connect, advertise/publish/subscribe on topics, and call
/rosapi/nodes (used for the System Nodes panel). It is deliberately not a
faithful rosbridge implementation - e.g. it broadcasts every "publish" to
every connected client rather than routing by actual subscriptions, which
is fine for the single-client (one ground station) case this exists for.

Run: python3 mock/ros_bridge.py [--port 9090]

Requires autobahn + twisted, which are already installed as roslibpy's
own dependencies.
"""

import argparse
import importlib.util
import json
import pathlib
import time

from autobahn.twisted.websocket import WebSocketServerFactory, WebSocketServerProtocol
from twisted.internet import reactor
from twisted.internet.task import LoopingCall

MOCK_NODE_NAMES = ["/rosbridge_websocket", "/rosapi", "/localization_status"]

# Loaded by path for the same reason tests/test_square_walk.py does it:
# mock/ is not a package, and this file is run as a script from the repo
# root by start_ground_station.sh.
_SQUARE_WALK = pathlib.Path(__file__).resolve().parent / "square_walk.py"
_spec = importlib.util.spec_from_file_location("square_walk", _SQUARE_WALK)
square_walk = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(square_walk)

POSE_HZ = 10.0      # the ground station throttles to 5 Hz anyway
STATUS_HZ = 2.0     # what localization_status publishes at


class MockRosBridgeProtocol(WebSocketServerProtocol):
    def onOpen(self):
        self.factory.clients.add(self)
        print(f"[mock-rosbridge] client connected ({self.peer})")

    def onClose(self, wasClean, code, reason):
        self.factory.clients.discard(self)
        print(f"[mock-rosbridge] client disconnected ({self.peer})")

    def onMessage(self, payload, isBinary):
        if isBinary:
            return
        try:
            message = json.loads(payload.decode("utf8"))
        except json.JSONDecodeError:
            print(f"[mock-rosbridge] ignoring non-JSON message: {payload!r}")
            return

        op = message.get("op")
        if op == "advertise":
            print(f"[mock-rosbridge] advertise {message.get('topic')}")
        elif op == "subscribe":
            print(f"[mock-rosbridge] subscribe {message.get('topic')}")
        elif op == "publish":
            self._handle_publish(message)
        elif op == "call_service":
            self._handle_call_service(message)
        else:
            print(f"[mock-rosbridge] unhandled op: {op}")

    def _handle_publish(self, message):
        topic = message.get("topic")
        msg = message.get("msg")
        print(f"[mock-rosbridge] publish {topic}: {msg}")
        self.factory.broadcast(topic, msg)

    def _handle_call_service(self, message):
        service = message.get("service")
        request_id = message.get("id")
        if service == "/rosapi/nodes":
            values = {"nodes": MOCK_NODE_NAMES}
        else:
            print(f"[mock-rosbridge] unhandled service: {service}")
            values = {}
        response = json.dumps({
            "op": "service_response",
            "id": request_id,
            "service": service,
            "values": values,
            "result": True,
        }).encode("utf8")
        self.sendMessage(response)


class MockRosBridgeFactory(WebSocketServerFactory):
    protocol = MockRosBridgeProtocol

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.clients = set()

    def broadcast(self, topic, msg):
        """Sends one message to every connected client. Deliberately not
        routed by subscription: this server has one client (the ground
        station) and pretending otherwise would be more code than the thing
        it stands in for."""
        outgoing = json.dumps({"op": "publish", "topic": topic, "msg": msg}).encode("utf8")
        for client in self.clients:
            client.sendMessage(outgoing)


def main() -> None:
    parser = argparse.ArgumentParser(description="Mock rosbridge server for local testing")
    parser.add_argument("--port", type=int, default=9090, help="port to listen on (default 9090)")
    parser.add_argument(
        "--localization-state", choices=["OK", "SEARCHING", "OFF"], default="OK",
        help=(
            "what the fake /localization/status reports (default OK). "
            "SEARCHING freezes the pose and counts the seconds, exactly as the "
            "rover does; OFF publishes no pose at all. Between the three, every "
            "marker the video panel can show is reachable without a rover."))
    parser.add_argument("--square-side", type=float, default=4.0,
                        help="side of the square the fake pose walks, in metres (default 4)")
    parser.add_argument("--square-speed", type=float, default=0.5,
                        help="how fast it walks it, in m/s (default 0.5)")
    args = parser.parse_args()

    factory = MockRosBridgeFactory(f"ws://0.0.0.0:{args.port}")
    reactor.listenTCP(args.port, factory)

    started = time.monotonic()
    # Frozen while SEARCHING, matching the rover: localization_status keeps
    # publishing the last good pose with its stamp frozen rather than
    # extrapolating, and a mock that kept walking would make the panel's
    # SEARCHING marker look like a cosmetic warning over a healthy picture.
    frozen = {"at": None}

    def publish_pose():
        if args.localization_state == "OFF":
            return
        if args.localization_state == "SEARCHING":
            if frozen["at"] is None:
                frozen["at"] = time.monotonic() - started
            elapsed = frozen["at"]
        else:
            elapsed = time.monotonic() - started
        x, y, yaw = square_walk.square_pose(elapsed, args.square_side, args.square_speed)
        factory.broadcast("/localization/pose",
                          square_walk.odometry_message(x, y, yaw))

    def publish_status():
        elapsed = time.monotonic() - started
        seconds_since_ok = elapsed if args.localization_state == "SEARCHING" else (
            0.0 if args.localization_state == "OK" else None)
        factory.broadcast("/localization/status", {"data": square_walk.status_payload(
            args.localization_state, seconds_since_ok,
            args.square_speed * elapsed if args.localization_state == "OK" else 0.0)})

    LoopingCall(publish_pose).start(1.0 / POSE_HZ, now=False)
    LoopingCall(publish_status).start(1.0 / STATUS_HZ, now=False)

    print(f"[mock-rosbridge] listening on ws://localhost:{args.port} - connect the ground station to \"localhost\"")
    print(f"[mock-rosbridge] fake /localization/pose walking a "
          f"{args.square_side} m square at {args.square_speed} m/s, "
          f"status {args.localization_state}")
    reactor.run()


if __name__ == "__main__":
    main()
