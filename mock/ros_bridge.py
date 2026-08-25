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
import json

from autobahn.twisted.websocket import WebSocketServerFactory, WebSocketServerProtocol
from twisted.internet import reactor

MOCK_NODE_NAMES = ["/rosbridge_websocket", "/rosapi"]


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
        outgoing = json.dumps({"op": "publish", "topic": topic, "msg": msg}).encode("utf8")
        for client in self.factory.clients:
            client.sendMessage(outgoing)

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


def main() -> None:
    parser = argparse.ArgumentParser(description="Mock rosbridge server for local testing")
    parser.add_argument("--port", type=int, default=9090, help="port to listen on (default 9090)")
    args = parser.parse_args()

    factory = MockRosBridgeFactory(f"ws://0.0.0.0:{args.port}")
    reactor.listenTCP(args.port, factory)
    print(f"[mock-rosbridge] listening on ws://localhost:{args.port} - connect the ground station to \"localhost\"")
    reactor.run()


if __name__ == "__main__":
    main()
