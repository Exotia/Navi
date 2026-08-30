# fake_bema_server.py
"""A stand-in for the primary's BEMA (:21022) and coordinator (:21031)
rpclib servers, speaking the same msgpack-RPC wire. Records every call so
tests can assert on the lease dance, the deadman, and unit conversion
without the rover. Run as a script with --forward-twist to drive the
Gazebo rover: each F1 becomes a Twist on /sim_test_twist.
"""

import socket
import threading

import msgpack


class _Server:
    def __init__(self, port, dispatch):
        self._dispatch = dispatch
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("127.0.0.1", port))
        self._srv.listen(4)
        self._srv.settimeout(0.2)
        self.port = self._srv.getsockname()[1]
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def _run(self):
        while not self._stop:
            try:
                conn, _ = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            conn.settimeout(0.2)
            threading.Thread(target=self._serve, args=(conn,), daemon=True).start()

    def _serve(self, conn):
        unpacker = msgpack.Unpacker(raw=False)
        while not self._stop:
            try:
                data = conn.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data:
                break
            unpacker.feed(data)
            for msg in unpacker:
                _type, msgid, method, args = msg
                error, result = self._dispatch(method, list(args))
                conn.sendall(msgpack.packb([1, msgid, error, result],
                                           use_bin_type=True))
        conn.close()

    def stop(self):
        self._stop = True
        try:
            self._srv.close()
        except OSError:
            pass


class FakeBemaServer:
    def __init__(self, bema_port=0, coordinator_port=0, on_drive=None):
        self.calls = []
        self.state = 1                 # Idle
        self.movement_enabled = False
        self.lease_held = False
        self.coord_lease_held = False
        self._on_drive = on_drive or (lambda vx, vy, w: None)
        self._bema = _Server(bema_port, self._bema_dispatch)
        self._coord = _Server(coordinator_port, self._coord_dispatch)
        self.bema_port = self._bema.port
        self.coordinator_port = self._coord.port

    def start(self):
        self._bema.start()
        self._coord.start()

    def stop(self):
        self._bema.stop()
        self._coord.stop()

    def set_state(self, value):
        self.state = value

    def _bema_dispatch(self, method, args):
        self.calls.append(("bema", method, args))
        if method == "__sam__request":
            self.lease_held = True
            return None, True
        if method == "__sam__ping":
            return None, self.lease_held
        if method == "__sam__release":
            self.lease_held = False
            return None, None
        if method == "__sam__force":
            self.lease_held = True
            return None, None
        if not self.lease_held and method != "F8":
            return 1, None             # rpc_base error when not the holder
        if method == "F1":
            self._on_drive(*args)
        if method == "F7":
            self.movement_enabled = bool(args[0])
        return None, None

    def _coord_dispatch(self, method, args):
        self.calls.append(("coord", method, args))
        if method == "__sam__request":
            self.coord_lease_held = True
            return None, True
        if method == "__sam__ping":
            return None, self.coord_lease_held
        if method == "__sam__release":
            self.coord_lease_held = False
            return None, None
        if method == "F9":             # getState - unguarded on the real proxy
            return None, self.state
        if method == "F10":            # notifyConnected - unguarded
            return None, None
        # F0-F6 sit behind checkAccess() on the real CoordinatorProxy: an
        # un-leased call is answered with error 1 and the handler never runs.
        if not self.coord_lease_held:
            return 1, None
        if method == "F6":             # startManual
            self.state = 3             # Manual (simplified: no 5 s delay)
            return None, None
        return None, None


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bema-port", type=int, default=21022)
    parser.add_argument("--coordinator-port", type=int, default=21031)
    parser.add_argument("--forward-twist", action="store_true",
                        help="republish each F1 as a Twist on /sim_test_twist")
    args = parser.parse_args()

    on_drive = None
    node = None
    if args.forward_twist:
        import rclpy
        from geometry_msgs.msg import Twist
        from math import radians
        rclpy.init()
        node = rclpy.create_node("fake_bema_forward")
        pub = node.create_publisher(Twist, "/sim_test_twist", 1)

        def on_drive(vx, vy, w_deg):
            t = Twist()
            t.linear.x, t.linear.y = vx, vy
            t.angular.z = -radians(w_deg)     # invert the bridge's negation
            pub.publish(t)

    server = FakeBemaServer(args.bema_port, args.coordinator_port, on_drive=on_drive)
    server.state = 3                          # Manual, so nothing gates F1
    server.start()
    print(f"fake BEMA on :{server.bema_port}, coordinator on :{server.coordinator_port}")
    try:
        if node is not None:
            rclpy.spin(node)
        else:
            import time
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == "__main__":
    main()
