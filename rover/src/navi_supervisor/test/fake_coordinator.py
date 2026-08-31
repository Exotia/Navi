# fake_coordinator.py
"""The primary's side of the NaVi wire, on a plain socket.

This is deliberately a model of `AutoConnection<navi::NaViEP>` -
`__sam__request`, a 500 ms `__sam__ping`, guarded `F` calls, `__sam__release`
on the way out - and not a copy of our own `navi_teleop.msgpack_rpc` client.
It is what the tests assert the server against, so when the primary changes,
this file is where the change is written down.

`send_raw`/`read_frame` exist for the hostile-input cases: they put bytes on
the socket that no well-behaved client would ever send.
"""

import socket

import msgpack

REQUEST_TYPE = 0
NOTIFY_TYPE = 2


class RpcError(Exception):
    def __init__(self, error):
        super().__init__(f"rpc error: {error!r}")
        self.error = error


class FakeCoordinator:
    def __init__(self, host, port, timeout_s=2.0):
        self._msgid = 0
        self._unpacker = msgpack.Unpacker(raw=False)
        self._sock = socket.create_connection((host, port), timeout=timeout_s)
        self._sock.settimeout(timeout_s)

    # --- the calls the primary makes ------------------------------------
    def call(self, method, *args):
        self._msgid += 1
        msgid = self._msgid
        self.send_raw(msgpack.packb([REQUEST_TYPE, msgid, method, list(args)],
                                    use_bin_type=True))
        while True:
            frame = self.read_frame()
            if frame[1] != msgid:
                continue
            _type, _id, error, result = frame
            if error is not None:
                raise RpcError(error)
            return result

    def notify(self, method, *args):
        self.send_raw(msgpack.packb([NOTIFY_TYPE, method, list(args)],
                                    use_bin_type=True))

    def access(self):
        return bool(self.call("__sam__request"))

    def force(self):
        return self.call("__sam__force")

    def ping(self):
        return bool(self.call("__sam__ping"))

    def release(self):
        return bool(self.call("__sam__release"))

    # --- the raw socket, for the cases a real client never produces -----
    def send_raw(self, payload):
        self._sock.sendall(payload)

    def read_frame(self):
        while True:
            for msg in self._unpacker:
                return msg
            data = self._sock.recv(4096)
            if not data:
                raise ConnectionError("server closed the connection")
            self._unpacker.feed(data)

    def half_close(self):
        """FIN our way, the reply still owed. A real client never does this;
        a killed one does it all the time."""
        self._sock.shutdown(socket.SHUT_WR)

    def closed_by_server(self):
        """True when the server has hung up on us (used by hostile input)."""
        try:
            while True:
                data = self._sock.recv(4096)
                if not data:
                    return True
        except socket.timeout:
            return False
        except OSError:
            return True

    def close(self):
        try:
            self._sock.close()
        except OSError:
            pass
