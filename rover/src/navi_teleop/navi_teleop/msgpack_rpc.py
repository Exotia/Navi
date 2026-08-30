"""A minimal msgpack-RPC client for the rover's rpclib servers.

rpclib frames a request as [0, msgid, method, [args]] and a response as
[1, msgid, error, result]; error is nil on success. This is the only wire
this project speaks to the primary, so it is kept dependency-light (just
`msgpack`) and free of any ROS import, so bema_session and the tests can
use it with no executor running.
"""

import socket

import msgpack

REQUEST_TYPE = 0
RESPONSE_TYPE = 1


class RpcError(Exception):
    def __init__(self, error):
        super().__init__(f"rpc error: {error!r}")
        self.error = error


class RpcTimeout(Exception):
    pass


class RpcDisconnected(Exception):
    pass


class RpcClient:
    def __init__(self, host: str, port: int, timeout_s: float = 1.0):
        self._host = host
        self._port = port
        self._timeout_s = timeout_s
        self._msgid = 0
        self._unpacker = msgpack.Unpacker(raw=False)
        self._sock = socket.create_connection((host, port), timeout=timeout_s)
        self._sock.settimeout(timeout_s)

    def call(self, method: str, *args):
        self._msgid = (self._msgid + 1) & 0xFFFFFFFF
        msgid = self._msgid
        payload = msgpack.packb([REQUEST_TYPE, msgid, method, list(args)],
                                use_bin_type=True)
        try:
            self._sock.sendall(payload)
            return self._read_response(msgid)
        except socket.timeout as exc:
            raise RpcTimeout(f"{method} timed out") from exc
        except (OSError, ValueError) as exc:
            raise RpcDisconnected(f"{method}: {exc}") from exc

    def _read_response(self, msgid: int):
        while True:
            for msg in self._unpacker:
                _type, rid, error, result = msg
                if rid != msgid:
                    continue
                if error is not None:
                    raise RpcError(error)
                return result
            data = self._sock.recv(4096)
            if not data:
                raise RpcDisconnected("server closed the connection")
            self._unpacker.feed(data)

    def close(self):
        try:
            self._sock.close()
        except OSError:
            pass
