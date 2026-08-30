import socket
import threading

import msgpack
import pytest

from navi_teleop.msgpack_rpc import RpcClient, RpcError, RpcDisconnected


def _one_shot_server(handler):
    """A server that accepts one connection and answers each request with
    handler(method, args) -> (error, result). Returns (host, port, stop)."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    host, port = srv.getsockname()

    def run():
        conn, _ = srv.accept()
        unpacker = msgpack.Unpacker(raw=False)
        while True:
            data = conn.recv(4096)
            if not data:
                break
            unpacker.feed(data)
            for msg in unpacker:
                _type, msgid, method, args = msg
                error, result = handler(method, list(args))
                conn.sendall(msgpack.packb([1, msgid, error, result], use_bin_type=True))
        conn.close()
        srv.close()

    t = threading.Thread(target=run, daemon=True)
    t.start()
    return host, port, t


def test_call_sends_request_and_returns_result():
    calls = []

    def handler(method, args):
        calls.append((method, args))
        return None, True

    host, port, _ = _one_shot_server(handler)
    client = RpcClient(host, port)
    assert client.call("__sam__request") is True
    assert client.call("F1", 0.1, 0.2, 5.0) is True
    assert calls == [("__sam__request", []), ("F1", [pytest.approx(0.1),
                                                     pytest.approx(0.2),
                                                     pytest.approx(5.0)])]
    client.close()


def test_non_nil_error_raises_rpc_error():
    host, port, _ = _one_shot_server(lambda m, a: (1, None))
    client = RpcClient(host, port)
    with pytest.raises(RpcError) as exc:
        client.call("F1", 0.0, 0.0, 0.0)
    assert exc.value.error == 1
    client.close()


def test_server_closing_raises_disconnected():
    host, port, _ = _one_shot_server(lambda m, a: (None, None))
    client = RpcClient(host, port)
    client.call("F8")            # server answers then loops; force a close
    client._sock.close()         # simulate a dropped link on our side
    with pytest.raises((RpcDisconnected, OSError)):
        client.call("F8")
    client.close()
