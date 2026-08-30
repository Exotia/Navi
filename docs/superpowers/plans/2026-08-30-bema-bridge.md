# BEMA bridge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A gamepad twist from the Navi ground station reaches the rover's real IK and drives the wheels, with a 1 s deadman that lives on the Orin's LAN.

**Architecture:** A new Python node `bema_bridge` in `rover/src/navi_teleop` subscribes `/manual_twist` and, over msgpack-RPC (the `msgpack` package, no rclpy in the client layer), calls the Pi's BEMA server (`:21022`, `F1` drive at 20 Hz under an access lease) and coordinator (`:21031`, `notifyConnected` heartbeat + `startManual`). A DRIVE row in the ground station publishes `/drive_command` and shows `/drive_status`. Nothing on the Pi or jetson changes.

**Tech Stack:** Python 3.10, `msgpack`, rclpy (Humble), PySide6, pytest.

**Spec:** `docs/superpowers/specs/2026-08-30-bema-bridge-design.md`

## Global Constraints

- No `rclpy` anywhere under `ground_station/`; `models.py` imports neither Qt nor ROS.
- Nothing on `a_primary` or `jetson` is created, modified, started or stopped. This plan only edits files under `rover/` and `ground_station/` in this repo.
- Never publish to `/manual_twist` from a test; node tests set a throwaway `ROS_DOMAIN_ID` in the fixture (not inherited from the ambient environment).
- The bridge never calls `__sam__force`. Start-up sends nothing that moves the wheels (`F0` init, `F4` reset-encoders, `startManual` only on explicit `/drive_command`).
- Commits use `git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de"`, message trailer:
  ```
  Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01JKPAhQayr2XFS87SjxAwzJ
  ```
  Never push.

### Protocol reference (verified against `bemacontroller/deps/rpclib` and the Pi's interface headers)

- **Wire framing (rpclib msgpack-RPC):** request is a 4-element msgpack array `[0, msgid, method, [args...]]` (0 = request type, `msgid` a uint that increments). Response is `[1, msgid, error, result]` (1 = response type). `error` is nil on success; non-nil means the call raised. Notifications (type 2) are not used here.
- **BEMA server `:21022`** (method names are the literal strings): `F0`=init, `F1(vX,vY,w)`=drive, `F2`=stopMovement, `F3`=resetOdometry, `F4`=resetEncoders, `F5`=changeDriveMode, `F6`=changeDriveState, `F7(bool)`=setMovementEnabled, `F8`=fetchOdometry. Lease methods on the same connection: `__sam__request`→bool, `__sam__ping`→bool, `__sam__release`, `__sam__force`. Any `F*` except `F8` needs the lease; a guarded call while not the holder responds with error `1`.
- **Coordinator server `:21031`:** `F6`=startManual, `F9`=getState→int, `F10`=notifyConnected. `State` enum ints: `Disconnected=0, Idle=1, PrepareManual=2, Manual=3, PrepareAutonomous=4, Autonomous=5, Waiting=6`.
- **Units:** `F1` takes vX,vY in m/s (rover frame) and **w in deg/s**. Bridge maps `vX=linear.x`, `vY=linear.y`, `w = -degrees(angular.z)`. The sign is verified on the sim and on blocks before free driving; if wrong, drop the negation in `_twist_to_drive` only.

---

## File structure

- Create `rover/src/navi_teleop/navi_teleop/msgpack_rpc.py` — pure TCP msgpack-RPC client. No ROS, no project imports.
- Create `rover/src/navi_teleop/navi_teleop/bema_session.py` — protocol/session over two `RpcClient`s. No ROS.
- Create `rover/src/navi_teleop/navi_teleop/bema_bridge.py` — the rclpy node.
- Create `rover/src/navi_teleop/test/fake_bema_server.py` — threaded msgpack-RPC server for tests and the sim bench.
- Create `rover/src/navi_teleop/test/test_msgpack_rpc.py`, `test_bema_session.py`, `test_bema_bridge.py`.
- Modify `rover/src/navi_teleop/setup.py` — add the `bema_bridge` console script.
- Modify `rover/src/navi_teleop/package.xml` — add `std_msgs` and a note on the `msgpack` pip dep.
- Modify `rover/start_navi.sh` — start the node, add its stale-cleanup pattern.
- Create `ground_station/ui/drive_row.py` — the DRIVE row widget.
- Modify `ground_station/models.py` — `DriveState` + `parse_drive_status` + `drive_command_json`.
- Modify `ground_station/ros_client.py` — `subscribe_drive_status`, `send_drive_command`, a new signal.
- Modify `ground_station/ui/dashboard_page.py`, `ground_station/ui/main_window.py` — place and wire the row.
- Create `ground_station`/`tests/test_drive_row.py` and add cases to the models test.

---

### Task 1: msgpack-RPC client

**Files:**
- Create: `rover/src/navi_teleop/navi_teleop/msgpack_rpc.py`
- Test: `rover/src/navi_teleop/test/test_msgpack_rpc.py`

**Interfaces:**
- Produces: `class RpcClient(host: str, port: int, timeout_s: float = 1.0)` with `call(method: str, *args) -> object`, `close() -> None`. Exceptions `RpcError(Exception)` (carries `.error`), `RpcTimeout(Exception)`, `RpcDisconnected(Exception)`. A module-level helper `_serve_one(sock, handler)` is NOT part of this task — the test uses a tiny inline server.

- [ ] **Step 1: Write the failing test**

```python
# test_msgpack_rpc.py
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
```

- [ ] **Step 2: Run to verify it fails**

Run: `bash -c 'source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_teleop:$PYTHONPATH python3 -m pytest rover/src/navi_teleop/test/test_msgpack_rpc.py -q'`
Expected: FAIL — `ModuleNotFoundError: navi_teleop.msgpack_rpc`. (If `msgpack` itself is missing, `pip install msgpack` first — this is the dep the spec calls out.)

- [ ] **Step 3: Implement**

```python
# msgpack_rpc.py
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
```

- [ ] **Step 4: Run to verify it passes**

Run the Step 2 command. Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
git add rover/src/navi_teleop/navi_teleop/msgpack_rpc.py rover/src/navi_teleop/test/test_msgpack_rpc.py
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "navi_teleop: a minimal msgpack-RPC client for the primary's rpclib servers

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01JKPAhQayr2XFS87SjxAwzJ"
```

---

### Task 2: fake BEMA + coordinator server (test double)

**Files:**
- Create: `rover/src/navi_teleop/test/fake_bema_server.py`
- Test: covered by its use in Tasks 3 and 5; add a self-check test at the bottom of `test_bema_session.py` in Task 3. This task's own gate is that it imports and serves.

**Interfaces:**
- Produces: `class FakeBemaServer(bema_port=0, coordinator_port=0)` with attributes `bema_port`, `coordinator_port` (actual bound ports), `calls: list[tuple[str, str, list]]` (server-tag, method, args), `state: int` (coordinator state, default 1=Idle), `movement_enabled: bool`, `lease_held: bool`; methods `start()`, `stop()`, and `set_state(int)`. When `--forward-twist` is used as a script it republishes `F1` to `/sim_test_twist`; the class itself has a hook `on_drive(vx, vy, w)` defaulting to a no-op.

- [ ] **Step 1: Write it (no separate test; exercised next task)**

```python
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
        self.port = self._srv.getsockname()[1]
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def _run(self):
        while not self._stop:
            try:
                self._srv.settimeout(0.2)
                conn, _ = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(target=self._serve, args=(conn,), daemon=True).start()

    def _serve(self, conn):
        unpacker = msgpack.Unpacker(raw=False)
        while not self._stop:
            try:
                data = conn.recv(4096)
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
        if method == "F9":             # getState
            return None, self.state
        if method == "F6":             # startManual
            self.state = 3             # Manual (simplified: no 5 s delay)
            return None, None
        if method == "F10":            # notifyConnected
            return None, None
        return None, None
```

- [ ] **Step 2: Commit**

```bash
git add rover/src/navi_teleop/test/fake_bema_server.py
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "navi_teleop tests: a fake BEMA + coordinator rpclib server

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01JKPAhQayr2XFS87SjxAwzJ"
```

---

### Task 3: bema_session (protocol + reconnect)

**Files:**
- Create: `rover/src/navi_teleop/navi_teleop/bema_session.py`
- Test: `rover/src/navi_teleop/test/test_bema_session.py`

**Interfaces:**
- Consumes: `RpcClient`, `RpcError`, `RpcTimeout`, `RpcDisconnected` (Task 1); `FakeBemaServer` (Task 2).
- Produces: `class BemaSession(host, bema_port, coordinator_port, clock, client_factory=RpcClient)`. Methods: `connect() -> None`, `tick(now: float) -> None`, `set_command(vx, vy, w_deg) -> None`, `stop() -> None`, `init()`, `reset_encoders()`, `reset_odometry()`, `change_drive_mode()`, `change_drive_state()`, `start_manual()`, `close() -> None`, `status() -> dict` with keys `connected, lease, coordinator_state, last_error, reconnect_in_s`. `PING_INTERVAL_S = 0.5`, `HEARTBEAT_INTERVAL_S = 1.0`. `client_factory` is injected so tests pass a factory that connects to the fake server's ports.

- [ ] **Step 1: Write the failing tests**

```python
# test_bema_session.py
import pytest

from navi_teleop.bema_session import BemaSession
from navi_teleop.msgpack_rpc import RpcClient
from fake_bema_server import FakeBemaServer


class Clock:
    def __init__(self):
        self.t = 0.0
    def __call__(self):
        return self.t


@pytest.fixture
def server():
    s = FakeBemaServer()
    s.start()
    yield s
    s.stop()


def _session(server, clock):
    def factory(host, port, timeout_s=1.0):
        return RpcClient("127.0.0.1", port, timeout_s)
    # host is unused because the fake binds loopback; ports select the server
    sess = BemaSession("127.0.0.1", server.bema_port, server.coordinator_port,
                       clock=clock, client_factory=factory)
    sess.connect()
    return sess


def test_connect_takes_the_lease(server):
    clock = Clock()
    sess = _session(server, clock)
    assert sess.status()["lease"] is True
    assert ("bema", "__sam__request", []) in server.calls


def test_tick_sends_drive_and_paces_ping_and_heartbeat(server):
    clock = Clock()
    sess = _session(server, clock)
    sess.set_command(0.1, 0.0, 5.0)
    sess.tick(clock.t)                       # t=0: drive + first ping + first hb
    clock.t = 0.2
    sess.tick(clock.t)                       # 0.2s: drive only
    clock.t = 0.6
    sess.tick(clock.t)                       # 0.6s: drive + ping (>=0.5)
    methods = [m for tag, m, a in server.calls if tag == "bema"]
    assert methods.count("F1") == 3
    assert methods.count("__sam__ping") == 2   # t=0 and t=0.6
    coord = [m for tag, m, a in server.calls if tag == "coord"]
    assert coord.count("F10") == 1             # heartbeat once in [0,0.6)


def test_drive_carries_the_command_verbatim(server):
    clock = Clock()
    sess = _session(server, clock)
    sess.set_command(0.25, -0.1, -12.0)
    sess.tick(clock.t)
    f1 = [a for tag, m, a in server.calls if m == "F1"][0]
    assert f1 == [pytest.approx(0.25), pytest.approx(-0.1), pytest.approx(-12.0)]


def test_stop_sends_zero_then_stopmovement(server):
    clock = Clock()
    sess = _session(server, clock)
    sess.stop()
    seq = [(m, a) for tag, m, a in server.calls if tag == "bema"
           and m in ("F1", "F2")]
    assert seq[0] == ("F1", [0.0, 0.0, 0.0])
    assert seq[1] == ("F2", [])


def test_reconnect_after_the_link_drops(server):
    clock = Clock()
    sess = _session(server, clock)
    sess.tick(clock.t)
    # drop the server; next tick must mark down and schedule a reconnect
    server.stop()
    clock.t = 1.0
    sess.tick(clock.t)
    assert sess.status()["connected"] is False
    # bring a new server up on the same ports is not possible here; assert
    # the session backed off rather than raising
    assert sess.status()["reconnect_in_s"] >= 1.0


def test_start_manual_calls_the_coordinator(server):
    clock = Clock()
    sess = _session(server, clock)
    sess.start_manual()
    assert ("coord", "F6", []) in server.calls
    assert server.state == 3
```

- [ ] **Step 2: Run to verify it fails**

Run: `bash -c 'source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_teleop:$PWD/rover/src/navi_teleop/test:$PYTHONPATH python3 -m pytest rover/src/navi_teleop/test/test_bema_session.py -q'`
Expected: FAIL — `ModuleNotFoundError: navi_teleop.bema_session`.

- [ ] **Step 3: Implement**

```python
# bema_session.py
"""The BEMA drive session: an rpclib client to the primary's drive server
(:21022) and coordinator (:21031), with the lease dance, the coordinator
heartbeat, and reconnect-with-backoff. No ROS here - the node in
bema_bridge.py owns the timers and calls tick(); this class owns the
protocol, so it can be tested against fake_bema_server with a fake clock.
"""

from navi_teleop.msgpack_rpc import (RpcClient, RpcError, RpcTimeout,
                                     RpcDisconnected)

PING_INTERVAL_S = 0.5
HEARTBEAT_INTERVAL_S = 1.0
_BACKOFF = [1.0, 2.0, 4.0, 5.0]


class BemaSession:
    def __init__(self, host, bema_port, coordinator_port, clock,
                 client_factory=RpcClient):
        self._host = host
        self._bema_port = bema_port
        self._coord_port = coordinator_port
        self._clock = clock
        self._factory = client_factory
        self._bema = None
        self._coord = None
        self._lease = False
        self._command = (0.0, 0.0, 0.0)
        self._last_ping = None
        self._last_hb = None
        self._last_error = None
        self._coord_state = None
        self._down_since = None
        self._backoff_index = 0
        self._retry_at = None

    # --- connection lifecycle -------------------------------------------
    def connect(self):
        try:
            self._bema = self._factory(self._host, self._bema_port)
            self._coord = self._factory(self._host, self._coord_port)
            self._lease = bool(self._bema.call("__sam__request"))
            self._last_error = None
            self._down_since = None
            self._backoff_index = 0
            self._retry_at = None
        except (RpcError, RpcTimeout, RpcDisconnected, OSError) as exc:
            self._mark_down(exc)

    def _mark_down(self, exc):
        self._last_error = str(exc)
        self._lease = False
        if self._down_since is None:
            self._down_since = self._clock()
        delay = _BACKOFF[min(self._backoff_index, len(_BACKOFF) - 1)]
        self._retry_at = self._clock() + delay
        self._backoff_index += 1
        for client in (self._bema, self._coord):
            if client is not None:
                client.close()
        self._bema = self._coord = None

    # --- per-tick work ---------------------------------------------------
    def tick(self, now):
        if self._bema is None:
            if self._retry_at is not None and now >= self._retry_at:
                self.connect()
            return
        try:
            if not self._lease:
                self._lease = bool(self._bema.call("__sam__request"))
            if self._last_ping is None or now - self._last_ping >= PING_INTERVAL_S:
                if not self._bema.call("__sam__ping"):
                    self._lease = bool(self._bema.call("__sam__request"))
                self._last_ping = now
            if self._last_hb is None or now - self._last_hb >= HEARTBEAT_INTERVAL_S:
                self._coord.call("F10")               # notifyConnected
                self._coord_state = self._coord.call("F9")   # getState
                self._last_hb = now
            vx, vy, w = self._command
            self._bema.call("F1", vx, vy, w)
        except (RpcError, RpcTimeout, RpcDisconnected, OSError) as exc:
            self._mark_down(exc)

    # --- commands --------------------------------------------------------
    def set_command(self, vx, vy, w_deg):
        self._command = (float(vx), float(vy), float(w_deg))

    def stop(self):
        self._command = (0.0, 0.0, 0.0)
        self._safe("F1", 0.0, 0.0, 0.0)
        self._safe("F2")

    def init(self):
        self._safe("F0")

    def reset_encoders(self):
        self._safe("F4")

    def reset_odometry(self):
        self._safe("F3")

    def change_drive_mode(self):
        self._safe("F5")

    def change_drive_state(self):
        self._safe("F6")

    def start_manual(self):
        self._safe_coord("F6")

    def _safe(self, method, *args):
        if self._bema is None:
            return
        try:
            self._bema.call(method, *args)
        except (RpcError, RpcTimeout, RpcDisconnected, OSError) as exc:
            self._mark_down(exc)

    def _safe_coord(self, method, *args):
        if self._coord is None:
            return
        try:
            self._coord.call(method, *args)
        except (RpcError, RpcTimeout, RpcDisconnected, OSError) as exc:
            self._mark_down(exc)

    def close(self):
        if self._bema is not None:
            self.stop()
            try:
                self._bema.call("__sam__release")
            except (RpcError, RpcTimeout, RpcDisconnected, OSError):
                pass
        for client in (self._bema, self._coord):
            if client is not None:
                client.close()
        self._bema = self._coord = None
        self._lease = False

    def status(self):
        now = self._clock()
        reconnect_in = None
        if self._retry_at is not None and self._bema is None:
            reconnect_in = max(0.0, self._retry_at - now)
        return {
            "connected": self._bema is not None,
            "lease": self._lease,
            "coordinator_state": self._coord_state,
            "last_error": self._last_error,
            "reconnect_in_s": reconnect_in,
        }
```

- [ ] **Step 4: Run to verify it passes**

Run the Step 2 command. Expected: PASS (6 tests). If `test_reconnect_after_the_link_drops` needs the ping before the drive to trip the failure, keep the tick order (`__sam__request` → ping → heartbeat → F1) exactly as written; the fake stops accepting mid-tick and the first `call` raises `RpcDisconnected`.

- [ ] **Step 5: Commit**

```bash
git add rover/src/navi_teleop/navi_teleop/bema_session.py rover/src/navi_teleop/test/test_bema_session.py
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "navi_teleop: BEMA session - lease, coordinator heartbeat, reconnect backoff

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01JKPAhQayr2XFS87SjxAwzJ"
```

---

### Task 4: bema_bridge node (deadman, command dispatch, status)

**Files:**
- Create: `rover/src/navi_teleop/navi_teleop/bema_bridge.py`
- Test: `rover/src/navi_teleop/test/test_bema_bridge.py`

**Interfaces:**
- Consumes: `BemaSession` (Task 3), `FakeBemaServer` (Task 2).
- Produces: `class BemaBridge(Node)` constructed as `BemaBridge(session_factory=..., parameter_overrides=[...])`. `session_factory(host, bema_port, coordinator_port, clock)` returns a `BemaSession`. Internal methods used by tests: `_on_twist(msg)`, `_on_command(msg)`, `_drive_tick()`, `_status_tick()`, attribute `_deadman_active: bool`. Parameters: `bema_host` (`192.168.178.26`), `bema_port` (21022), `coordinator_port` (21031), `deadman_s` (1.0), `twist_topic` (`/manual_twist`).

- [ ] **Step 1: Write the failing tests**

```python
# test_bema_bridge.py
import json
import os

os.environ.setdefault("ROS_DOMAIN_ID", "93")   # throwaway; never the rover's

import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from navi_teleop.bema_bridge import BemaBridge
from navi_teleop.bema_session import BemaSession
from navi_teleop.msgpack_rpc import RpcClient
from fake_bema_server import FakeBemaServer


class Clock:
    def __init__(self):
        self.t = 0.0
    def __call__(self):
        return self.t


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def server():
    s = FakeBemaServer()
    s.start()
    yield s
    s.stop()


@pytest.fixture
def bridge(server):
    clock = Clock()

    def session_factory(host, bema_port, coordinator_port, clk):
        def client_factory(h, port, timeout_s=1.0):
            return RpcClient("127.0.0.1", port, timeout_s)
        sess = BemaSession("127.0.0.1", server.bema_port, server.coordinator_port,
                           clock=clock, client_factory=client_factory)
        sess.connect()
        return sess

    node = BemaBridge(session_factory=session_factory, clock=clock,
                      parameter_overrides=[
                          rclpy.parameter.Parameter("deadman_s", value=1.0)])
    node._clock_ref = clock
    yield node, server, clock
    node.destroy_node()


def _twist(x, y, wz):
    t = Twist()
    t.linear.x, t.linear.y, t.angular.z = x, y, wz
    return t


def test_a_fresh_twist_is_forwarded_as_drive(bridge):
    node, server, clock = bridge
    node._on_twist(_twist(0.2, 0.0, 1.0))    # angular.z 1 rad/s -> -57.3 deg/s
    node._drive_tick()
    f1 = [a for tag, m, a in server.calls if m == "F1"][-1]
    assert f1[0] == pytest.approx(0.2)
    assert f1[2] == pytest.approx(-57.2958, abs=1e-3)


def test_deadman_stops_after_the_timeout(bridge):
    node, server, clock = bridge
    node._on_twist(_twist(0.2, 0.0, 0.0))
    node._drive_tick()
    clock.t = 1.5                            # > deadman_s past the twist
    node._drive_tick()
    assert node._deadman_active is True
    seq = [(m, a) for tag, m, a in server.calls if m in ("F1", "F2")]
    assert ("F2", []) in [(m, a) for m, a in seq]
    last_f1 = [a for m, a in seq if m == "F1"][-1]
    assert last_f1 == [0.0, 0.0, 0.0]


def test_stop_command_is_dispatched(bridge):
    node, server, clock = bridge
    node._on_command(_string({"action": "stop"}))
    assert ("bema", "F2", []) in server.calls


def test_init_command_calls_f0(bridge):
    node, server, clock = bridge
    node._on_command(_string({"action": "init"}))
    assert ("bema", "F0", []) in server.calls


def test_manual_command_calls_the_coordinator(bridge):
    node, server, clock = bridge
    node._on_command(_string({"action": "manual"}))
    assert ("coord", "F6", []) in server.calls


def test_status_tick_publishes_json(bridge):
    node, server, clock = bridge
    published = []
    node._status_pub.publish = lambda msg: published.append(msg.data)
    node._on_twist(_twist(0.1, 0.0, 0.0))
    node._drive_tick()
    node._status_tick()
    status = json.loads(published[-1])
    assert status["connected"] is True
    assert "twist_age_s" in status and "deadman_active" in status


def test_a_malformed_command_does_not_raise(bridge):
    node, server, clock = bridge
    node._on_command(_string({"action": "no_such_action"}))
    node._on_command(_bad_string("{not json"))
    # no exception = pass


def _string(payload):
    m = String()
    m.data = json.dumps(payload)
    return m


def _bad_string(raw):
    m = String()
    m.data = raw
    return m
```

- [ ] **Step 2: Run to verify it fails**

Run: `bash -c 'source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_teleop:$PWD/rover/src/navi_teleop/test:$PYTHONPATH python3 -m pytest rover/src/navi_teleop/test/test_bema_bridge.py -q'`
Expected: FAIL — `ModuleNotFoundError: navi_teleop.bema_bridge`.

- [ ] **Step 3: Implement**

```python
# bema_bridge.py
"""The rover-side node that turns /manual_twist into real wheel commands.

It owns the timers; bema_session owns the protocol. A twist is forwarded
to the primary's IK at 20 Hz; if the stream stops for deadman_s the wheels
are zeroed and stopped, and kept stopped until a fresh twist arrives.
/drive_command (JSON) drives the coordinator/BEMA buttons the ground
station shows; /drive_status (JSON, 1 Hz) reports what is happening.

Nothing here calls init() or startManual() on its own - the rover only
moves after the operator presses a button.
"""

import json
from math import degrees
from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from navi_teleop.bema_session import BemaSession

DRIVE_HZ = 20.0
STATUS_HZ = 1.0


def _default_session_factory(host, bema_port, coordinator_port, clock):
    session = BemaSession(host, bema_port, coordinator_port, clock=clock)
    session.connect()
    return session


class BemaBridge(Node):
    def __init__(self, session_factory=_default_session_factory,
                 clock=monotonic, parameter_overrides=None):
        super().__init__("bema_bridge",
                         parameter_overrides=parameter_overrides or [])
        self.declare_parameter("bema_host", "192.168.178.26")
        self.declare_parameter("bema_port", 21022)
        self.declare_parameter("coordinator_port", 21031)
        self.declare_parameter("deadman_s", 1.0)
        self.declare_parameter("twist_topic", "/manual_twist")

        self._clock = clock
        self._deadman_s = float(self.get_parameter("deadman_s").value)
        self._twist = (0.0, 0.0, 0.0)
        self._twist_at = None
        self._deadman_active = True
        self._last_action = None

        self._session = session_factory(
            self.get_parameter("bema_host").value,
            int(self.get_parameter("bema_port").value),
            int(self.get_parameter("coordinator_port").value),
            self._clock)

        self.create_subscription(
            Twist, self.get_parameter("twist_topic").value, self._on_twist, 1)
        self.create_subscription(String, "/drive_command", self._on_command, 10)
        self._status_pub = self.create_publisher(String, "/drive_status", 1)
        self.create_timer(1.0 / DRIVE_HZ, self._drive_tick)
        self.create_timer(1.0 / STATUS_HZ, self._status_tick)

    def _on_twist(self, msg: Twist):
        # w to the IK is degrees/second; the server negates again so a
        # positive angular.z (CCW) reaches the model as positive u.
        self._twist = (msg.linear.x, msg.linear.y, -degrees(msg.angular.z))
        self._twist_at = self._clock()

    def _drive_tick(self):
        try:
            now = self._clock()
            fresh = (self._twist_at is not None
                     and now - self._twist_at <= self._deadman_s)
            if fresh:
                self._deadman_active = False
                self._session.set_command(*self._twist)
                self._session.tick(now)
            else:
                if not self._deadman_active:
                    self._deadman_active = True
                    self._session.stop()
                self._session.set_command(0.0, 0.0, 0.0)
                self._session.tick(now)
        except Exception as exc:                     # never kill the node
            self.get_logger().error(f"drive tick failed: {exc!r}")

    def _on_command(self, msg: String):
        try:
            action = json.loads(msg.data).get("action")
        except (json.JSONDecodeError, TypeError, AttributeError):
            self.get_logger().warn(f"unreadable drive command: {msg.data!r}")
            return
        self._last_action = action
        table = {
            "stop": self._session.stop,
            "manual": self._session.start_manual,
            "init": self._session.init,
            "reset_encoders": self._session.reset_encoders,
            "reset_odometry": self._session.reset_odometry,
            "drive_mode": self._session.change_drive_mode,
            "drive_state": self._session.change_drive_state,
        }
        handler = table.get(action)
        if handler is None:
            self.get_logger().warn(f"unknown drive action: {action!r}")
            return
        try:
            handler()
        except Exception as exc:
            self.get_logger().error(f"drive action {action} failed: {exc!r}")

    def _status_tick(self):
        now = self._clock()
        status = dict(self._session.status())
        status["twist_age_s"] = (None if self._twist_at is None
                                 else round(now - self._twist_at, 2))
        status["deadman_active"] = self._deadman_active
        status["last_action"] = self._last_action
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

    def destroy_node(self):
        try:
            self._session.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = BemaBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run to verify it passes**

Run the Step 2 command. Expected: PASS (7 tests).

- [ ] **Step 5: Commit**

```bash
git add rover/src/navi_teleop/navi_teleop/bema_bridge.py rover/src/navi_teleop/test/test_bema_bridge.py
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "navi_teleop: bema_bridge node - /manual_twist to real wheels with a 1 s deadman

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01JKPAhQayr2XFS87SjxAwzJ"
```

---

### Task 5: package the node and wire start_navi.sh

**Files:**
- Modify: `rover/src/navi_teleop/setup.py` (entry_points), `rover/src/navi_teleop/package.xml` (std_msgs dep + msgpack note)
- Modify: `rover/start_navi.sh` (start the node; add its cleanup pattern)

**Interfaces:**
- Produces: a `bema_bridge` console script; a running node in the launch sequence.

- [ ] **Step 1: Add the console script**

In `rover/src/navi_teleop/setup.py`, add to `console_scripts`:
```python
            'bema_bridge = navi_teleop.bema_bridge:main',
```

- [ ] **Step 2: Declare the deps**

In `rover/src/navi_teleop/package.xml`, after the `sensor_msgs` depend:
```xml
  <depend>std_msgs</depend>
  <!-- bema_bridge needs the `msgpack` Python package (pip install msgpack);
       it is not an ament package, so it is documented here rather than
       declared. -->
```

- [ ] **Step 3: Start it in start_navi.sh**

In `rover/start_navi.sh`, add a stale-cleanup pattern next to the existing teleop one (line ~238), matching the node's process:
```bash
    kill_stale "bema_bridge nodes" "navi_teleop/bema_bridge"
```
And start the node in the background before the final `manual_twist_listener` foreground line (~357), gated so it can be skipped:
```bash
if [ "$START_BRIDGE" -eq 1 ]; then
    echo "starting bema_bridge (idle until the ground station drives)"
    ros2 run navi_teleop bema_bridge &
fi
```
Add `START_BRIDGE=1` near the other flags with a `--no-bridge)` case that sets it to 0, mirroring `--no-video`. Keep `manual_twist_listener` as the foreground process it already is.

- [ ] **Step 4: Verify the script parses and the entry point resolves**

Run: `bash -n rover/start_navi.sh && echo "syntax ok"`
Run: `bash -c 'cd rover && grep -n bema_bridge src/navi_teleop/setup.py'`
Expected: `syntax ok`, and the entry line present. (Full `colcon build` happens on the Orin in Task 8.)

- [ ] **Step 5: Commit**

```bash
git add rover/src/navi_teleop/setup.py rover/src/navi_teleop/package.xml rover/start_navi.sh
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "navi_teleop: install and launch bema_bridge, with its own stale-cleanup and --no-bridge

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01JKPAhQayr2XFS87SjxAwzJ"
```

---

### Task 6: ground station models — DriveState, parser, command JSON

**Files:**
- Modify: `ground_station/models.py`
- Test: `tests/test_models.py` (add cases; create the file if the repo keeps model tests elsewhere — check first with `ls tests/ | grep model`)

**Interfaces:**
- Produces: `@dataclass DriveState(connected: bool, lease: bool, coordinator_state: str | None, deadman_active: bool, twist_age_s: float | None, last_action: str | None, last_error: str | None)`; `parse_drive_status(payload: str) -> DriveState | None`; `drive_command_json(action: str) -> str`. A helper `_coordinator_name(value)` mapping the int state to a name.

- [ ] **Step 1: Write the failing tests**

```python
# in tests/test_models.py
import json

from ground_station.models import (DriveState, parse_drive_status,
                                    drive_command_json)


def test_parse_drive_status_reads_the_fields():
    payload = json.dumps({"connected": True, "lease": True,
                          "coordinator_state": 3, "deadman_active": False,
                          "twist_age_s": 0.1, "last_action": "manual",
                          "last_error": None})
    state = parse_drive_status(payload)
    assert state.connected is True and state.lease is True
    assert state.coordinator_state == "Manual"
    assert state.deadman_active is False and state.twist_age_s == 0.1


def test_parse_drive_status_tolerates_garbage():
    assert parse_drive_status("{not json") is None
    assert parse_drive_status(json.dumps([1, 2])) is None
    # wrong field types fall back, they do not raise
    state = parse_drive_status(json.dumps({"connected": "yes",
                                           "coordinator_state": "nonsense"}))
    assert state.connected is False
    assert state.coordinator_state is None


def test_drive_command_json_round_trips():
    assert json.loads(drive_command_json("stop")) == {"action": "stop"}
```

- [ ] **Step 2: Run to verify it fails**

Run: `QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest tests/test_models.py -q`
Expected: FAIL — import error for the new names.

- [ ] **Step 3: Implement** (append to `ground_station/models.py`, reusing existing `_safe_int`/`_safe_float` helpers — check their names first)

```python
_COORDINATOR_STATES = {
    0: "Disconnected", 1: "Idle", 2: "PrepareManual", 3: "Manual",
    4: "PrepareAutonomous", 5: "Autonomous", 6: "Waiting",
}


def _coordinator_name(value):
    return _COORDINATOR_STATES.get(value) if isinstance(value, int) else None


@dataclass
class DriveState:
    """/drive_status as the Drive row shows it."""
    connected: bool
    lease: bool
    coordinator_state: str | None
    deadman_active: bool
    twist_age_s: float | None
    last_action: str | None
    last_error: str | None


def parse_drive_status(payload: str):
    try:
        status = json.loads(payload)
    except (json.JSONDecodeError, TypeError):
        return None
    if not isinstance(status, dict):
        return None
    age = status.get("twist_age_s")
    return DriveState(
        connected=status.get("connected") is True,
        lease=status.get("lease") is True,
        coordinator_state=_coordinator_name(status.get("coordinator_state")),
        deadman_active=status.get("deadman_active") is True,
        twist_age_s=age if isinstance(age, (int, float)) else None,
        last_action=status.get("last_action") if isinstance(
            status.get("last_action"), str) else None,
        last_error=status.get("last_error") if isinstance(
            status.get("last_error"), str) else None)


def drive_command_json(action: str) -> str:
    return json.dumps({"action": action})
```

- [ ] **Step 4: Run to verify it passes**

Run the Step 2 command. Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add ground_station/models.py tests/test_models.py
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "ground_station: DriveState, parse_drive_status, drive_command_json

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01JKPAhQayr2XFS87SjxAwzJ"
```

---

### Task 7: ground station DRIVE row + client wiring

**Files:**
- Create: `ground_station/ui/drive_row.py`
- Test: `tests/test_drive_row.py`
- Modify: `ground_station/ros_client.py` (signal, subscribe, publish), `ground_station/ui/dashboard_page.py` (place the row), `ground_station/ui/main_window.py` (wire signals + staleness)

**Interfaces:**
- Consumes: `DriveState`, `parse_drive_status`, `drive_command_json` (Task 6).
- Produces: `class DriveRow(QWidget)` with signals `stop_requested`, `manual_requested`, `init_requested`, `reset_encoders_requested`, `reset_odometry_requested`, `drive_mode_requested`, `drive_state_requested`; methods `set_state(DriveState | None)`, `refresh(now=None)`; injectable `confirm_init`, `confirm_reset_encoders` (default to `QMessageBox` dialogs). `RosSignals` gains `drive_status_received = Signal(object)`. `RosBridgeClient` gains `subscribe_drive_status(topic="/drive_status")` and `send_drive_command(action, topic="/drive_command")`.

- [ ] **Step 1: Write the failing tests** (mirror `tests/test_map_row.py`)

```python
# test_drive_row.py
from ground_station.models import DriveState
from ground_station.ui.drive_row import DriveRow


def state(**over):
    base = dict(connected=True, lease=True, coordinator_state="Manual",
                deadman_active=False, twist_age_s=0.1, last_action=None,
                last_error=None)
    base.update(over)
    return DriveState(**base)


def test_stop_is_always_enabled_even_with_no_status(qtbot):
    row = DriveRow()
    qtbot.addWidget(row)
    row.set_state(None)
    assert row.stop_button.isEnabled()


def test_stop_emits_without_confirmation(qtbot):
    row = DriveRow()
    qtbot.addWidget(row)
    with qtbot.waitSignal(row.stop_requested):
        row.stop_button.click()


def test_init_asks_for_confirmation_before_emitting(qtbot):
    row = DriveRow()
    qtbot.addWidget(row)
    emitted = []
    row.init_requested.connect(lambda: emitted.append(True))
    row.confirm_init = lambda: False
    row.init_button.click()
    assert emitted == []
    row.confirm_init = lambda: True
    row.init_button.click()
    assert emitted == [True]


def test_status_line_shows_coordinator_state_and_deadman(qtbot):
    row = DriveRow()
    qtbot.addWidget(row)
    row.set_state(state(coordinator_state="Idle", deadman_active=True))
    text = row.status_label.text().lower()
    assert "idle" in text and "deadman" in text


def test_manual_shows_arming_while_preparing(qtbot):
    row = DriveRow()
    qtbot.addWidget(row)
    row.set_state(state(coordinator_state="PrepareManual"))
    assert "arming" in row.status_label.text().lower()
```

- [ ] **Step 2: Run to verify it fails**

Run: `QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest tests/test_drive_row.py -q`
Expected: FAIL — no `drive_row` module.

- [ ] **Step 3: Implement `ground_station/ui/drive_row.py`** (follow `map_row.py`'s structure: no ROS, rich-text status line, escaped values, injectable dialogs)

```python
"""The Drive row: STOP, Manual, Init and the reset/mode buttons, plus a
status line from /drive_status. Shown when a rover drive link is wanted.
No ROS in this file - it talks through signals the window routes."""

import html
from time import monotonic

from PySide6.QtWidgets import (QHBoxLayout, QLabel, QMessageBox, QPushButton,
                               QWidget)

from ground_station import theme
from ground_station.models import DriveState


def _plain(text) -> str:
    return html.escape(str(text))


class DriveRow(QWidget):
    from PySide6.QtCore import Signal
    stop_requested = Signal()
    manual_requested = Signal()
    init_requested = Signal()
    reset_encoders_requested = Signal()
    reset_odometry_requested = Signal()
    drive_mode_requested = Signal()
    drive_state_requested = Signal()

    def __init__(self, parent=None, clock=monotonic):
        super().__init__(parent)
        self._state = None
        self._clock = clock
        self.confirm_init = self._confirm_init_dialog
        self.confirm_reset_encoders = self._confirm_reset_encoders_dialog

        self.stop_button = QPushButton("STOP")
        self.manual_button = QPushButton("Manual")
        self.init_button = QPushButton("Init drive")
        self.reset_enc_button = QPushButton("Reset encoders")
        self.reset_odom_button = QPushButton("Reset odometry")
        self.mode_button = QPushButton("Drive mode")
        self.state_button = QPushButton("Drive state")
        self.status_label = QLabel()
        from PySide6.QtCore import Qt
        self.status_label.setTextFormat(Qt.TextFormat.RichText)
        self.status_label.setStyleSheet(
            f"color: {theme.TEXT_DIM}; font-family: {theme.MONO_FONT_FAMILY};")
        self.stop_button.setStyleSheet(f"color: {theme.BAD};")

        layout = QHBoxLayout(self)
        layout.addWidget(QLabel("DRIVE"))
        for b in (self.stop_button, self.manual_button, self.init_button,
                  self.reset_enc_button, self.reset_odom_button,
                  self.mode_button, self.state_button):
            layout.addWidget(b)
        layout.addWidget(self.status_label, stretch=2)

        self.stop_button.clicked.connect(self.stop_requested.emit)
        self.manual_button.clicked.connect(self.manual_requested.emit)
        self.init_button.clicked.connect(self._on_init)
        self.reset_enc_button.clicked.connect(self._on_reset_encoders)
        self.reset_odom_button.clicked.connect(self.reset_odometry_requested.emit)
        self.mode_button.clicked.connect(self.drive_mode_requested.emit)
        self.state_button.clicked.connect(self.drive_state_requested.emit)
        self.set_state(None)

    def set_state(self, state) -> None:
        self._state = state
        movable = state is not None and state.connected
        for b in (self.manual_button, self.init_button, self.reset_enc_button,
                  self.reset_odom_button, self.mode_button, self.state_button):
            b.setEnabled(movable)
        self.stop_button.setEnabled(True)          # always
        self._refresh_status()

    def refresh(self, now=None) -> None:
        self._refresh_status(now)

    def _refresh_status(self, now=None) -> None:
        if self._state is None:
            self.status_label.setText("DRIVE: no status")
            return
        s = self._state
        parts = []
        if not s.connected:
            parts.append(f'<span style="color: {theme.BAD};">disconnected</span>')
        else:
            parts.append(_plain(f"lease {'held' if s.lease else 'none'}"))
        if s.coordinator_state == "PrepareManual":
            parts.append(_plain("arming (5 s)"))
        elif s.coordinator_state:
            parts.append(_plain(s.coordinator_state))
        if s.deadman_active:
            parts.append(f'<span style="color: {theme.ACCENT};">deadman</span>')
        if s.twist_age_s is not None:
            parts.append(_plain(f"twist {s.twist_age_s:.1f}s"))
        if s.last_error:
            parts.append(f'<span style="color: {theme.BAD};">'
                         f'{_plain(s.last_error)}</span>')
        self.status_label.setText(" | ".join(parts))

    def _on_init(self):
        if self.confirm_init():
            self.init_requested.emit()

    def _on_reset_encoders(self):
        if self.confirm_reset_encoders():
            self.reset_encoders_requested.emit()

    def _confirm_init_dialog(self) -> bool:
        answer = QMessageBox.question(
            self, "Init drive",
            "Initialise the drive? The wheels will move to zero the steering.")
        return answer == QMessageBox.StandardButton.Yes

    def _confirm_reset_encoders_dialog(self) -> bool:
        answer = QMessageBox.question(
            self, "Reset encoders",
            "Reset the steering encoders? The wheels will move.")
        return answer == QMessageBox.StandardButton.Yes
```

- [ ] **Step 4: Wire the client** — in `ground_station/ros_client.py`: add `drive_status_received = Signal(object)` to `RosSignals`; add (mirroring `subscribe_map_status`/`send_map_command`):
```python
    def subscribe_drive_status(self, topic_name: str = "/drive_status") -> None:
        topic = self._topic_factory(self._ros, topic_name, "std_msgs/String")
        topic.subscribe(lambda msg: self.signals.drive_status_received.emit(
            parse_drive_status(msg.get("data", ""))))
        self._drive_status_topic = topic

    def send_drive_command(self, action: str,
                           topic_name: str = "/drive_command") -> None:
        if not self.is_connected:
            print("ground_station: not connected, drive command dropped", file=sys.stderr)
            return
        if self._drive_command_topic is None:
            self._drive_command_topic = self._topic_factory(
                self._ros, topic_name, "std_msgs/String")
        self._drive_command_topic.publish(self._message_factory(
            {"data": drive_command_json(action)}))
```
Initialise `self._drive_status_topic = None` and `self._drive_command_topic = None` in `__init__`, and import `parse_drive_status, drive_command_json`. Call `subscribe_drive_status()` in `_connect_to` beside the other subscribes.

- [ ] **Step 5: Place and wire the row** — in `dashboard_page.py` add `self.drive_row = DriveRow()`, `self.drive_row.setVisible(False)`, and `left.addWidget(self.drive_row)` below `map_row`. In `main_window.py`: connect the seven signals to `self.ros_client.send_drive_command("stop"|"manual"|"init"|"reset_encoders"|"reset_odometry"|"drive_mode"|"drive_state")`; connect `drive_status_received` to a `_on_drive_status` that calls `self.dashboard_page.drive_row.set_state(state)` and records `self._drive_status_at`; make the row visible in `semi_auto` (and, if driving is wanted in `simulation`, there too — match `map_row`'s visibility rule); in the staleness timer add the same `set_state(None)`-when-stale / `refresh(now)` pair the map row has.

- [ ] **Step 6: Run all GS tests**

Run: `QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest tests/test_drive_row.py tests/test_models.py -q`
Expected: PASS. Then the full GS suite: `QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest tests/ -q` — expected green.

- [ ] **Step 7: Commit**

```bash
git add ground_station/ui/drive_row.py ground_station/ui/dashboard_page.py ground_station/ui/main_window.py ground_station/ros_client.py tests/test_drive_row.py
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "ground_station: DRIVE row - STOP/Manual/Init and a /drive_status line

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01JKPAhQayr2XFS87SjxAwzJ"
```

---

### Task 8: sim bench + full verification (no hardware)

**Files:**
- Modify: `rover/src/navi_teleop/test/fake_bema_server.py` (add the `--forward-twist` script `main`)
- No new production code.

**Interfaces:**
- Consumes: everything above.

- [ ] **Step 1: Add the script entrypoint to fake_bema_server.py**

```python
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
```

- [ ] **Step 2: Run the full navi_teleop suite locally**

Run: `bash -c 'source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_teleop:$PWD/rover/src/navi_teleop/test:$PYTHONPATH python3 -m pytest rover/src/navi_teleop/test/ -q'`
Expected: all green (existing video/image tests plus the four new files).

- [ ] **Step 3: Sim bench end-to-end (throwaway domain, no rover, no hardware)**

In three shells, all with `ROS_DOMAIN_ID=93`:
```bash
# 1: the sim
./start_sim.sh                       # or the project's sim launch, semi mode
# 2: the fake primary, forwarding to the sim's test twist
ROS_DOMAIN_ID=93 python3 rover/src/navi_teleop/test/fake_bema_server.py \
  --bema-port 21022 --coordinator-port 21031 --forward-twist
# 3: the bridge, pointed at localhost
ROS_DOMAIN_ID=93 ros2 run navi_teleop bema_bridge \
  --ros-args -p bema_host:=127.0.0.1
# then publish a twist and watch the Gazebo rover move:
ROS_DOMAIN_ID=93 ros2 topic pub -r 20 /manual_twist geometry_msgs/Twist \
  '{linear: {x: 0.2}}'
```
Expected: the Gazebo rover drives forward; stopping the twist publisher stops it within `deadman_s`. **This is the sign check for `w`:** publish `{angular: {z: 0.5}}` and confirm the rover yaws counter-clockwise (left). If it yaws right, the negation in `_twist_to_drive`/`_on_twist` is wrong — fix it in that one place and note it in PROJECT_SUMMARY.

- [ ] **Step 4: Deploy to the Orin and run the suite there**

```bash
./deploy_rover.sh --test
```
Expected: `colcon build` clean; the navi_teleop tests pass on the Orin. **Before the first deploy**, ensure `msgpack` is installed in the Orin's Python (one-time): `ssh star@a_navi 'python3 -c "import msgpack" || pip install --user msgpack'`.

- [ ] **Step 5: Update PROJECT_SUMMARY.md**

Add a short section: the bridge exists, how it starts (`start_navi.sh`, `--no-bridge`), the `msgpack` dependency, the deadman value, that the `w` sign was confirmed in sim, and that the coordinator/BEMA addresses are node parameters defaulting to `192.168.178.26`. Note what is still open: hardware bring-up on blocks, and the autonomy RPC-server path (out of scope here).

- [ ] **Step 6: Commit**

```bash
git add rover/src/navi_teleop/test/fake_bema_server.py PROJECT_SUMMARY.md
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "navi_teleop: sim bench for the BEMA bridge (fake server forwards F1 to /sim_test_twist); document the bridge

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01JKPAhQayr2XFS87SjxAwzJ"
```

---

## Hardware bring-up (operator, after the plan — not an automated task)

Not part of the automated plan; run with the operator present:

1. Rover on blocks, wheels clear. Ground station in the drive mode; a hand on STOP.
2. Press **Manual**, wait for the coordinator to reach `Manual` (status line stops saying "arming"). Press **Init drive** (wheels will move to zero the steering).
3. Nudge the stick: confirm each wheel drives and steers the expected way, and that the **`w` sign** matches the sim (CCW stick = CCW yaw).
4. Release the stick / close the GS: confirm the wheels stop within `deadman_s`.
5. Only then, off blocks, at walking pace, STOP under a hand.

## Self-review notes

- Spec coverage: msgpack client (T1), fake server (T2), session with lease+heartbeat+reconnect (T3), node with deadman+dispatch+status (T4), packaging+launch (T5), GS models (T6), GS row+wiring (T7), sim bench+verify+docs (T8). Deadman 1 s = `deadman_s` default. `w` sign check is an explicit step (T8.3, hardware.3). Units, addresses-as-parameters, no-move-on-startup, no-`__sam__force`, no `rclpy` in GS — all encoded.
- The coordinator's real `startManual` has a 5 s Idle→Manual delay; the fake collapses it to immediate. Tests that assert "arming" drive the row with `coordinator_state="PrepareManual"` directly (T7), so the delay is not needed in the double.
- `msgpack` is a genuine new dependency on the Orin; T8.4 installs it before the first deploy and T5/T8.5 document it.
