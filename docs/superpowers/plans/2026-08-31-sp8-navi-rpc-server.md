# SP8: NaVi RPC Server Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The Orin serves the endpoint the primary's coordinator calls "NaVi" — msgpack-RPC on `192.168.178.18:21021` — so that `CoordinatorImpl::startNaViTask` finds it, hands over the waypoints, and drives the mission state machine into `Autonomous` instead of logging *"Cannot start navigation task: NaVi not reachable"* and dropping back to `Idle`. Without this, our own Go button cannot arm an autonomous run, no matter how good Nav2 is.

**Architecture:** Two ROS-free modules and one thin node in the existing `navi_supervisor` package. `navi_rpc_protocol.py` is the wire and the lease: a threaded socket server speaking rpclib's `[0, msgid, method, [args]]` / `[1, msgid, error, result]` framing, plus the `__sam__` access manager the primary's `AutoConnection` performs before every guarded call. `navi_rpc_state.py` is the NaVi contract: the served method table `F0`–`F9`, the waypoint list, the run flags, and the queue of side effects the run implies. `navi_rpc_server.py` is the rclpy node: it publishes the waypoints as a latched `nav_msgs/Path`, publishes its own state as JSON for SP11's `goal_relay`, takes waypoint progress back in, and reports it to the coordinator as `notifyTaskFinished(tag)` through the msgpack session `bema_bridge` already owns.

**Tech Stack:** Python 3.10, `msgpack` 1.0.3, rclpy (ROS 2 Humble: `nav_msgs`, `geometry_msgs`, `std_msgs`), colcon (ament_python), pytest, bash.

**Spec:** `docs/superpowers/specs/autonomy-plan.md` — §3 (binding: the coordinator/NaVi contract, the `.18` alias, operator-initiated runs only), §2 (topic graph), §8 SP8 row, §12.

**Work in the `autonomy` worktree** (`.worktrees/autonomy`, branch `autonomy`), where SP4–SP7 already landed. All paths below are relative to that worktree's root.

---

## Global Constraints

### The wire, read out of the primary's own sources

Ground truth is `coordinator/deps/naviInterface/` and `coordinator/deps/rpc_base/`, not memory:

- **Endpoint:** `navi::NaViEP::address = "192.168.178.18"`, `port = 21021`. Hard-coded in the primary's binary. The `.18` alias on the Orin is what makes those constants land on us; **nothing on the primary changes** (spec §3).
- **Framing:** rpclib/msgpack-RPC. Request `[0, msgid, method, [args]]`, response `[1, msgid, error, result]`, `error` nil on success. Notifications `[2, method, [args]]` get no response. Same wire `navi_teleop/msgpack_rpc.py` already speaks as a client.
- **The lease is not optional.** `AutoConnection<navi::NaViEP>::getCapability()` calls `__sam__request` and returns `nullptr` when it is not answered `true` — and `nullptr` is exactly the branch that logs *"Cannot start navigation task: NaVi not reachable"*. A server that binds `:21021` and serves `F0`–`F9` but not `__sam__` is, from the coordinator's side, **not there at all**. Four methods, never guarded:

  | method | answer | semantics (`starpc::ServerAccessManager`) |
  |---|---|---|
  | `__sam__request` | `bool` | `true` if free or already ours, `false` if another session holds it |
  | `__sam__force` | nil | takes the lease unconditionally (`PrivilegedClient::takeOverServer`) |
  | `__sam__ping` | `bool` | `true` only while we are the holder; refreshes the lease. The client pings every **500 ms** (`ServerCapability::keepAlive`) |
  | `__sam__release` | `bool` | `true` only if we were the holder |

  Lease timeout **4.0 s** of inactivity (`rwd.setup(4s)`), refreshed by `__sam__ping` *and* by any successful guarded call (`notifyClientActivity`).
- **Refusing a guarded call to a non-holder is `respond_error(1)`** — the msgpack error field is the **integer 1**, not a string. `ServerProxy::checkAccess` does exactly that, and the client turns it into `rpc::rpc_error` → `AccessRevokedException` → a fresh `__sam__request` on the next tick. Our refusals of *unimplemented* methods use a **string** error instead, so the two are distinguishable in a capture.
- **Method table** (`NaViProxy` binding order; guard flags from `NaVi.idl`'s `@noguard`):

  | method | name | SP8 | guarded | args | result |
  |---|---|---|---|---|---|
  | `F0` | `init()` | **serve** | yes | — | nil |
  | `F1` | `setPosition(x, y)` | named refusal | yes | 2 floats | error (string) |
  | `F2` | `getPosition()` | benign stub | **no** | — | 4×4 identity |
  | `F3` | `setTargets(vector<(float,float,float)>)` | **serve** | yes | 1 array of 3-arrays | nil |
  | `F4` | `startNavigation()` | **serve** | yes | — | nil |
  | `F5` | `isTargetReached()` | **serve** | yes | — | **bool** |
  | `F6` | `stopNavigation()` | **serve** | yes | — | nil |
  | `F7` | `setMovementEnabled(bool)` | **serve** | yes | 1 bool | nil |
  | `F8` | `getTofData()` | benign stub | **no** | — | `[]` |
  | `F9` | `takeSnapshot(index)` | benign stub | **no** | 1 int | nil |
  | `F10`–`F12` | camera pan/tilt/zoom | not bound | — | — | unknown-method error |

  `F5` must answer a real msgpack **bool**: the client does `.as<bool>()`, which throws on an int.
- **The three `@noguard` methods must not be answered with an error.** Every method in `NaViEP.h` wraps its `mStub->call(...)` in `try { } catch (rpc::rpc_error&)` — except the three unguarded ones. `WeakNaViEP::getPosition()`, `getTofData()` and `takeSnapshot()` (`WeakNaViEP.h:28-58`) call straight through with no handler, so an error reply becomes an `rpc::rpc_error` propagating into whatever thread the caller runs on, and `std::terminate` if it escapes a thread function. `CoordinatorImpl` never calls them, but the whole point of the `.18` alias is that we *become* the NaVi endpoint for the entire rover LAN: anything on the primary's side that used to poll the real NaVi for pose, ToF frames or a snapshot now reaches us. So they answer benign, correctly-shaped values — an identity pose, an empty frame, nil — which any consumer reads as "no data". `F1` is guarded and `NaViEP::setPosition` *does* catch, so it keeps the named string refusal.
- **Progress back to the coordinator:** `notifyTaskFinished(tag)` = the coordinator's **`F8` on `:21031`, unguarded** (`@noguard` in `Coordinator.idl`, bound without `checkAccess` in `CoordinatorProxy`). `TAG_WaypointReached = 0x31` (49), `TAG_DestinationReached = 0x32` (50).

### Binding, tests and safety

- The server binds **`0.0.0.0:21021`** by default, so a connection to `192.168.178.18:21021` succeeds whenever the alias exists — and the node still starts, and is still testable, on a machine where the alias could not be added. **Tests bind `127.0.0.1` on port `0`** (ephemeral). No test ever binds the alias, and no test needs root.
- **Never publish to `/manual_twist`.** ROS-graph tests keep the package's throwaway `ROS_DOMAIN_ID` **91** (`os.environ.setdefault` before importing rclpy — 93 is `test_bema_bridge.py`'s). Note the `setdefault`: within one pytest process the first test module to import wins, so every `navi_supervisor` test module must name the *same* domain, 91.
- Only `mode_supervisor` publishes `/rover_twist`. This node publishes `/drive_command`, `/navi_rpc/targets` and `/navi_rpc/status` — never a twist, and **never `/mode_request`**: mode is the supervisor's alone (SP5), and reaching for it from here turns the coordinator's pause into an abort. `/drive_command` already has `mode_supervisor` as a publisher, so this is a second writer of an existing multi-writer command topic, not a new single-writer breach.
- **A hostile or buggy client must never kill a thread or the node.** Wrong arity, wrong types, unknown methods, unknown `__sam__` variants, oversized waypoint lists, truncated frames and non-msgpack garbage are all answered or dropped per-connection. Hostile-input tests are part of Task 1 and are not optional.
- The pure layers (`navi_rpc_protocol.py`, `navi_rpc_state.py`) import **no ROS**. Same split as `bema_session.py`/`bema_bridge.py` and `supervisor_state.py`/`mode_supervisor.py`: time is passed in, so everything runs against a fake clock.
- Both suites stay green after **every** task, at or above their current counts:
  - `navi_supervisor`: **49 passed** today.
  - `navi_teleop`: **79 passed** today.
- Commits: explicit `git add <paths>` (never `git add -A`), `git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit`, **never push**. On `index.lock`, wait 2 s and retry — other planning and implementation agents work in this tree.
- **Do not touch** anything under `sim/`, `ground_station/`, `rover/src/navi_autonomy/`, `rover/src/navi_localization/`, or the SP4–SP7/SP9–SP11 plan files. The only shared file this plan edits is `rover/start_navi.sh` (Task 6), and that edit is confined to one flag, one function and one launch block.

### Commands

Package suite (no ROS graph needed for Tasks 1–3, needed from Task 4):

```
bash -c 'source /opt/ros/humble/setup.bash &&
  PYTHONPATH=$PWD/rover/src/navi_supervisor:$PYTHONPATH \
  python3 -m pytest rover/src/navi_supervisor/test -q -p no:cacheprovider'
```

Teleop suite (Task 5 touches it):

```
bash -c 'source /opt/ros/humble/setup.bash &&
  PYTHONPATH=$PWD/rover/src/navi_teleop:$PYTHONPATH \
  python3 -m pytest rover/src/navi_teleop/test -q -p no:cacheprovider'
```

Build: `bash -c 'source /opt/ros/humble/setup.bash && cd rover && colcon build --packages-select navi_supervisor navi_teleop'`.

Alias gate (Task 6): `bash rover/test/test_navi_alias.sh` (exit 0 = pass).

### Design decisions

1. **The code lives in `rover/src/navi_supervisor/`, as three modules, not in a new `navi_rpc` package.** SP8 depends on SP5, and everything it does lands on the supervisor's own input `/drive_command`, on `/mode_status` coming back, and on state SP11's `goal_relay` reads beside it. A fourth package would duplicate `package.xml`/`setup.py`/`setup.cfg` for two files and no separation anyone needs. (SP7's decision 2 predicted `navi_rpc_server` would land in `navi_autonomy`; SP5 then put the arbitration in `navi_supervisor`, and the RPC server belongs with the arbitration, not with the costmap maths.)
2. **Typed messages where a standard one fits the data exactly; JSON-in-`std_msgs/String` for status.** The waypoint list is `nav_msgs/Path` (latched, `transient_local`): it is a list of poses in `map`, which is precisely what `Path` is, and it is what SP11 hands to `NavigateThroughPoses` with no translation. The run state is a dict, so it goes out as JSON on `/navi_rpc/status`, matching `/drive_status`, `/mode_status`, `/localization/status` and `/map_status`, which the ground station already reads over rosbridge with no ROS installed. A custom `.msg` would cost an `ament_cmake` package (SP5 decision 2).
3. **`notifyTaskFinished` goes out through `bema_bridge`'s existing coordinator session,** as `{"action": "task_finished", "tag": 49|50}` on `/drive_command` → `BemaSession.notify_task_finished(tag)` → coordinator `F8`. Reason: SP5's precedent ("the supervisor deliberately does not open its own connection to the primary"), one socket instead of two, and reconnect-with-backoff for free. `F8` is `@noguard`, so this path never touches the coordinator lease and cannot fight `bema_bridge` for it — the reason SP5's precedent exists in the first place. `BemaSession._safe_coord` already exists for exactly this shape and is currently unused. One consequence worth stating rather than leaving a reader to derive: `bema_bridge` runs a single executor, so `_on_command` → `notify_task_finished` → a synchronous `RpcClient.call` with `timeout_s=0.3` shares its thread with the 20 Hz `_drive_tick`, and unlike the operator-pressed `abort`/`manual` this one is machine-generated during a run. It is bounded and safe — at most two per leg, and the bridge's own deadman is 1.0 s, so a stalled tick cannot trip it — but it is a real 0.3 s worst-case stall on the drive tick, and the same applies to the `navi_task`/`pause_task`/`resume_task` calls Task 5 adds (those are operator-paced, one per run).
4. **The server never starts a run by itself.** `F4 startNavigation` sets `navigation_requested` and bumps `start_seq` in `/navi_rpc/status`; it does **not** publish `/nav_request` and does not command Nav2. Spec §3: runs are operator-initiated only, coordinator-initiated runs are out of scope. In the real sequence `F4` arrives ~5 s *after* the operator's Go (the coordinator's `PrepareAutonomous` → `Autonomous` delay), which makes it the useful signal it is: "the coordinator has reached `Autonomous`, the run is armed". SP11's `goal_relay` waits for it before sending the Nav2 goal.
5. **The lease is dropped when its connection closes.** rpclib leaks it (its own `TODO` says the holder is never cleaned up). A leaked lease here would mean that after a coordinator restart every `__sam__request` from the new session is refused and the rover is permanently "NaVi not reachable". Strictly safer, and invisible to a well-behaved client.
6. **Lease expiry is uniform** (4 s from `request`, `force`, `ping` or any guarded call). rpclib arms its watchdog only in `request`, so a `force`d lease there never expires by itself; mirroring that quirk buys nothing and keeps a stale holder alive forever. Expiry is checked lazily on each call rather than by a timer thread.

### Rulings on spec ambiguities (binding for this plan)

- **`w` in a target tuple is yaw in radians in the `map` frame.** The IDL calls it "the target rotation" and gives no unit. We are the only producer end-to-end (spec §3: operator-initiated only — our `goal_relay` calls `startNaViTask`, the coordinator relays the tuples straight back to us), so we define it. `targets_to_path` converts it to a quaternion about z. If ERC ever supplies coordinates in another convention, the conversion belongs in `goal_relay`, not here.
- **`stopNavigation` stops the wheels and raises a stop flag; it never asks for a mode.** The IDL says it "may also be used as an emergency stop", so it must have a real effect now, not when SP9 lands. The effect is `{"action": "stop"}` on `/drive_command`, a bumped `stop_seq` and `stop_requested = true` on `/navi_rpc/status`. It is emphatically **not** a `/mode_request {"mode": "manual"}`, for two independent reasons:
  - `SupervisorState.on_mode_request` (`supervisor_state.py:179-193`) refuses any non-`manual` request while the e-stop is latched, but a `manual` request falls through to `self._estop_latched = False`. The primary's state machine must not be able to clear our operator's e-stop.
  - Worse, in the case that looks safe: `CoordinatorImpl::pause()` (`CoordinatorImpl.cpp:93-119`) calls `F6` **first** and only then leaves `Autonomous`, so when `F6` lands our last `/mode_status` still says `autonomous`. A `manual` request there makes `SupervisorState` queue `CANCEL_GOAL, DEACTIVATE_NAV2, COORDINATOR_ABORT` → `/drive_command {"action": "abort"}` → coordinator `F7 abort` → `Idle` with `m_activeTask = None`. The operator's **Resume** is then refused from `Idle` and the ERC run is gone — a pause silently became an abort, with an `F6`→abort→`F6` RPC ping-pong on the way. This is the "two components owning stop" failure §4 exists to prevent, and it contradicts SP11's binding ruling (`2026-08-31-sp11-gs-nav.md:148`): *"the supervisor is the single authority on mode (SP5). One authority, one direction."*

  Until SP11 lands, `F6` therefore has no autonomy-stopping teeth beyond the chassis stop. That is honest and harmless: nothing publishes `/autonomy_twist` before SP9. From SP11 on, `goal_relay` watches `stop_seq` on `/navi_rpc/status` and cancels the Nav2 goal — cancelling at the source is what actually stops `/autonomy_twist`, and it keeps mode authority in one place.
- **`setMovementEnabled(false)` is treated as `stopNavigation`; `true` is recorded and nothing else.** `movement_enabled` is **never** a precondition for `F4`/`F5`. The coordinator's `movementUpdateLoop` for NaVi is commented out in `CoordinatorImpl.cpp` (line 38), so `F7` may never arrive at all; gating the run on it would mean autonomy never starts.
- **A failed run invents no completion tag.** The coordinator has no failure tag — `notifyTaskFinished` accepts only `0x31` and `0x32`, and both advance its state machine as if we had arrived. On `{"event": "failed"}` the server clears `navigation_requested`, records the reason in `/navi_rpc/status`, and takes the **same** stop path as `F6` and `F7(false)` — one chassis stop, one bumped `stop_seq`. All three stop paths are identical on purpose: whichever fired, `goal_relay` sees the same counter move. And a failed run needs no mode change: Nav2 stops publishing `/autonomy_twist`, and the supervisor's 0.5 s autonomy deadman zeroes `/rover_twist` by itself. The coordinator stays in `Autonomous` until the operator aborts, which is the truthful state and is what the NAV row shows.
- **After `0x31` the coordinator immediately calls `F4` again and then transitions to `Waiting`.** That is what `CoordinatorImpl::notifyTaskFinished` does (it clears `awaitingTargetReached`, calls `startNavigation_async()`, then `stateTransition(Waiting, Autonomous)`), and it also means a *second* `0x31` would be ignored until a `resume()` sets `awaitingTargetReached` again. Our server does not try to fix the primary: it treats every `F4` as idempotent, bumps `start_seq`, and clears the per-leg `target_reached` latch so `F5` cannot answer `true` for a leg that has not been driven. What actually stops the rover at an intermediate waypoint is the coordinator's own `Waiting` state disabling BEMA movement — not anything on this side.
- **`F5 isTargetReached` means "the current leg has been reported reached since the last `F4`".** Today's coordinator never polls it (no `isTargetReached` call exists in `CoordinatorImpl.cpp`); it is served because the interface promises it and because a bench client is the cheapest way to see the run state.
- **`F3 setTargets` with an empty list is refused.** `startNaViTask` returns early on empty waypoints, so an empty list on the wire is a bug on the caller's side, not a "clear the list" idiom. Refusing it says so.
- **`F10`–`F12` (the camera) get an unknown-method error, not a stub.** Spec §3 lists only `F0`–`F9` as the NaVi interface, and the pan/tilt/zoom head is not ours. A clean error beats a hang — and unlike the `@noguard` trio, nothing on the primary's side calls them at all, so there is no un-caught `rpc_error` to worry about.

---

## File structure

- Create `rover/src/navi_supervisor/navi_supervisor/navi_rpc_protocol.py` — the wire and the lease. No ROS.
- Create `rover/src/navi_supervisor/navi_supervisor/navi_rpc_state.py` — the NaVi contract and its method table. No ROS.
- Create `rover/src/navi_supervisor/navi_supervisor/navi_rpc_server.py` — the rclpy node.
- Create `rover/src/navi_supervisor/test/fake_coordinator.py` — the primary's client, on a plain socket.
- Create `rover/src/navi_supervisor/test/test_navi_rpc_protocol.py`, `test/test_navi_rpc_state.py`, `test/test_navi_rpc_server.py`
- Create `rover/test/test_navi_alias.sh` — the alias step, exercised without root.
- Modify `rover/src/navi_supervisor/setup.py`, `rover/src/navi_supervisor/package.xml`
- Modify `rover/src/navi_teleop/navi_teleop/bema_session.py` — `_coord_guarded()` factored out of `start_manual`/`abort`, then `start_navi_task(waypoints)`, `pause_task()`, `resume_task()` and `notify_task_finished(tag)`.
- Modify `rover/src/navi_teleop/navi_teleop/bema_bridge.py` — the `task_finished`, `navi_task`, `pause_task` and `resume_task` actions.
- Modify `rover/src/navi_teleop/test/fake_bema_server.py` — coordinator `F8` is unguarded.
- Modify `rover/src/navi_teleop/test/test_bema_bridge.py`, `test/test_bema_session.py`
- Modify `rover/start_navi.sh` — the alias, the flag, the launch, the cleanup pattern.

---

## Task 1: the wire and the lease (`navi_rpc_protocol.py`)

**Files:**
- Create: `rover/src/navi_supervisor/navi_supervisor/navi_rpc_protocol.py`
- Test: `rover/src/navi_supervisor/test/fake_coordinator.py`, `rover/src/navi_supervisor/test/test_navi_rpc_protocol.py`

**Interfaces:**
- Produces: `navi_supervisor.navi_rpc_protocol` with `RpcServer(methods, guarded=(), host="0.0.0.0", port=21021, clock=monotonic, logger=None, backlog=8, lease=None)` exposing `.start()`, `.stop()`, `.host`, `.port`, `.lease`; `Lease(timeout_s=LEASE_TIMEOUT_S)` with `request/force/ping/release/holds/touch/drop/holder` — every one of them takes `now` explicitly, so the `Lease` holds no clock of its own; `RpcRefusal(error)`; constants `LEASE_TIMEOUT_S = 4.0`, `ACCESS_DENIED_ERROR = 1`, `REQUEST_TYPE = 0`, `RESPONSE_TYPE = 1`, `NOTIFY_TYPE = 2`.
- Consumes: `socket`, `threading`, `msgpack`, `time.monotonic`. No ROS, no project imports.

### Steps

- [ ] Write `rover/src/navi_supervisor/test/fake_coordinator.py` — the client half of the contract, so every later test drives the server the way the primary does:

```python
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
```

- [ ] Write the failing test at `rover/src/navi_supervisor/test/test_navi_rpc_protocol.py`:

```python
"""The wire and the lease, against the client the primary actually runs.

Every server here binds 127.0.0.1 on port 0 - never the .18 alias, never
:21021, never anything that needs root.
"""

import time

import msgpack
import pytest

from fake_coordinator import FakeCoordinator, RpcError
from navi_supervisor.navi_rpc_protocol import (ACCESS_DENIED_ERROR,
                                               LEASE_TIMEOUT_S, Lease,
                                               RpcRefusal, RpcServer)


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class Recorder:
    """A toy method table: enough surface to test the transport with."""

    def __init__(self):
        self.calls = []

    def table(self):
        return {
            "F0": self._noop,
            "F4": self._noop,
            "F5": self._true,
            "F9": self._refuse,
            "echo": self._echo,
            "boom": self._boom,
        }

    def _noop(self):
        self.calls.append("noop")

    def _true(self):
        self.calls.append("true")
        return True

    def _echo(self, value):
        self.calls.append(("echo", value))
        return value

    def _refuse(self, index):
        raise RpcRefusal("F9 (takeSnapshot) is not served by navi_rpc_server")

    def _boom(self):
        raise RuntimeError("handler exploded")


@pytest.fixture
def served():
    clock = Clock()
    recorder = Recorder()
    server = RpcServer(recorder.table(), guarded=("F0", "F4", "F5"),
                       host="127.0.0.1", port=0, clock=clock)
    server.start()
    clients = []

    def connect():
        client = FakeCoordinator("127.0.0.1", server.port)
        clients.append(client)
        return client

    yield server, clock, recorder, connect
    for client in clients:
        client.close()
    server.stop()


# --- the lease, as a unit ------------------------------------------------
def test_a_free_lease_is_granted_and_a_taken_one_is_refused():
    clock = Clock()
    lease = Lease()
    assert lease.request(1, clock.t) is True
    assert lease.request(2, clock.t) is False
    assert lease.request(1, clock.t) is True          # already ours


def test_the_lease_expires_after_four_seconds_of_silence():
    clock = Clock()
    lease = Lease()
    lease.request(1, clock.t)
    clock.t = LEASE_TIMEOUT_S - 0.01
    assert lease.holds(1, clock.t) is True
    clock.t = LEASE_TIMEOUT_S + 0.01
    assert lease.holds(1, clock.t) is False
    assert lease.request(2, clock.t) is True


def test_a_ping_refreshes_the_lease_and_only_for_the_holder():
    clock = Clock()
    lease = Lease()
    lease.request(1, clock.t)
    clock.t = 3.0
    assert lease.ping(1, clock.t) is True
    assert lease.ping(2, clock.t) is False
    clock.t = 6.0                                     # 3 s after the ping
    assert lease.holds(1, clock.t) is True


def test_force_takes_the_lease_from_the_holder():
    clock = Clock()
    lease = Lease()
    lease.request(1, clock.t)
    lease.force(2, clock.t)
    assert lease.holds(2, clock.t) is True
    assert lease.holds(1, clock.t) is False


def test_release_only_works_for_the_holder():
    clock = Clock()
    lease = Lease()
    lease.request(1, clock.t)
    assert lease.release(2, clock.t) is False
    assert lease.release(1, clock.t) is True
    assert lease.request(2, clock.t) is True


def test_a_dropped_connection_frees_the_lease():
    clock = Clock()
    lease = Lease()
    lease.request(1, clock.t)
    lease.drop(1)
    assert lease.request(2, clock.t) is True


# --- the wire ------------------------------------------------------------
def test_the_sam_dance_then_a_guarded_call(served):
    server, clock, recorder, connect = served
    client = connect()
    assert client.access() is True
    assert client.call("F0") is None
    assert client.ping() is True
    assert client.release() is True
    assert recorder.calls == ["noop"]


def test_a_guarded_call_without_the_lease_is_error_one(served):
    server, clock, recorder, connect = served
    client = connect()
    with pytest.raises(RpcError) as excinfo:
        client.call("F0")
    assert excinfo.value.error == ACCESS_DENIED_ERROR
    assert recorder.calls == []


def test_a_second_session_is_refused_the_lease_but_still_served(served):
    server, clock, recorder, connect = served
    first, second = connect(), connect()
    assert first.access() is True
    assert second.access() is False
    with pytest.raises(RpcError):
        second.call("F4")
    assert first.call("F4") is None                   # the holder is unaffected


def test_a_guarded_call_refreshes_the_lease_like_notify_client_activity(served):
    server, clock, recorder, connect = served
    client = connect()
    client.access()
    clock.t = 3.0
    client.call("F4")                                 # activity, not a ping
    clock.t = 6.0
    assert client.call("F4") is None                  # still ours


def test_closing_the_connection_frees_the_lease_for_the_next_session(served):
    server, clock, recorder, connect = served
    first = connect()
    assert first.access() is True
    first.close()
    # Real sleeps, not fake-clock ones: this is the serve thread noticing the
    # FIN, and the fake clock never advances far enough to expire the lease
    # (4 s), so a pass here can only mean the close freed it.
    for _ in range(50):
        second = FakeCoordinator("127.0.0.1", server.port)
        granted = second.access()
        second.close()
        if granted:
            return
        time.sleep(0.02)
    pytest.fail("the lease was never freed by the closed connection")


def test_an_unguarded_method_needs_no_lease(served):
    server, clock, recorder, connect = served
    client = connect()
    assert client.call("echo", 7) == 7


def test_a_bool_result_stays_a_bool_on_the_wire(served):
    server, clock, recorder, connect = served
    client = connect()
    client.access()
    result = client.call("F5")
    assert result is True                             # not 1


def test_a_notification_is_run_and_not_answered(served):
    server, clock, recorder, connect = served
    client = connect()
    client.notify("echo", 3)
    assert client.call("echo", 4) == 4                # the next answer is not the notify's
    assert ("echo", 3) in recorder.calls


# --- hostile input -------------------------------------------------------
def test_an_unknown_method_is_an_error_not_a_crash(served):
    server, clock, recorder, connect = served
    client = connect()
    with pytest.raises(RpcError) as excinfo:
        client.call("F42")
    assert "unknown method" in str(excinfo.value.error)
    assert client.call("echo", 1) == 1


def test_the_camera_methods_are_unknown_methods(served):
    server, clock, recorder, connect = served
    client = connect()
    with pytest.raises(RpcError):
        client.call("F10", 0.0, 0.0, 0.0, 0.0)


def test_a_wrong_argument_count_is_an_error_not_a_crash(served):
    server, clock, recorder, connect = served
    client = connect()
    with pytest.raises(RpcError) as excinfo:
        client.call("echo")                            # echo takes one
    assert "bad arguments" in str(excinfo.value.error)
    with pytest.raises(RpcError):
        client.call("echo", 1, 2, 3)
    assert client.call("echo", 5) == 5


def test_a_refusal_carries_its_message_and_leaves_the_link_up(served):
    server, clock, recorder, connect = served
    client = connect()
    with pytest.raises(RpcError) as excinfo:
        client.call("F9", 0)
    assert "not served" in str(excinfo.value.error)
    assert client.call("echo", 2) == 2


def test_a_handler_that_raises_is_an_error_not_a_dead_thread(served):
    server, clock, recorder, connect = served
    client = connect()
    with pytest.raises(RpcError) as excinfo:
        client.call("boom")
    assert "boom" in str(excinfo.value.error)
    assert client.call("echo", 8) == 8


def test_arguments_that_are_not_an_array_are_an_error(served):
    server, clock, recorder, connect = served
    client = connect()
    client.send_raw(msgpack.packb([0, 99, "echo", "not-an-array"],
                                  use_bin_type=True))
    frame = client.read_frame()
    assert frame[0] == 1 and frame[1] == 99
    assert frame[2] is not None


def test_a_frame_that_is_not_a_request_closes_only_that_connection(served):
    server, clock, recorder, connect = served
    victim, survivor = connect(), connect()
    victim.send_raw(msgpack.packb({"not": "a frame"}, use_bin_type=True))
    assert victim.closed_by_server() is True
    assert survivor.call("echo", 6) == 6


def test_garbage_bytes_close_only_that_connection(served):
    server, clock, recorder, connect = served
    victim, survivor = connect(), connect()
    victim.send_raw(b"\xc1\xc1\xc1\xc1 not msgpack at all \xc1")
    assert victim.closed_by_server() is True
    assert survivor.call("echo", 6) == 6


def test_an_unknown_sam_method_is_an_unknown_method(served):
    server, clock, recorder, connect = served
    client = connect()
    with pytest.raises(RpcError):
        client.call("__sam__whatever")
    assert client.access() is True


def test_a_frame_split_across_two_packets_is_answered(served):
    # The single most load-bearing behaviour of _drain: next(unpacker)
    # raising StopIteration on a partial frame, and resuming on the next
    # feed(). TCP splits frames whenever it feels like it.
    server, clock, recorder, connect = served
    client = connect()
    frame = msgpack.packb([0, 77, "echo", [42]], use_bin_type=True)
    client.send_raw(frame[:3]); time.sleep(0.05); client.send_raw(frame[3:])
    assert client.read_frame() == [1, 77, None, 42]


def test_an_oversized_stream_costs_only_that_connection(served):
    # _MAX_BUFFER_BYTES, and the unpacker.feed -> BufferFull -> close path.
    server, clock, recorder, connect = served
    victim, survivor = connect(), connect()
    try:
        victim.send_raw(b"\x91" * (2 * 1024 * 1024))   # 2 MB, never a complete frame
    except OSError:
        pass
    assert victim.closed_by_server() is True
    assert survivor.call("echo", 6) == 6


def test_a_half_closed_socket_still_gets_its_answer(served):
    # The buffered frame drains and the reply goes out before recv() returns
    # b"". Correct today; pinned so it stays correct.
    server, clock, recorder, connect = served
    client = connect()
    client.send_raw(msgpack.packb([0, 88, "echo", [9]], use_bin_type=True))
    client.half_close()
    assert client.read_frame() == [1, 88, None, 9]
```

- [ ] Run it and watch it fail: `bash -c 'source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_supervisor:$PYTHONPATH python3 -m pytest rover/src/navi_supervisor/test/test_navi_rpc_protocol.py -q -p no:cacheprovider'` → expect a collection error, `ModuleNotFoundError: No module named 'navi_supervisor.navi_rpc_protocol'`, 0 passed.

- [ ] Implement `rover/src/navi_supervisor/navi_supervisor/navi_rpc_protocol.py`:

```python
"""The rpclib wire, served rather than called, and the lease that guards it.

The primary's coordinator talks to us with rpclib: a request is
[0, msgid, method, [args]], a response [1, msgid, error, result] with error
nil on success, and a notification [2, method, [args]] gets no response at
all. `navi_teleop/msgpack_rpc.py` is the client half of the same wire; this
is the server half, and it is deliberately the same dependency-light shape
(just `msgpack` and `socket`), with no ROS in it, so the whole contract can
be tested against a fake coordinator with no executor running.

The lease is the part that is easy to miss and fatal to omit.
`AutoConnection<navi::NaViEP>::getCapability()` calls `__sam__request` first
and returns nullptr if it is not answered `true` - and nullptr is the branch
that logs "Cannot start navigation task: NaVi not reachable". A server that
binds :21021 and serves F0-F9 but not `__sam__` is, from the coordinator's
side, not there at all.

Two deliberate differences from `starpc::ServerAccessManager`:

  * the lease is released when its connection closes. rpclib leaks it (its
    own TODO says so), which would mean that after a coordinator restart the
    new session's request is refused forever and the rover is permanently
    unreachable.
  * expiry is uniform. rpclib arms its watchdog only in `request`, so a
    forced lease there never times out by itself.
"""

import socket
import threading
from time import monotonic

import msgpack

REQUEST_TYPE = 0
RESPONSE_TYPE = 1
NOTIFY_TYPE = 2

# starpc::ServerAccessManager: rwd.setup(4s), refreshed by __sam__ping and by
# notifyClientActivity() after every successful guarded call.
LEASE_TIMEOUT_S = 4.0

# ServerProxy::checkAccess() answers a non-holder with respond_error(1) - the
# integer, not a string. Unimplemented methods answer with a string instead,
# so the two are told apart in a capture.
ACCESS_DENIED_ERROR = 1

SAM_METHODS = ("__sam__request", "__sam__force", "__sam__ping",
               "__sam__release")

_ACCEPT_TIMEOUT_S = 0.2
_RECV_TIMEOUT_S = 0.2
_RECV_BYTES = 4096
# A waypoint list is a few hundred bytes. Anything approaching this is not a
# coordinator, and unpacking it would be the whole attack.
_MAX_BUFFER_BYTES = 1024 * 1024


class RpcRefusal(Exception):
    """A method that exists but declines to act. `error` goes on the wire."""

    def __init__(self, error):
        super().__init__(str(error))
        self.error = error


class Lease:
    """`starpc::ServerAccessManager`, with the two differences above.

    It holds no clock: every method takes `now` from its caller, which is
    what lets the whole thing be driven by a fake clock in the tests.
    """

    def __init__(self, timeout_s=LEASE_TIMEOUT_S):
        self._timeout_s = float(timeout_s)
        self._lock = threading.RLock()
        self._holder = None
        self._deadline = None

    def _expire(self, now):
        if (self._holder is not None and self._deadline is not None
                and now >= self._deadline):
            self._holder = None
            self._deadline = None

    def request(self, session, now):
        with self._lock:
            self._expire(now)
            if self._holder is not None and self._holder != session:
                return False
            self._holder = session
            self._deadline = now + self._timeout_s
            return True

    def force(self, session, now):
        with self._lock:
            self._holder = session
            self._deadline = now + self._timeout_s

    def ping(self, session, now):
        with self._lock:
            self._expire(now)
            if self._holder != session:
                return False
            self._deadline = now + self._timeout_s
            return True

    def release(self, session, now):
        with self._lock:
            self._expire(now)
            if self._holder != session:
                return False
            self._holder = None
            self._deadline = None
            return True

    def holds(self, session, now):
        with self._lock:
            self._expire(now)
            return self._holder == session

    def touch(self, now):
        """ServerAccessManager::notifyClientActivity()."""
        with self._lock:
            if self._holder is not None:
                self._deadline = now + self._timeout_s

    def drop(self, session):
        with self._lock:
            if self._holder == session:
                self._holder = None
                self._deadline = None

    def holder(self, now):
        with self._lock:
            self._expire(now)
            return self._holder


class RpcServer:
    """One accept thread, one thread per connection, one shared method table.

    `methods` maps a name to a plain callable; `guarded` names the subset
    that needs the lease. The `__sam__` methods are handled here rather than
    in the table, because they are the only ones that need the session id.
    """

    def __init__(self, methods, guarded=(), host="0.0.0.0", port=21021,
                 clock=monotonic, logger=None, backlog=8, lease=None):
        self._methods = dict(methods)
        self._guarded = frozenset(guarded)
        self._clock = clock
        self._logger = logger
        self.lease = lease if lease is not None else Lease()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, int(port)))
        self._sock.listen(backlog)
        self._sock.settimeout(_ACCEPT_TIMEOUT_S)
        self.host, self.port = self._sock.getsockname()
        self._stop = False
        self._sessions = 0
        self._sessions_lock = threading.Lock()
        self._thread = threading.Thread(target=self._accept_loop, daemon=True)

    # --- lifecycle -------------------------------------------------------
    def start(self):
        self._thread.start()

    def stop(self):
        self._stop = True
        try:
            self._sock.close()
        except OSError:
            pass

    def _log(self, level, message):
        if self._logger is None:
            return
        try:
            getattr(self._logger, level)(message)
        except Exception:                              # a logger must not kill us
            pass

    def _accept_loop(self):
        while not self._stop:
            try:
                conn, peer = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with self._sessions_lock:
                self._sessions += 1
                session = self._sessions
            conn.settimeout(_RECV_TIMEOUT_S)
            threading.Thread(target=self._serve, args=(conn, session, peer),
                             daemon=True).start()

    # --- one connection --------------------------------------------------
    def _serve(self, conn, session, peer):
        unpacker = msgpack.Unpacker(raw=False, max_buffer_size=_MAX_BUFFER_BYTES)
        try:
            while not self._stop:
                try:
                    data = conn.recv(_RECV_BYTES)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not data:
                    break
                try:
                    unpacker.feed(data)
                except Exception as exc:               # buffer limit exceeded
                    self._log("warn", f"session {session} from {peer}: "
                                      f"oversized stream ({exc!r}); closing")
                    break
                if not self._drain(conn, session, peer, unpacker):
                    break
        finally:
            # Never leak the lease onto a socket nobody holds any more.
            self.lease.drop(session)
            try:
                conn.close()
            except OSError:
                pass

    def _drain(self, conn, session, peer, unpacker):
        """Answer every complete frame in the buffer. False = hang up."""
        while True:
            try:
                message = next(unpacker)
            except StopIteration:
                return True
            except Exception as exc:                   # not msgpack at all
                self._log("warn", f"session {session} from {peer}: "
                                  f"undecodable msgpack ({exc!r}); closing")
                return False
            frame = self._parse(message)
            if frame is None:
                self._log("warn", f"session {session} from {peer}: "
                                  f"not an rpc frame ({message!r}); closing")
                return False
            msgid, method, args, wants_reply = frame
            error, result = self._dispatch(session, method, args)
            if not wants_reply:
                continue
            try:
                conn.sendall(msgpack.packb([RESPONSE_TYPE, msgid, error, result],
                                           use_bin_type=True))
            except OSError:
                return False

    @staticmethod
    def _parse(message):
        """(msgid, method, args, wants_reply), or None if it is not a frame.

        A desynchronised stream cannot be recovered - the next bytes are the
        middle of something - so a frame that is not a request or a
        notification costs that one connection, and nothing else.
        """
        if not isinstance(message, (list, tuple)):
            return None
        if len(message) == 4 and message[0] == REQUEST_TYPE:
            _type, msgid, method, args = message
            wants_reply = True
        elif len(message) == 3 and message[0] == NOTIFY_TYPE:
            _type, method, args = message
            msgid, wants_reply = None, False
        else:
            return None
        if isinstance(method, bytes):
            method = method.decode("utf-8", "replace")
        if not isinstance(method, str):
            return None
        if args is None:
            args = []
        if not isinstance(args, (list, tuple)):
            # Answerable rather than fatal: the frame itself is well formed,
            # so the stream is still in sync.
            return msgid, method, None, wants_reply
        return msgid, method, list(args), wants_reply

    def _dispatch(self, session, method, args):
        now = self._clock()
        if method in SAM_METHODS:
            return self._sam(session, method, now)
        if args is None:
            return f"{method}: arguments must be an array", None
        handler = self._methods.get(method)
        if handler is None:
            return f"unknown method {method!r}", None
        if method in self._guarded and not self.lease.holds(session, now):
            return ACCESS_DENIED_ERROR, None
        try:
            result = handler(*args)
        except RpcRefusal as exc:
            return exc.error, None
        except TypeError as exc:
            # Arity and type mismatches from the call itself. A TypeError
            # raised deeper inside a handler is reported the same way; the
            # message carries which.
            return f"{method}: bad arguments ({exc})", None
        except ValueError as exc:
            return f"{method}: {exc}", None
        except Exception as exc:
            self._log("error", f"{method} failed: {exc!r}")
            return f"{method} failed: {exc!r}", None
        if method in self._guarded:
            self.lease.touch(now)
        return None, result

    def _sam(self, session, method, now):
        if method == "__sam__request":
            return None, self.lease.request(session, now)
        if method == "__sam__force":
            self.lease.force(session, now)
            return None, None
        if method == "__sam__ping":
            return None, self.lease.ping(session, now)
        return None, self.lease.release(session, now)
```

- [ ] Run the file's tests and watch them pass: `bash -c 'source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_supervisor:$PYTHONPATH python3 -m pytest rover/src/navi_supervisor/test/test_navi_rpc_protocol.py -q -p no:cacheprovider'` → expect **26 passed**.

- [ ] Run the whole package suite: `bash -c 'source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_supervisor:$PYTHONPATH python3 -m pytest rover/src/navi_supervisor/test -q -p no:cacheprovider'` → expect **75 passed** (49 + 26).

- [ ] Commit: `git add rover/src/navi_supervisor/navi_supervisor/navi_rpc_protocol.py rover/src/navi_supervisor/test/test_navi_rpc_protocol.py rover/src/navi_supervisor/test/fake_coordinator.py && git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "The rpclib wire served rather than called, with the __sam__ lease the coordinator needs before it will believe NaVi exists"`

---

## Task 2: the NaVi contract (`navi_rpc_state.py`)

**Files:**
- Create: `rover/src/navi_supervisor/navi_supervisor/navi_rpc_state.py`
- Test: `rover/src/navi_supervisor/test/test_navi_rpc_state.py`

**Interfaces:**
- Produces: `navi_supervisor.navi_rpc_state.NaviRpcState(clock)` with `init()`, `set_targets(targets)`, `start_navigation()`, `is_target_reached() -> bool`, `stop_navigation()`, `set_movement_enabled(enable)`, `on_progress(event, index=None, reason=None)`, `on_mode(mode)`, `snapshot() -> dict`, `take_actions() -> list[str]`; constants `TAG_WAYPOINT_REACHED = 0x31`, `TAG_DESTINATION_REACHED = 0x32`, `PUBLISH_TARGETS`, `CHASSIS_STOP`, `NOTIFY_WAYPOINT`, `NOTIFY_DESTINATION`, `WAYPOINT_REACHED`, `DESTINATION_REACHED`, `NAVIGATION_FAILED`, `MAX_TARGETS = 64`. `snapshot()` carries `stop_seq` and `stop_requested`; there is deliberately **no mode-request action** and no mode constant — `on_mode` only records what `/mode_status` last said, for the status payload.
- Consumes: `navi_supervisor.navi_rpc_protocol.RpcRefusal` (for the `F1`/`F2`/`F8`/`F9` stubs, added in Task 3). No ROS.

### Steps

- [ ] Write the failing test at `rover/src/navi_supervisor/test/test_navi_rpc_state.py`:

```python
"""What the coordinator's calls mean, with time passed in rather than read -
the same fake-clock shape test_supervisor_state.py and test_bema_session.py
use. No sockets and no ROS here; Task 3 puts this behind the wire."""

import pytest

from navi_supervisor.navi_rpc_state import (CHASSIS_STOP, DESTINATION_REACHED,
                                            MAX_TARGETS, NAVIGATION_FAILED,
                                            NOTIFY_DESTINATION,
                                            NOTIFY_WAYPOINT, PUBLISH_TARGETS,
                                            TAG_DESTINATION_REACHED,
                                            TAG_WAYPOINT_REACHED,
                                            WAYPOINT_REACHED, NaviRpcState)


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def _state(mode=None):
    clock = Clock()
    state = NaviRpcState(clock=clock)
    if mode is not None:
        state.on_mode(mode)
    return state, clock


def test_the_tags_are_the_coordinators_own():
    assert TAG_WAYPOINT_REACHED == 0x31
    assert TAG_DESTINATION_REACHED == 0x32


def test_set_targets_stores_them_and_asks_for_a_path():
    state, clock = _state()
    state.set_targets([[1.0, 2.0, 0.0], (3.0, 4.0, 1.57)])
    assert state.snapshot()["targets"] == [[1.0, 2.0, 0.0], [3.0, 4.0, 1.57]]
    assert state.take_actions() == [PUBLISH_TARGETS]


def test_set_targets_rejects_an_empty_list():
    state, clock = _state()
    with pytest.raises(ValueError):
        state.set_targets([])


def test_set_targets_rejects_malformed_tuples():
    state, clock = _state()
    for bad in ([[1.0, 2.0]], [[1.0, 2.0, 3.0, 4.0]], [[1.0, 2.0, "x"]],
                [1.0, 2.0, 3.0], "targets", [[1.0, 2.0, float("nan")]],
                [[1.0, 2.0, True]]):
        with pytest.raises(ValueError):
            state.set_targets(bad)


def test_set_targets_rejects_an_absurd_list():
    state, clock = _state()
    with pytest.raises(ValueError):
        state.set_targets([[0.0, 0.0, 0.0]] * (MAX_TARGETS + 1))


def test_set_targets_clears_the_progress_of_the_previous_run():
    state, clock = _state()
    state.set_targets([[1.0, 2.0, 0.0]])
    state.start_navigation()
    state.on_progress(DESTINATION_REACHED, index=0)
    state.take_actions()
    state.set_targets([[5.0, 6.0, 0.0]])
    snapshot = state.snapshot()
    assert snapshot["target_reached"] is False
    assert snapshot["last_point_reached"] is False
    assert snapshot["navigation_requested"] is False
    assert snapshot["reached_index"] is None


def test_start_navigation_without_targets_is_refused():
    state, clock = _state()
    with pytest.raises(ValueError):
        state.start_navigation()


def test_start_navigation_arms_the_run_and_bumps_the_sequence():
    state, clock = _state()
    state.set_targets([[1.0, 2.0, 0.0]])
    state.take_actions()
    clock.t = 12.0
    state.start_navigation()
    snapshot = state.snapshot()
    assert snapshot["navigation_requested"] is True
    assert snapshot["start_seq"] == 1
    assert snapshot["started_at"] == 12.0
    assert state.take_actions() == []          # a run is never started here


def test_is_target_reached_is_a_real_bool_and_follows_progress():
    state, clock = _state()
    state.set_targets([[1.0, 2.0, 0.0], [3.0, 4.0, 0.0]])
    state.start_navigation()
    assert state.is_target_reached() is False
    state.on_progress(WAYPOINT_REACHED, index=0)
    assert state.is_target_reached() is True
    state.start_navigation()                   # the next leg
    assert state.is_target_reached() is False


def test_a_waypoint_reached_asks_for_the_waypoint_tag():
    state, clock = _state()
    state.set_targets([[1.0, 2.0, 0.0], [3.0, 4.0, 0.0]])
    state.start_navigation()
    state.take_actions()
    state.on_progress(WAYPOINT_REACHED, index=0)
    assert state.take_actions() == [NOTIFY_WAYPOINT]
    assert state.snapshot()["reached_index"] == 0
    assert state.snapshot()["navigation_requested"] is True


def test_the_destination_ends_the_run_and_asks_for_the_destination_tag():
    state, clock = _state()
    state.set_targets([[1.0, 2.0, 0.0]])
    state.start_navigation()
    state.take_actions()
    state.on_progress(DESTINATION_REACHED, index=0)
    assert state.take_actions() == [NOTIFY_DESTINATION]
    snapshot = state.snapshot()
    assert snapshot["last_point_reached"] is True
    assert snapshot["navigation_requested"] is False


def test_two_waypoints_in_one_batch_are_two_notifies():
    state, clock = _state()
    state.set_targets([[1.0, 2.0, 0.0], [3.0, 4.0, 0.0], [5.0, 6.0, 0.0]])
    state.start_navigation()
    state.take_actions()
    state.on_progress(WAYPOINT_REACHED, index=0)
    state.on_progress(WAYPOINT_REACHED, index=1)
    assert state.take_actions() == [NOTIFY_WAYPOINT, NOTIFY_WAYPOINT]


def test_a_failure_invents_no_completion():
    state, clock = _state()
    state.set_targets([[1.0, 2.0, 0.0]])
    state.start_navigation()
    state.take_actions()
    state.on_progress(NAVIGATION_FAILED, reason="planner gave up")
    assert state.take_actions() == [CHASSIS_STOP]
    snapshot = state.snapshot()
    assert snapshot["navigation_requested"] is False
    assert snapshot["last_error"] == "planner gave up"
    assert snapshot["last_point_reached"] is False
    # The failure path is the SAME stop path as F6 and F7(false): one stop,
    # one bumped counter, whichever of the three fired.
    assert snapshot["stop_seq"] == 1
    assert snapshot["stop_requested"] is True


def test_an_unknown_progress_event_is_recorded_and_ignored():
    state, clock = _state()
    state.set_targets([[1.0, 2.0, 0.0]])
    state.start_navigation()
    state.take_actions()
    state.on_progress("teleported", index=0)
    assert state.take_actions() == []
    assert "teleported" in state.snapshot()["last_error"]
    assert state.snapshot()["navigation_requested"] is True


def test_stop_navigation_stops_the_chassis_and_bumps_the_stop_seq():
    state, clock = _state(mode="autonomous")
    state.set_targets([[1.0, 2.0, 0.0]])
    state.start_navigation()
    state.take_actions()
    state.stop_navigation()
    assert state.take_actions() == [CHASSIS_STOP]
    snapshot = state.snapshot()
    assert snapshot["navigation_requested"] is False
    assert snapshot["stop_seq"] == 1
    assert snapshot["stop_requested"] is True


def test_stop_navigation_is_the_same_from_every_mode():
    # No mode ever changes what F6 does, and it never asks for one. Asking
    # for `manual` would clear a latched e-stop (SupervisorState.
    # on_mode_request falls through to _estop_latched = False for a manual
    # request), and from `autonomous` it would turn the coordinator's PAUSE
    # into a full ABORT: CoordinatorImpl::pause() calls F6 while it is still
    # in Autonomous, so the supervisor would answer with COORDINATOR_ABORT
    # and the run would be unrecoverable.
    for mode in ("autonomous", "estop", "manual", None):
        state, clock = _state(mode=mode)
        state.set_targets([[1.0, 2.0, 0.0]])
        state.start_navigation()
        state.take_actions()
        state.stop_navigation()
        assert state.take_actions() == [CHASSIS_STOP], mode
        assert state.snapshot()["stop_seq"] == 1, mode


def test_a_new_run_clears_the_stop_flag_but_never_the_counter():
    state, clock = _state(mode="autonomous")
    state.set_targets([[1.0, 2.0, 0.0]])
    state.start_navigation()
    state.stop_navigation()
    state.take_actions()
    assert state.snapshot()["stop_requested"] is True
    state.start_navigation()
    snapshot = state.snapshot()
    assert snapshot["stop_requested"] is False
    # Monotonic: goal_relay dedupes on the counter, so it must never go back.
    assert snapshot["stop_seq"] == 1


def test_a_second_stop_bumps_the_sequence_again():
    state, clock = _state(mode="autonomous")
    state.set_targets([[1.0, 2.0, 0.0]])
    state.start_navigation()
    state.take_actions()
    state.stop_navigation()
    assert state.take_actions() == [CHASSIS_STOP]
    state.stop_navigation()
    assert state.take_actions() == [CHASSIS_STOP]
    assert state.snapshot()["stop_seq"] == 2


def test_movement_disabled_stops_the_run_and_movement_enabled_does_not_gate_it():
    state, clock = _state(mode="autonomous")
    state.set_targets([[1.0, 2.0, 0.0]])
    state.take_actions()
    state.start_navigation()                   # never told movement is enabled
    assert state.snapshot()["navigation_requested"] is True
    state.take_actions()
    state.set_movement_enabled(False)
    assert state.take_actions() == [CHASSIS_STOP]
    assert state.snapshot()["navigation_requested"] is False
    assert state.snapshot()["stop_seq"] == 1
    assert state.snapshot()["movement_enabled"] is False
    state.set_movement_enabled(True)
    assert state.take_actions() == []
    assert state.snapshot()["movement_enabled"] is True


def test_init_is_recorded_and_changes_nothing_else():
    state, clock = _state()
    state.set_targets([[1.0, 2.0, 0.0]])
    state.take_actions()
    state.init()
    assert state.snapshot()["inited"] is True
    assert state.snapshot()["targets"] == [[1.0, 2.0, 0.0]]
    assert state.take_actions() == []


def test_the_snapshot_records_the_last_call_and_its_age():
    state, clock = _state()
    clock.t = 5.0
    state.init()
    clock.t = 7.5
    snapshot = state.snapshot()
    assert snapshot["last_method"] == "init"
    assert snapshot["last_call_age_s"] == 2.5


def test_the_snapshot_is_json_serialisable():
    import json
    state, clock = _state(mode="autonomous")
    state.set_targets([[1.0, 2.0, 0.5]])
    state.start_navigation()
    state.on_progress(WAYPOINT_REACHED, index=0)
    json.dumps(state.snapshot())
```

- [ ] Run it and watch it fail: same command with `test_navi_rpc_state.py` → `ModuleNotFoundError: No module named 'navi_supervisor.navi_rpc_state'`.

- [ ] Implement `rover/src/navi_supervisor/navi_supervisor/navi_rpc_state.py`:

```python
"""What the coordinator's NaVi calls mean on this side, and nothing else.

`navi_rpc_protocol.py` owns the wire; this owns the contract: the waypoint
list, the run flags, and the side effects a call implies. Like
supervisor_state.py it queues actions rather than performing them - the node
drains them on a timer, because an RPC arrives on a socket thread and rclpy
publishers may only be touched from the executor's.

It is locked, because those socket threads are real: every public method
takes the same RLock, `snapshot()` and `take_actions()` included.

Two things this file deliberately does NOT do:

  * `start_navigation` does not start anything. Runs are operator-initiated
    (spec 3); the coordinator's F4 is the signal that it has reached
    Autonomous and the run is armed, which SP11's goal_relay waits for
    before it sends a Nav2 goal.
  * `movement_enabled` never gates a run. The coordinator's
    movementUpdateLoop for NaVi is commented out (CoordinatorImpl.cpp:38),
    so F7 may never arrive at all; a run gated on it would never start.
    Only the transition to False has an effect, and that effect is a stop.
  * no stop of any kind asks for a mode. The supervisor is the single
    authority on mode (SP5), and a `manual` request from here would both
    clear a latched e-stop and turn the coordinator's pause into an abort.
    A stop bumps stop_seq instead; see _stop_actions().
"""

import math
import threading
from time import monotonic

# navi::TAG_WaypointReached / TAG_DestinationReached, from NaVi.idl.
TAG_WAYPOINT_REACHED = 0x31
TAG_DESTINATION_REACHED = 0x32

# The side effects the node performs on this class's behalf. There is
# deliberately no "request manual" among them - see _stop_actions().
PUBLISH_TARGETS = "publish_targets"        # /navi_rpc/targets (nav_msgs/Path)
CHASSIS_STOP = "chassis_stop"              # /drive_command {"action": "stop"}
NOTIFY_WAYPOINT = "notify_waypoint"        # coordinator F8, tag 0x31
NOTIFY_DESTINATION = "notify_destination"  # coordinator F8, tag 0x32

# Deduplicated within one batch; the notifies never are, because two
# waypoints reached between two drains are two things the coordinator has to
# be told about.
_DEDUPED = (PUBLISH_TARGETS, CHASSIS_STOP)

# The vocabulary of /navi_rpc/progress, which SP11's goal_relay publishes.
WAYPOINT_REACHED = "waypoint_reached"
DESTINATION_REACHED = "destination_reached"
NAVIGATION_FAILED = "failed"
PROGRESS_EVENTS = (WAYPOINT_REACHED, DESTINATION_REACHED, NAVIGATION_FAILED)

# An ERC run is a handful of waypoints. A list longer than this is not a
# coordinator, and storing it would be the whole attack.
MAX_TARGETS = 64


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"target components must be numbers, got {value!r}")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("target components must be finite")
    return value


def parse_targets(targets):
    """The wire's [[x, y, w], ...] as a list of (x, y, w) floats, or ValueError.

    `w` is yaw in radians in the map frame - see the plan's rulings: the IDL
    gives no unit, and we are the only producer end to end.
    """
    if isinstance(targets, (str, bytes)) or not isinstance(targets, (list, tuple)):
        raise ValueError("setTargets takes an array of (x, y, w) targets")
    if not targets:
        raise ValueError("setTargets needs at least one target")
    if len(targets) > MAX_TARGETS:
        raise ValueError(f"setTargets takes at most {MAX_TARGETS} targets, "
                         f"got {len(targets)}")
    parsed = []
    for target in targets:
        if (isinstance(target, (str, bytes))
                or not isinstance(target, (list, tuple)) or len(target) != 3):
            raise ValueError(f"a target must be (x, y, w), got {target!r}")
        parsed.append(tuple(_number(component) for component in target))
    return parsed


class NaviRpcState:

    def __init__(self, clock=monotonic):
        self._clock = clock
        self._lock = threading.RLock()
        self._targets = []
        self._inited = False
        self._navigation_requested = False
        self._start_seq = 0
        self._started_at = None
        self._target_reached = False
        self._reached_index = None
        self._last_point_reached = False
        self._movement_enabled = False
        self._stop_seq = 0
        self._stop_requested = False
        self._last_method = None
        self._last_call_at = None
        self._last_error = None
        self._mode = None
        self._actions = []

    # --- the served methods ---------------------------------------------
    def init(self):
        with self._lock:
            self._inited = True
            self._mark("init")

    def set_targets(self, targets):
        parsed = parse_targets(targets)
        with self._lock:
            self._targets = parsed
            self._navigation_requested = False
            self._target_reached = False
            self._reached_index = None
            self._last_point_reached = False
            self._last_error = None
            self._mark("setTargets")
            self._queue(PUBLISH_TARGETS)

    def start_navigation(self):
        with self._lock:
            if not self._targets:
                self._mark("startNavigation")
                raise ValueError("no targets set")
            self._navigation_requested = True
            self._start_seq += 1
            self._started_at = self._clock()
            # A new run answers the last stop. The counter itself is
            # monotonic - goal_relay dedupes on it - so only the flag clears.
            self._stop_requested = False
            # The per-leg latch. Without clearing it, F5 would answer true
            # for a leg that has not been driven yet - the coordinator calls
            # F4 again the instant it is told a waypoint was reached.
            self._target_reached = False
            self._mark("startNavigation")

    def is_target_reached(self):
        with self._lock:
            self._mark("isTargetReached")
            return bool(self._target_reached)

    def stop_navigation(self):
        with self._lock:
            self._navigation_requested = False
            self._target_reached = False
            self._mark("stopNavigation")
            self._stop_actions()

    def set_movement_enabled(self, enable):
        if not isinstance(enable, bool):
            raise ValueError("setMovementEnabled takes a bool")
        with self._lock:
            self._movement_enabled = enable
            self._mark("setMovementEnabled")
            if not enable:
                self._navigation_requested = False
                self._target_reached = False
                self._stop_actions()

    # --- inputs from the ROS side ---------------------------------------
    def on_progress(self, event, index=None, reason=None):
        with self._lock:
            if event == WAYPOINT_REACHED:
                self._target_reached = True
                self._reached_index = None if index is None else int(index)
                self._queue(NOTIFY_WAYPOINT)
            elif event == DESTINATION_REACHED:
                self._target_reached = True
                self._last_point_reached = True
                self._navigation_requested = False
                self._reached_index = None if index is None else int(index)
                self._queue(NOTIFY_DESTINATION)
            elif event == NAVIGATION_FAILED:
                # No completion is invented: the coordinator has no failure
                # tag, and both of the tags it has advance its state machine
                # as if we had arrived. The operator aborts.
                self._navigation_requested = False
                self._target_reached = False
                self._last_error = str(reason) if reason else "navigation failed"
                # The same stop path as F6 and F7(false), not a weaker one:
                # goal_relay watches stop_seq, so all three must move it.
                self._stop_actions()
            else:
                self._last_error = f"unknown progress event {event!r}"

    def on_mode(self, mode):
        with self._lock:
            self._mode = mode if isinstance(mode, str) else None

    # --- output ----------------------------------------------------------
    def snapshot(self):
        with self._lock:
            now = self._clock()
            age = (None if self._last_call_at is None
                   else round(now - self._last_call_at, 2))
            return {
                "inited": self._inited,
                "targets": [list(target) for target in self._targets],
                "target_count": len(self._targets),
                "navigation_requested": self._navigation_requested,
                "start_seq": self._start_seq,
                "started_at": self._started_at,
                "target_reached": self._target_reached,
                "reached_index": self._reached_index,
                "last_point_reached": self._last_point_reached,
                "movement_enabled": self._movement_enabled,
                # What SP11's goal_relay watches to cancel a Nav2 goal. The
                # counter is monotonic so a missed status message cannot lose
                # a stop; the flag says whether the last stop still stands.
                "stop_seq": self._stop_seq,
                "stop_requested": self._stop_requested,
                "last_method": self._last_method,
                "last_call_age_s": age,
                "last_error": self._last_error,
                "supervisor_mode": self._mode,
            }

    def take_actions(self):
        with self._lock:
            actions, self._actions = self._actions, []
            return actions

    # --- internals -------------------------------------------------------
    def _mark(self, method):
        self._last_method = method
        self._last_call_at = self._clock()

    def _stop_actions(self):
        """The one stop path, shared by F6, F7(false) and a failed run.

        A chassis stop and a bumped counter - never a mode request. Two
        reasons, either of which is sufficient:

          * SupervisorState.on_mode_request clears the e-stop latch for an
            accepted `manual`, so the primary's state machine could clear
            our operator's e-stop.
          * CoordinatorImpl::pause() calls F6 while it is still in
            Autonomous. A `manual` request there comes back through the
            supervisor as CANCEL_GOAL/DEACTIVATE_NAV2/COORDINATOR_ABORT, so
            the operator's pause would silently become an abort and Resume
            would be refused from Idle.

        Mode belongs to the supervisor, alone (SP5). What actually stops
        /autonomy_twist is SP11's goal_relay cancelling the Nav2 goal when
        it sees stop_seq move.
        """
        self._queue(CHASSIS_STOP)
        self._stop_seq += 1
        self._stop_requested = True

    def _queue(self, *actions):
        for action in actions:
            if action in _DEDUPED and action in self._actions:
                continue
            self._actions.append(action)
```

- [ ] Run the file's tests and watch them pass → expect **22 passed**.

- [ ] Run the package suite → expect **97 passed** (49 + 26 + 22).

- [ ] Commit: `git add rover/src/navi_supervisor/navi_supervisor/navi_rpc_state.py rover/src/navi_supervisor/test/test_navi_rpc_state.py && git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "The NaVi contract as a locked, fake-clock state object: targets, run flags, and the side effects a coordinator call implies"`

---

## Task 3: the method table, and `startNaViTask` end to end over a socket

**Files:**
- Modify: `rover/src/navi_supervisor/navi_supervisor/navi_rpc_state.py` (add `navi_method_table`, `GUARDED_METHODS`, `STUBS`)
- Test: `rover/src/navi_supervisor/test/test_navi_rpc_state.py` (append the end-to-end block)

**Interfaces:**
- Produces: `navi_method_table(state, logger=None) -> dict[str, callable]` binding `F0`–`F9`; `GUARDED_METHODS = frozenset({"F0", "F1", "F3", "F4", "F5", "F6", "F7"})`; `STUBS` mapping the four unserved methods to their names; `IDENTITY_POSE`. `F1` (guarded) refuses by name; the three unguarded ones answer benign, correctly-shaped values and log the call.
- Consumes: `RpcRefusal` from `navi_rpc_protocol`, `RpcServer` (in the test only).

### Steps

- [ ] Append the failing tests to `rover/src/navi_supervisor/test/test_navi_rpc_state.py`:

```python
# --- the method table, behind the real wire ------------------------------
from fake_coordinator import FakeCoordinator, RpcError            # noqa: E402
from navi_supervisor.navi_rpc_protocol import (ACCESS_DENIED_ERROR,  # noqa: E402
                                               RpcServer)
from navi_supervisor.navi_rpc_state import (GUARDED_METHODS,       # noqa: E402
                                            IDENTITY_POSE,
                                            navi_method_table)


@pytest.fixture
def served():
    clock = Clock()
    state = NaviRpcState(clock=clock)
    server = RpcServer(navi_method_table(state), guarded=GUARDED_METHODS,
                       host="127.0.0.1", port=0, clock=clock)
    server.start()
    client = FakeCoordinator("127.0.0.1", server.port)
    yield state, clock, client
    client.close()
    server.stop()


def test_the_guarded_set_matches_the_idl():
    assert GUARDED_METHODS == frozenset({"F0", "F1", "F3", "F4", "F5",
                                         "F6", "F7"})


def test_the_start_navi_task_sequence_the_coordinator_performs(served):
    state, clock, client = served
    # AutoConnection::getCapability() -> Client::accessServer()
    assert client.access() is True
    # the naviIniter thread's init()
    assert client.call("F0") is None
    # startNaViTask -> setTargets(waypoints)
    assert client.call("F3", [[1.0, 2.0, 0.0], [3.0, 4.0, 1.5]]) is None
    # 5 s later, onAutonomousEntered -> startNavigation_async()
    assert client.call("F4") is None
    assert client.call("F5") is False
    snapshot = state.snapshot()
    assert snapshot["inited"] is True
    assert snapshot["navigation_requested"] is True
    assert snapshot["targets"] == [[1.0, 2.0, 0.0], [3.0, 4.0, 1.5]]


def test_every_guarded_method_is_refused_with_error_one_without_the_lease(served):
    state, clock, client = served
    for method, args in (("F0", ()), ("F1", (1.0, 2.0)),
                         ("F3", ([[1.0, 2.0, 0.0]],)), ("F4", ()),
                         ("F5", ()), ("F6", ()), ("F7", (True,))):
        with pytest.raises(RpcError) as excinfo:
            client.call(method, *args)
        assert excinfo.value.error == ACCESS_DENIED_ERROR, method


def test_the_unguarded_stubs_answer_safely_and_need_no_lease(served):
    state, clock, client = served
    # WeakNaViEP::getPosition/getTofData/takeSnapshot are the three methods
    # whose mStub->call() is NOT wrapped in try/catch (WeakNaViEP.h:28-58),
    # so an error reply becomes an rpc::rpc_error in the caller's own thread
    # - std::terminate if it escapes a thread function. We hold the .18 alias
    # for the whole rover LAN, so anything that used to poll the real NaVi
    # for a pose, a ToF frame or a snapshot now reaches us. Shaped emptiness
    # is readable as "no data"; an exception is not.
    pose = client.call("F2")
    assert [list(row) for row in pose] == IDENTITY_POSE
    assert client.call("F8") == []
    assert client.call("F9", 0) is None


def test_set_position_refuses_even_with_the_lease(served):
    state, clock, client = served
    client.access()
    with pytest.raises(RpcError) as excinfo:
        client.call("F1", 1.0, 2.0)
    assert "not served" in str(excinfo.value.error)


def test_bad_targets_on_the_wire_are_an_error_not_a_dead_server(served):
    state, clock, client = served
    client.access()
    with pytest.raises(RpcError):
        client.call("F3", [[1.0, 2.0]])
    with pytest.raises(RpcError):
        client.call("F3", [])
    assert client.call("F3", [[1.0, 2.0, 0.0]]) is None


def test_is_target_reached_answers_a_bool_over_the_wire(served):
    state, clock, client = served
    client.access()
    client.call("F3", [[1.0, 2.0, 0.0]])
    client.call("F4")
    assert client.call("F5") is False
    state.on_progress(DESTINATION_REACHED, index=0)
    assert client.call("F5") is True


def test_set_movement_enabled_takes_a_bool_and_nothing_else(served):
    state, clock, client = served
    client.access()
    assert client.call("F7", False) is None
    with pytest.raises(RpcError):
        client.call("F7", 1)
```

- [ ] Run and watch them fail → `ImportError: cannot import name 'navi_method_table'`.

- [ ] Implement: append to `rover/src/navi_supervisor/navi_supervisor/navi_rpc_state.py` (and add `from navi_supervisor.navi_rpc_protocol import RpcRefusal` to its imports):

```python
# --- the method table -----------------------------------------------------
#
# Names, arities and guard flags come from coordinator/deps/naviInterface:
# NaViProxy binds F0-F12, and NaVi.idl marks F2, F8 and F9 @noguard. The
# camera methods F10-F12 are deliberately not bound - spec 3 lists F0-F9 as
# the interface, and the pan/tilt/zoom head is not ours; an unknown-method
# error beats a hang.

GUARDED_METHODS = frozenset({"F0", "F1", "F3", "F4", "F5", "F6", "F7"})

STUBS = {
    "F1": "setPosition",
    "F2": "getPosition",
    "F8": "getTofData",
    "F9": "takeSnapshot",
}

# What F2 answers. NaViEP::getPosition() unpacks the reply as a
# std::array<std::array<float,4>,4>, so a 4x4 identity is the correctly
# shaped way to say "no pose information from here".
IDENTITY_POSE = [[1.0, 0.0, 0.0, 0.0],
                 [0.0, 1.0, 0.0, 0.0],
                 [0.0, 0.0, 1.0, 0.0],
                 [0.0, 0.0, 0.0, 1.0]]


def _refusal(method):
    return RpcRefusal(f"{method} ({STUBS[method]}) is not served by "
                      f"navi_rpc_server")


def navi_method_table(state, logger=None):
    """The F-methods, bound to a NaviRpcState. Arities match the IDL, so a
    call with the wrong number of arguments is answered as bad arguments
    rather than silently accepted.

    F1 is guarded, and NaViEP::setPosition() catches rpc::rpc_error, so it
    refuses by name. F2, F8 and F9 are @noguard, and their client half -
    WeakNaViEP::getPosition/getTofData/takeSnapshot - does NOT catch: an
    error reply there propagates as rpc::rpc_error into whatever thread
    made the call, and std::terminate if it escapes a thread function.
    Since the .18 alias makes us the NaVi endpoint for the whole rover LAN,
    anything that used to poll the real NaVi for a pose, a ToF frame or a
    snapshot now reaches us - so those three answer benign, correctly-shaped
    values (an identity pose, an empty frame, nil) and log the call. That is
    deliberate: "no data" is readable, an exception is not.
    """

    def _note(method):
        if logger is None:
            return
        try:
            logger.info(f"{method} ({STUBS[method]}) is not served by "
                        f"navi_rpc_server; answering with an empty value")
        except Exception:                       # a logger must not kill us
            pass

    def f0():
        state.init()

    def f1(x, y):
        raise _refusal("F1")

    def f2():
        _note("F2")
        return IDENTITY_POSE

    def f3(targets):
        state.set_targets(targets)

    def f4():
        state.start_navigation()

    def f5():
        return state.is_target_reached()

    def f6():
        state.stop_navigation()

    def f7(enable):
        state.set_movement_enabled(enable)

    def f8():
        _note("F8")
        return []                               # an empty std::vector<float>

    def f9(index):
        _note("F9")
        return None

    return {"F0": f0, "F1": f1, "F2": f2, "F3": f3, "F4": f4,
            "F5": f5, "F6": f6, "F7": f7, "F8": f8, "F9": f9}
```

- [ ] Run the file's tests → expect **30 passed** (22 + 8).

- [ ] Run the package suite → expect **105 passed**.

- [ ] Commit: `git add rover/src/navi_supervisor/navi_supervisor/navi_rpc_state.py rover/src/navi_supervisor/test/test_navi_rpc_state.py && git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "F0-F9 bound to the contract: the startNaViTask sequence answered end to end over a loopback socket, F1 refused by name and the unguarded three answered with shaped emptiness"`

---

## Task 4: the `navi_rpc_server` node

**Files:**
- Create: `rover/src/navi_supervisor/navi_supervisor/navi_rpc_server.py`
- Test: `rover/src/navi_supervisor/test/test_navi_rpc_server.py`
- Modify: `rover/src/navi_supervisor/setup.py`, `rover/src/navi_supervisor/package.xml`

**Interfaces:**
- Produces:
  - `/navi_rpc/targets` — `nav_msgs/Path`, reliable + **transient_local**, depth 1. `header.frame_id` = the `frame_id` parameter (`"map"`); one `PoseStamped` per target, `position.x/y` from the tuple, `z = 0.0`, orientation = yaw `w` about z (`qz = sin(w/2)`, `qw = cos(w/2)`).
  - `/navi_rpc/status` — `std_msgs/String`, JSON of `NaviRpcState.snapshot()` plus `"listening": "<host>:<port>"`. 2 Hz and on every action. `stop_seq` (a monotonic counter) and `stop_requested` are the fields SP11's `goal_relay` watches to cancel a Nav2 goal.
  - `/drive_command` — `std_msgs/String`, `{"action": "stop"}` and `{"action": "task_finished", "tag": 49|50}`, depth 10.
  - **No `/mode_request`.** The supervisor is the single authority on mode (SP5), and a `manual` request from here would both clear a latched e-stop and turn the coordinator's *pause* into an *abort* — see the ruling on `stopNavigation`. `test_the_server_never_publishes_a_mode_request` pins it.
- Consumes:
  - `/navi_rpc/progress` — `std_msgs/String`, JSON `{"event": "waypoint_reached"|"destination_reached"|"failed", "index": int|null, "reason": str|null}`. **SP11's `goal_relay` is the publisher.**
  - `/mode_status` — `std_msgs/String`, JSON, key `"mode"` (published by `mode_supervisor`).
  - The msgpack-RPC wire on `bind_host:port` (`0.0.0.0:21021`).
- Parameters: `bind_host` (`"0.0.0.0"`), `port` (`21021`), `frame_id` (`"map"`), `targets_topic` (`"/navi_rpc/targets"`), `status_topic` (`"/navi_rpc/status"`), `progress_topic` (`"/navi_rpc/progress"`).

### Steps

- [ ] Write the failing test at `rover/src/navi_supervisor/test/test_navi_rpc_server.py`:

```python
import json
import os

os.environ.setdefault("ROS_DOMAIN_ID", "91")   # throwaway; never the rover's

import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String

from fake_coordinator import FakeCoordinator, RpcError
from navi_supervisor.navi_rpc_server import NaviRpcServer


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
def node():
    clock = Clock()
    server = NaviRpcServer(clock=clock, parameter_overrides=[
        Parameter("bind_host", Parameter.Type.STRING, "127.0.0.1"),
        Parameter("port", Parameter.Type.INTEGER, 0)])
    paths, statuses, commands = [], [], []
    server._targets_pub.publish = lambda msg: paths.append(msg)
    server._status_pub.publish = lambda msg: statuses.append(json.loads(msg.data))
    server._command_pub.publish = lambda msg: commands.append(json.loads(msg.data))
    client = FakeCoordinator("127.0.0.1", server.port)
    yield server, clock, client, paths, statuses, commands
    client.close()
    server.destroy_node()


def _string(payload):
    msg = String()
    msg.data = json.dumps(payload)
    return msg


def test_the_server_binds_loopback_on_an_ephemeral_port(node):
    server, clock, client, paths, statuses, commands = node
    assert server.port != 0
    assert server.port != 21021


def test_set_targets_publishes_a_latched_path_in_the_map_frame(node):
    server, clock, client, paths, statuses, commands = node
    client.access()
    client.call("F3", [[1.0, 2.0, 0.0], [3.0, 4.0, 3.141592653589793]])
    server._action_tick()
    path = paths[-1]
    assert path.header.frame_id == "map"
    assert len(path.poses) == 2
    assert path.poses[0].header.frame_id == "map"
    assert path.poses[0].pose.position.x == pytest.approx(1.0)
    assert path.poses[0].pose.position.y == pytest.approx(2.0)
    assert path.poses[0].pose.orientation.z == pytest.approx(0.0)
    assert path.poses[0].pose.orientation.w == pytest.approx(1.0)
    assert path.poses[1].pose.orientation.z == pytest.approx(1.0)
    assert path.poses[1].pose.orientation.w == pytest.approx(0.0, abs=1e-9)


def test_start_navigation_shows_up_in_the_status_and_starts_nothing(node):
    server, clock, client, paths, statuses, commands = node
    client.access()
    client.call("F3", [[1.0, 2.0, 0.0]])
    client.call("F4")
    server._action_tick()
    server._status_tick()
    assert statuses[-1]["navigation_requested"] is True
    assert statuses[-1]["start_seq"] == 1
    assert statuses[-1]["listening"].startswith("127.0.0.1:")
    assert commands == []            # no run is commanded from here


def test_a_reached_waypoint_becomes_the_coordinator_tag(node):
    server, clock, client, paths, statuses, commands = node
    client.access()
    client.call("F3", [[1.0, 2.0, 0.0], [3.0, 4.0, 0.0]])
    client.call("F4")
    server._action_tick()
    server._on_progress(_string({"event": "waypoint_reached", "index": 0}))
    assert commands[-1] == {"action": "task_finished", "tag": 0x31}
    assert client.call("F5") is True


def test_the_destination_becomes_the_destination_tag(node):
    server, clock, client, paths, statuses, commands = node
    client.access()
    client.call("F3", [[1.0, 2.0, 0.0]])
    client.call("F4")
    server._action_tick()
    server._on_progress(_string({"event": "destination_reached", "index": 0}))
    assert commands[-1] == {"action": "task_finished", "tag": 0x32}
    server._status_tick()
    assert statuses[-1]["last_point_reached"] is True


def test_a_failure_stops_and_reports_and_invents_no_tag(node):
    server, clock, client, paths, statuses, commands = node
    client.access()
    client.call("F3", [[1.0, 2.0, 0.0]])
    client.call("F4")
    server._action_tick()
    server._on_progress(_string({"event": "failed", "reason": "no valid path"}))
    assert commands[-1] == {"action": "stop"}
    assert all(c.get("action") != "task_finished" for c in commands)
    server._status_tick()
    assert statuses[-1]["last_error"] == "no valid path"


def test_stop_navigation_stops_the_chassis_and_bumps_the_stop_seq(node):
    server, clock, client, paths, statuses, commands = node
    server._on_mode_status(_string({"mode": "autonomous"}))
    client.access()
    client.call("F3", [[1.0, 2.0, 0.0]])
    client.call("F4")
    server._action_tick()
    client.call("F6")
    server._action_tick()
    server._status_tick()
    assert commands[-1] == {"action": "stop"}
    assert statuses[-1]["stop_seq"] == 1
    assert statuses[-1]["stop_requested"] is True
    assert statuses[-1]["navigation_requested"] is False


def test_the_server_never_publishes_a_mode_request(node):
    server, clock, client, paths, statuses, commands = node
    # The regression pin for the pause-becomes-abort trap. CoordinatorImpl::
    # pause() calls F6 while it is still in Autonomous, so a /mode_request
    # from here would come back through the supervisor as COORDINATOR_ABORT
    # and the run would be gone; from estop it would clear the operator's own
    # latch. The supervisor owns mode, alone (SP5).
    assert "/mode_request" not in [p.topic_name for p in server.publishers]
    for mode in ("autonomous", "estop", "manual"):
        server._on_mode_status(_string({"mode": mode}))
        client.access()
        client.call("F3", [[1.0, 2.0, 0.0]])
        client.call("F6")
        server._action_tick()
        assert commands[-1] == {"action": "stop"}


def test_an_unreadable_progress_message_does_not_kill_the_node(node):
    server, clock, client, paths, statuses, commands = node
    bad = String()
    bad.data = "{not json"
    server._on_progress(bad)
    server._on_progress(_string(["not", "a", "dict"]))
    client.access()
    assert client.call("F0") is None


def test_an_unreadable_mode_status_leaves_the_mode_unknown(node):
    server, clock, client, paths, statuses, commands = node
    bad = String()
    bad.data = "nonsense"
    server._on_mode_status(bad)
    client.access()
    client.call("F3", [[1.0, 2.0, 0.0]])
    client.call("F6")
    server._action_tick()
    server._status_tick()
    assert statuses[-1]["supervisor_mode"] is None
    assert commands[-1] == {"action": "stop"}   # the stop goes out regardless


def test_a_hostile_client_leaves_the_node_serving(node):
    server, clock, client, paths, statuses, commands = node
    with pytest.raises(RpcError):
        client.call("F99")
    with pytest.raises(RpcError):
        client.call("F3", "not-an-array")
    client.access()
    assert client.call("F3", [[1.0, 2.0, 0.0]]) is None
    server._status_tick()
    assert statuses[-1]["target_count"] == 1
```

- [ ] Run it and watch it fail → `ModuleNotFoundError: No module named 'navi_supervisor.navi_rpc_server'`.

- [ ] Implement `rover/src/navi_supervisor/navi_supervisor/navi_rpc_server.py`:

```python
"""The endpoint the primary's coordinator calls "NaVi".

`CoordinatorImpl::startNaViTask` hands its waypoints to a msgpack-RPC server
at 192.168.178.18:21021, and drops straight back to Idle when it is not
there. This node is that server: the wire and the lease live in
navi_rpc_protocol.py, the contract in navi_rpc_state.py, and this file owns
the socket's lifetime, the timers, and the topics the rest of the graph
sees.

It never publishes /mode_request. The supervisor is the single authority on
mode (SP5): a `manual` request from here would clear a latched e-stop, and
would turn the coordinator's own pause - which calls F6 while it is still in
Autonomous - into a full abort. F6 stops the chassis and bumps `stop_seq` on
/navi_rpc/status instead; SP11's goal_relay watches that and cancels the Nav2
goal, which is what actually stops /autonomy_twist.

It binds 0.0.0.0 rather than the alias itself, so the node still starts (and
is still testable, and still reachable on .33) on a machine where
start_navi.sh could not add 192.168.178.18. The alias is what makes the
coordinator's hard-coded address land here; the bind is what makes it
answerable.

Nothing here starts a run. F4 startNavigation is the coordinator telling us
it has reached Autonomous, which is the arming signal SP11's goal_relay
waits for - runs themselves are operator-initiated (spec 3).

Calls arrive on socket threads, so the state queues actions and this node
drains them on a timer: an rclpy publisher may only be touched from the
executor's thread.
"""

import json
from math import cos, sin
from time import monotonic

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import String

from navi_supervisor import navi_rpc_state as contract
from navi_supervisor.navi_rpc_protocol import RpcServer
from navi_supervisor.navi_rpc_state import (GUARDED_METHODS, NaviRpcState,
                                            navi_method_table)

ACTION_HZ = 20.0
STATUS_HZ = 2.0

# Latched: goal_relay and the ground station may both start after a run has
# been armed, and must not wait for the next setTargets to learn the route.
LATCHED = QoSProfile(depth=1,
                     history=QoSHistoryPolicy.KEEP_LAST,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def targets_to_path(targets, frame_id, stamp):
    """[(x, y, yaw)] -> nav_msgs/Path. yaw is radians about z, in `frame_id`."""
    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = stamp
    for x, y, yaw in targets:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = stamp
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = sin(float(yaw) / 2.0)
        pose.pose.orientation.w = cos(float(yaw) / 2.0)
        path.poses.append(pose)
    return path


def _default_server_factory(state, host, port, clock, logger):
    server = RpcServer(navi_method_table(state, logger=logger),
                       guarded=GUARDED_METHODS,
                       host=host, port=port, clock=clock, logger=logger)
    server.start()
    return server


class NaviRpcServer(Node):

    def __init__(self, clock=monotonic, server_factory=_default_server_factory,
                 parameter_overrides=None):
        super().__init__("navi_rpc_server",
                         parameter_overrides=parameter_overrides or [])
        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("port", 21021)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("targets_topic", "/navi_rpc/targets")
        self.declare_parameter("status_topic", "/navi_rpc/status")
        self.declare_parameter("progress_topic", "/navi_rpc/progress")

        # NOT self._clock: rclpy.node.Node already owns that name.
        self._now = clock
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._state = NaviRpcState(clock=clock)

        self._targets_pub = self.create_publisher(
            Path, self.get_parameter("targets_topic").value, LATCHED)
        self._status_pub = self.create_publisher(
            String, self.get_parameter("status_topic").value, 1)
        self._command_pub = self.create_publisher(String, "/drive_command", 10)

        self.create_subscription(
            String, self.get_parameter("progress_topic").value,
            self._on_progress, 10)
        self.create_subscription(String, "/mode_status",
                                 self._on_mode_status, 10)

        # A failure to bind is fatal on purpose: a rover whose NaVi endpoint
        # is missing must not look healthy. The usual cause is a stale server
        # still holding :21021 - see start_navi.sh's cleanup.
        try:
            self._server = server_factory(
                self._state, str(self.get_parameter("bind_host").value),
                int(self.get_parameter("port").value), clock, self.get_logger())
        except OSError as exc:
            self.get_logger().error(
                f"could not bind {self.get_parameter('bind_host').value}:"
                f"{self.get_parameter('port').value} ({exc}); the coordinator "
                f"will report 'NaVi not reachable'")
            raise
        self.get_logger().info(
            f"navi_rpc_server is serving {self.listening}")

        self.create_timer(1.0 / ACTION_HZ, self._action_tick)
        self.create_timer(1.0 / STATUS_HZ, self._status_tick)

    @property
    def port(self):
        return self._server.port

    @property
    def listening(self):
        return f"{self._server.host}:{self._server.port}"

    # --- inputs ----------------------------------------------------------
    def _on_progress(self, msg: String):
        try:
            payload = json.loads(msg.data)
            event = payload.get("event")
            index = payload.get("index")
            reason = payload.get("reason")
        except (json.JSONDecodeError, TypeError, AttributeError):
            self.get_logger().warn(f"unreadable progress: {msg.data!r}")
            return
        try:
            self._state.on_progress(event, index=index, reason=reason)
            self._run_actions()
        except Exception as exc:                     # never kill the node
            self.get_logger().error(f"progress callback failed: {exc!r}")

    def _on_mode_status(self, msg: String):
        try:
            mode = json.loads(msg.data).get("mode")
        except (json.JSONDecodeError, TypeError, AttributeError):
            self.get_logger().warn(f"unreadable mode status: {msg.data!r}")
            return
        try:
            self._state.on_mode(mode)
        except Exception as exc:
            self.get_logger().error(f"mode status callback failed: {exc!r}")

    # --- outputs ---------------------------------------------------------
    def _run_actions(self):
        actions = self._state.take_actions()
        if not actions:
            return
        for action in actions:
            try:
                if action == contract.PUBLISH_TARGETS:
                    self._publish_targets()
                elif action == contract.CHASSIS_STOP:
                    self._send_command({"action": "stop"})
                elif action == contract.NOTIFY_WAYPOINT:
                    self._notify(contract.TAG_WAYPOINT_REACHED)
                elif action == contract.NOTIFY_DESTINATION:
                    self._notify(contract.TAG_DESTINATION_REACHED)
                else:
                    self.get_logger().warn(f"unknown rpc action: {action!r}")
            except Exception as exc:
                self.get_logger().error(f"rpc action {action} failed: {exc!r}")
        self._publish_status()

    def _publish_targets(self):
        targets = self._state.snapshot()["targets"]
        self._targets_pub.publish(
            targets_to_path(targets, self._frame_id, self.get_clock().now().to_msg()))

    def _notify(self, tag):
        # The coordinator's F8 (notifyTaskFinished) is unguarded, so it needs
        # no lease - which is why it can travel on bema_bridge's existing
        # session instead of a second client of our own (SP5's precedent).
        self._send_command({"action": "task_finished", "tag": int(tag)})

    def _send_command(self, payload):
        msg = String()
        msg.data = json.dumps(payload)
        self._command_pub.publish(msg)

    def _action_tick(self):
        try:
            self._run_actions()
        except Exception as exc:
            self.get_logger().error(f"action tick failed: {exc!r}")

    def _status_tick(self):
        try:
            self._publish_status()
        except Exception as exc:
            self.get_logger().error(f"status tick failed: {exc!r}")

    def _publish_status(self):
        status = self._state.snapshot()
        status["listening"] = self.listening
        msg = String()
        # default=str for the same reason /drive_status uses it: one odd
        # field must not black out the status the operator reads.
        msg.data = json.dumps(status, default=str)
        self._status_pub.publish(msg)

    def destroy_node(self):
        try:
            self._server.stop()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = NaviRpcServer()
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

- [ ] Add the entry point in `rover/src/navi_supervisor/setup.py` — inside `console_scripts`, after the `mode_supervisor` line:

```python
            'navi_rpc_server = navi_supervisor.navi_rpc_server:main',
```

- [ ] Declare the new dependency in `rover/src/navi_supervisor/package.xml` — after `<depend>geometry_msgs</depend>`:

```xml
  <depend>nav_msgs</depend>
  <!-- navi_rpc_server needs the `msgpack` Python package (pip install
       msgpack); it is not an ament package, so it is documented here rather
       than declared - the same note navi_teleop carries for bema_bridge. -->
```

- [ ] Run the file's tests → expect **11 passed**.

- [ ] Run the package suite → expect **116 passed**.

- [ ] Build: `bash -c 'source /opt/ros/humble/setup.bash && cd rover && colcon build --packages-select navi_supervisor'` → expect `Summary: 1 package finished`.

- [ ] Commit: `git add rover/src/navi_supervisor/navi_supervisor/navi_rpc_server.py rover/src/navi_supervisor/test/test_navi_rpc_server.py rover/src/navi_supervisor/setup.py rover/src/navi_supervisor/package.xml && git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "The navi_rpc_server node: waypoints out as a latched Path, run state out as JSON, progress in, and a stop that bumps stop_seq without ever touching the supervisor's mode"`

---

## Task 5: the coordinator client — `startNaViTask`, pause/resume, and `notifyTaskFinished`

This is the second half of spec §8's SP8 row (*"+ coordinator client for task state"*), and it is the half SP11 assumes exists: `2026-08-31-sp11-gs-nav.md:11` calls `TaskControl`'s real implementation *"SP8's `navi_rpc` client, bound in Task 8"*, and `:1507` calls its own `task_control.py` *"the stopgap that stands in until SP8's navi_rpc client exists"*. `abort` (coordinator `F7`), `startManual` (`F6`) and `getState` (`F9`) already exist in `bema_session.py`; the missing calls are **`F0 startNaViTask(waypoints)`**, **`F4 pause`** and **`F5 resume`**. All three sit behind `checkAccess()` on `CoordinatorProxy` (`CoordinatorProxy.h:20-23`, `:36-43`), so each needs the same just-in-time `__sam__request` that `start_manual()` and `abort()` already perform — which is why that duplicated body is factored into `_coord_guarded()` first.

**Files:**
- Modify: `rover/src/navi_teleop/navi_teleop/bema_session.py`, `rover/src/navi_teleop/navi_teleop/bema_bridge.py`
- Test: `rover/src/navi_teleop/test/fake_bema_server.py`, `rover/src/navi_teleop/test/test_bema_session.py`, `rover/src/navi_teleop/test/test_bema_bridge.py`

**Interfaces:**
- Produces: `BemaSession._coord_guarded(method, *args, label=None)`; `BemaSession.start_navi_task(waypoints)` → coordinator `F0`, `pause_task()` → `F4`, `resume_task()` → `F5`, `notify_task_finished(tag)` → `F8`; `bema_bridge` accepts `{"action": "task_finished", "tag": 49|50}`, `{"action": "navi_task", "waypoints": [[x, y, w], ...]}`, `{"action": "pause_task"}` and `{"action": "resume_task"}` on `/drive_command`.
- Consumes: `/drive_command` messages published by `navi_rpc_server` (Task 4) and, from SP11, by `goal_relay`.

### Steps

- [ ] Add the failing tests. In `rover/src/navi_teleop/test/test_bema_session.py`, append:

```python
def test_notify_task_finished_calls_the_coordinators_f8(server):
    clock = Clock()
    sess = _session(server, clock)
    server.calls.clear()
    sess.notify_task_finished(0x31)
    assert ("coord", "F8", [0x31]) in server.calls


def test_notify_task_finished_needs_no_coordinator_lease(server):
    clock = Clock()
    sess = _session(server, clock)
    server.coord_lease_held = False
    server.calls.clear()
    sess.notify_task_finished(0x32)
    assert ("coord", "F8", [0x32]) in server.calls
    assert not any(m == "__sam__request" for tag, m, a in server.calls
                   if tag == "coord")
    assert sess.status()["last_error"] is None


def test_start_navi_task_takes_the_coordinator_lease_and_calls_f0(server):
    clock = Clock()
    sess = _session(server, clock)
    server.calls.clear()
    sess.start_navi_task([(1.0, 2.0, 0.0), (3.0, 4.0, 1.5)])
    coord = [(m, a) for tag, m, a in server.calls if tag == "coord"]
    # The lease first, then the guarded call - the same order start_manual
    # and abort use, because CoordinatorProxy guards F0 the same way.
    assert coord[0][0] == "__sam__request"
    assert ("F0", [[[1.0, 2.0, 0.0], [3.0, 4.0, 1.5]]]) in coord


def test_start_navi_task_packs_every_waypoint_as_three_floats(server):
    clock = Clock()
    sess = _session(server, clock)
    server.calls.clear()
    sess.start_navi_task([(1, 2, 0)])          # ints on the way in
    sent = [a for tag, m, a in server.calls if tag == "coord" and m == "F0"]
    assert sent == [[[1.0, 2.0, 0.0]]]
    assert all(isinstance(c, float) for c in sent[0][0])


def test_pause_and_resume_are_the_coordinators_f4_and_f5(server):
    clock = Clock()
    sess = _session(server, clock)
    server.calls.clear()
    sess.pause_task()
    sess.resume_task()
    coord = [m for tag, m, a in server.calls if tag == "coord"]
    assert "F4" in coord and "F5" in coord
    assert coord.count("__sam__request") == 2   # both are guarded
    assert sess.status()["last_error"] is None
```

  In `rover/src/navi_teleop/test/test_bema_bridge.py`, append — the `bridge` fixture there yields `(node, server, clock)` in that order, and `_string` is already defined at the bottom of the file:

```python
def test_a_task_finished_command_reaches_the_coordinator(bridge):
    node, server, clock = bridge
    node._on_command(_string({"action": "task_finished", "tag": 0x31}))
    assert ("coord", "F8", [0x31]) in server.calls
    node._on_command(_string({"action": "task_finished", "tag": 0x32}))
    assert ("coord", "F8", [0x32]) in server.calls


def test_a_task_finished_command_with_a_foreign_tag_is_refused(bridge):
    node, server, clock = bridge
    server.calls.clear()
    node._on_command(_string({"action": "task_finished", "tag": 7}))
    node._on_command(_string({"action": "task_finished"}))
    node._on_command(_string({"action": "task_finished", "tag": "0x31"}))
    assert not any(m == "F8" for tag, m, a in server.calls)


def test_the_task_commands_reach_the_coordinator(bridge):
    node, server, clock = bridge
    server.calls.clear()
    node._on_command(_string({"action": "navi_task",
                              "waypoints": [[1.0, 2.0, 0.0]]}))
    assert ("coord", "F0", [[[1.0, 2.0, 0.0]]]) in server.calls
    node._on_command(_string({"action": "pause_task"}))
    node._on_command(_string({"action": "resume_task"}))
    coord = [m for tag, m, a in server.calls if tag == "coord"]
    assert "F4" in coord and "F5" in coord


def test_a_navi_task_command_with_malformed_waypoints_is_refused(bridge):
    node, server, clock = bridge
    server.calls.clear()
    for bad in ({"action": "navi_task"},
                {"action": "navi_task", "waypoints": []},
                {"action": "navi_task", "waypoints": "1,2,3"},
                {"action": "navi_task", "waypoints": [[1.0, 2.0]]},
                {"action": "navi_task", "waypoints": [[1.0, 2.0, "x"]]},
                {"action": "navi_task", "waypoints": [[1.0, 2.0, True]]},
                {"action": "navi_task", "waypoints": [[0.0, 0.0, 0.0]] * 65}):
        node._on_command(_string(bad))
    assert not any(m == "F0" for tag, m, a in server.calls)
```

- [ ] Run both teleop test files and watch them fail → `AttributeError: 'BemaSession' object has no attribute 'notify_task_finished'` (and `start_navi_task`), and `unknown drive action: 'task_finished'` leaving `server.calls` without an `F0`/`F8`.

- [ ] Implement in `rover/src/navi_teleop/navi_teleop/bema_session.py` — first factor the duplicated just-in-time lease dance out of `start_manual()` and `abort()`, which are byte-for-byte the same but for the method and the error string:

```python
    def _coord_guarded(self, method, *args, label=None):
        # CoordinatorProxy binds F0-F7 behind checkAccess() on the
        # coordinator's OWN ServerAccessManager - a separate lease from
        # BEMA's. Acquire it just-in-time; it lapses by itself after 4 s of
        # guarded-call silence, and the unguarded F8/F9/F10 traffic neither
        # needs nor refreshes it.
        if self._coord is None:
            return
        try:
            if not self._coord.call("__sam__request"):
                self._last_error = ("coordinator refused the lease for "
                                    f"{label or method}")
                return
            self._coord.call(method, *args)
        except RpcError as exc:
            self._mark_refused(exc, label or method)
        except (RpcTimeout, RpcDisconnected, OSError) as exc:
            self._mark_down(exc)

    def start_manual(self):
        self._coord_guarded("F6", label="start_manual")

    def abort(self):
        # F7 on the COORDINATOR is abort - not BEMA's F7, which is
        # setMovementEnabled(bool) on a different server on a different port.
        self._coord_guarded("F7", label="abort")
```

  (the `label` keeps `_last_error` and `_mark_refused` reading exactly as they do today, so no existing test moves.) Then add the three task-state calls after `abort()`:

```python
    def start_navi_task(self, waypoints):
        # CoordinatorProxy binds F0 (startNaViTask) behind checkAccess() on
        # the coordinator's own ServerAccessManager - same just-in-time lease
        # as start_manual() and abort(). waypoints is a list of (x, y, yaw)
        # floats; the coordinator relays them straight back to us as NaVi F3.
        self._coord_guarded("F0", [[float(x), float(y), float(w)]
                                   for x, y, w in waypoints])

    def pause_task(self):
        self._coord_guarded("F4")

    def resume_task(self):
        self._coord_guarded("F5")

    def notify_task_finished(self, tag):
        # CoordinatorProxy binds F8 (notifyTaskFinished) with no
        # checkAccess() - it is @noguard in Coordinator.idl - so this needs
        # no lease and cannot fight anything for one. That is exactly why
        # navi_rpc_server reports progress through this session instead of
        # opening a second client to the primary.
        self._safe_coord("F8", int(tag))
```

- [ ] Implement in `rover/src/navi_teleop/navi_teleop/bema_bridge.py` — replace the first lines of `_on_command` so the whole payload survives, and handle the two actions that carry an argument:

```python
    _TASK_TAGS = (0x31, 0x32)
    _MAX_WAYPOINTS = 64

    @staticmethod
    def _waypoints(value):
        """[[x, y, w], ...] of finite non-bool numbers, or None.

        The same shape navi_rpc_state.parse_targets enforces on the way in
        from the coordinator, applied here on the way back out to it: this
        is a JSON topic anyone on the graph can publish to, and F0 arms an
        autonomous run.
        """
        if isinstance(value, (str, bytes)) or not isinstance(value, list):
            return None
        if not value or len(value) > BemaBridge._MAX_WAYPOINTS:
            return None
        out = []
        for point in value:
            if isinstance(point, (str, bytes)) or not isinstance(point, list) \
                    or len(point) != 3:
                return None
            row = []
            for component in point:
                if isinstance(component, bool) \
                        or not isinstance(component, (int, float)) \
                        or not isfinite(float(component)):
                    return None
                row.append(float(component))
            out.append(row)
        return out

    def _on_command(self, msg: String):
        try:
            payload = json.loads(msg.data)
            action = payload.get("action")
        except (json.JSONDecodeError, TypeError, AttributeError):
            self.get_logger().warn(f"unreadable drive command: {msg.data!r}")
            return
        if action == "task_finished":
            # navi_rpc_server's progress, on its way to the coordinator's F8.
            # The tag is whitelisted: TAG_WaypointReached (0x31) and
            # TAG_DestinationReached (0x32) are the only two the coordinator
            # acts on, and an arbitrary one would drive a state machine we do
            # not own.
            tag = payload.get("tag")
            if not isinstance(tag, int) or isinstance(tag, bool) \
                    or tag not in self._TASK_TAGS:
                self.get_logger().warn(f"refusing task_finished tag {tag!r}")
                return
            self._last_action = action
            try:
                self._session.notify_task_finished(tag)
            except Exception as exc:
                self.get_logger().error(f"task_finished failed: {exc!r}")
            return
        if action == "navi_task":
            # The operator's Go, on its way to the coordinator's guarded F0.
            waypoints = self._waypoints(payload.get("waypoints"))
            if waypoints is None:
                self.get_logger().warn(
                    f"refusing navi_task waypoints {payload.get('waypoints')!r}")
                return
            self._last_action = action
            try:
                self._session.start_navi_task(waypoints)
            except Exception as exc:
                self.get_logger().error(f"navi_task failed: {exc!r}")
            return
        table = {
            "pause_task": self._session.pause_task,
            "resume_task": self._session.resume_task,
```

  (`from math import isfinite` joins the existing `from math import degrees` import. The rest of `_on_command` — the remaining `table` entries, the unknown-action warning, the stop latch and the `handler()` call — is unchanged, and `pause_task`/`resume_task` take no argument so they need nothing beyond the two table lines.)

- [ ] `rover/src/navi_teleop/test/fake_bema_server.py` — `_coord_dispatch` already answers `1, None` for an un-leased `F0`–`F6` and falls through to `None, None` once the lease is held, so `F0`/`F4`/`F5` need **no change at all**; that is what the new session tests assert against. The only edit is the unguarded `F8`, beside `F9`/`F10` and **before** the lease gate:

```python
        if method == "F8":             # notifyTaskFinished - @noguard
            return None, None
```

- [ ] Update the docstring of `bema_bridge.py` so the new channels are written down where the node is read: after the `/drive_command (JSON) drives the coordinator/BEMA buttons...` sentence, add `navi_rpc_server uses the same topic to send the coordinator its waypoint progress, because F8 is unguarded and needs no lease; navi_task/pause_task/resume_task go the other way, to the coordinator's guarded F0/F4/F5, and are how an autonomous run is armed at all.`

- [ ] Run the teleop suite → expect **88 passed** (79 + 4 + 5).

- [ ] Run the supervisor suite → expect **116 passed**, unchanged.

- [ ] Commit: `git add rover/src/navi_teleop/navi_teleop/bema_session.py rover/src/navi_teleop/navi_teleop/bema_bridge.py rover/src/navi_teleop/test/fake_bema_server.py rover/src/navi_teleop/test/test_bema_session.py rover/src/navi_teleop/test/test_bema_bridge.py && git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "The coordinator client for task state: startNaViTask, pause and resume behind one just-in-time lease, and waypoint progress back as notifyTaskFinished, all on the session bema_bridge already owns"`

---

## Task 6: the `.18` alias, the launcher, and deploying it to the Orin

**Files:**
- Modify: `rover/start_navi.sh`
- Test: `rover/test/test_navi_alias.sh`

**Interfaces:**
- Produces: `ensure_navi_alias()` in `start_navi.sh` (idempotent, never fatal); `--no-navi-rpc`; the `navi_rpc_server` launch line; the stale-process pattern.
- Consumes: `ip`, `sudo -n`. Nothing else.

### Steps

- [ ] Write the failing test at `rover/test/test_navi_alias.sh`:

```bash
#!/usr/bin/env bash
# start_navi.sh's NaVi alias step, exercised without root and without a LAN.
#
# The coordinator on the primary calls a hard-coded 192.168.178.18:21021, so
# the Orin has to answer on a second address. Adding it needs sudo, which a
# bring-up over ssh may not have - and the one thing this step must never do
# is take the rover down because of that. So: idempotent when the alias is
# already there, loud when it cannot be added, exit 0 either way.
#
# start_navi.sh is sourced with NAVI_FUNCTIONS_ONLY=1, which stops it before
# anything is launched, and ensure_navi_alias is then run against a fake `ip`
# and a fake `sudo` on PATH.
#
# Run it directly: bash rover/test/test_navi_alias.sh (exit 0 = pass).

set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT="$HERE/../start_navi.sh"
FAILURES=0

# A fake `ip` that prints the given `ip -o -4 addr show` lines, and records
# every invocation so a case can assert an `addr add` did or did not happen.
make_fakes() {
    local dir="$1" sudo_status="$2"; shift 2
    printf '%s\n' "$@" > "$dir/addrs"
    cat > "$dir/ip" <<'FAKE'
#!/usr/bin/env bash
here="$(dirname "$0")"
echo "ip $*" >> "$here/log"
for arg in "$@"; do
    [ "$arg" = "add" ] && exit 0
done
cat "$here/addrs"
FAKE
    cat > "$dir/sudo" <<FAKE
#!/usr/bin/env bash
here="\$(dirname "\$0")"
echo "sudo \$*" >> "\$here/log"
exit $sudo_status
FAKE
    chmod +x "$dir/ip" "$dir/sudo"
    : > "$dir/log"
}

run_alias() {
    local dir="$1"
    PATH="$dir:$PATH" NAVI_FUNCTIONS_ONLY=1 \
        bash -c 'source "$0"; ensure_navi_alias' "$SCRIPT" 2>&1
}

expect() {
    local name="$1" want_status="$2" want_log_grep="$3" want_absent="$4"
    # want_output_grep is what the operator must SEE. The sudo-refused case
    # exists only to prove the warning is loud, so asserting exit 0 and an
    # attempted `addr add` would assert nothing that case does not share with
    # the case above it.
    local want_output_grep="$5" sudo_status="$6"; shift 6
    local dir output status log
    dir=$(mktemp -d)
    make_fakes "$dir" "$sudo_status" "$@"
    output=$(run_alias "$dir")
    status=$?
    log=$(cat "$dir/log")
    rm -rf "$dir"

    if [ "$status" -ne "$want_status" ]; then
        echo "FAIL $name: exit $status, expected $want_status"
        echo "$output" | sed 's/^/     | /'
        FAILURES=$((FAILURES + 1))
        return
    fi
    if [ -n "$want_log_grep" ] && ! grep -q -- "$want_log_grep" <<< "$log"; then
        echo "FAIL $name: expected '$want_log_grep' in the command log:"
        echo "$log" | sed 's/^/     | /'
        FAILURES=$((FAILURES + 1))
        return
    fi
    if [ -n "$want_absent" ] && grep -q -- "$want_absent" <<< "$log"; then
        echo "FAIL $name: '$want_absent' should not have been run:"
        echo "$log" | sed 's/^/     | /'
        FAILURES=$((FAILURES + 1))
        return
    fi
    if [ -n "$want_output_grep" ] && ! grep -q -- "$want_output_grep" <<< "$output"; then
        echo "FAIL $name: expected '$want_output_grep' in the output:"
        echo "$output" | sed 's/^/     | /'
        FAILURES=$((FAILURES + 1))
        return
    fi
    echo "ok   $name"
}

LOOPBACK='1: lo    inet 127.0.0.1/8 scope host lo'
ROVER='2: eth0    inet 192.168.178.33/24 brd 192.168.178.255 scope global eth0'
ALIAS='2: eth0    inet 192.168.178.18/24 brd 192.168.178.255 scope global secondary eth0'

expect "already present is a no-op" 0 "" "addr add" "" 0 "$LOOPBACK" "$ROVER" "$ALIAS"
expect "absent and permitted adds it" 0 "addr add 192.168.178.18/24 dev eth0" "" "added the NaVi alias" 0 "$LOOPBACK" "$ROVER"
expect "sudo refused warns and continues" 0 "addr add 192.168.178.18/24 dev eth0" "" "could not add the NaVi alias" 1 "$LOOPBACK" "$ROVER"
expect "no rover LAN warns and continues" 0 "" "addr add" "no interface holds a 192.168.178.x address" 0 "$LOOPBACK"

if [ "$FAILURES" -ne 0 ]; then
    echo "$FAILURES case(s) failed"
    exit 1
fi
echo "all NaVi alias cases pass"
```

- [ ] Run it and watch it fail: `bash rover/test/test_navi_alias.sh` → every case fails with `ensure_navi_alias: command not found` (exit 127, expected 0).

- [ ] Implement in `rover/start_navi.sh`. Four edits, all above the `NAVI_FUNCTIONS_ONLY` guard except the launch block.

  **(a)** In the header comment, after the `mode_supervisor` bullet, insert:

```
#   5. navi_rpc_server - the endpoint the primary's coordinator calls "NaVi",
#      msgpack-RPC on :21021. It is what makes startNaViTask succeed instead
#      of logging "NaVi not reachable"; the 192.168.178.18 alias this script
#      adds is the address the coordinator has hard-coded.
```

  and renumber the two bullets below it; in the usage block add:

```
#   ./start_navi.sh --no-navi-rpc  no navi_rpc_server (the coordinator cannot arm an autonomous run)
```

  **(b)** With the other flags: `START_NAVI_RPC=1` beside `START_SUPERVISOR=1`, and `--no-navi-rpc) START_NAVI_RPC=0; shift ;;` in the `case`.

  **(c)** Above the `NAVI_FUNCTIONS_ONLY` guard, beside the other helper functions:

```bash
# ---------------------------------------------------------------------------
# The second address the primary's coordinator calls.
#
# navi::NaViEP::address is a compile-time 192.168.178.18 in the primary's
# binary, so serving the NaVi interface means answering on that address as
# well as on the Orin's own .33. Adding it needs root, which a bring-up over
# ssh may not have - and losing the rover because of that would be far worse
# than losing the autonomous arming path. So this warns and continues:
# navi_rpc_server binds 0.0.0.0 either way, and the moment someone adds the
# alias by hand it starts answering, with no restart.
#
# Rebuilding the primary's rpc_coord with .33 in it is the rejected
# alternative (spec 3): it modifies the flight computer of a competition
# rover for a cosmetic reason.
# ---------------------------------------------------------------------------
NAVI_ALIAS="${NAVI_ALIAS:-192.168.178.18}"
NAVI_ALIAS_CIDR="${NAVI_ALIAS_CIDR:-$NAVI_ALIAS/24}"

ensure_navi_alias() {
    local iface
    if ip -o -4 addr show 2>/dev/null \
        | awk -v want="$NAVI_ALIAS/" 'index($4, want) == 1 { found = 1 }
                                      END { exit !found }'; then
        echo "the NaVi alias $NAVI_ALIAS is already on this machine"
        return 0
    fi
    iface=$(ip -o -4 addr show 2>/dev/null \
        | awk '$4 ~ /^192\.168\.178\./ { print $2; exit }')
    if [ -z "$iface" ]; then
        echo "warning: no interface holds a 192.168.178.x address, so the NaVi" >&2
        echo "         alias $NAVI_ALIAS cannot be added. navi_rpc_server still" >&2
        echo "         binds 0.0.0.0, but the coordinator calls .18 and will" >&2
        echo "         report 'NaVi not reachable' until the rover is on the" >&2
        echo "         rover LAN." >&2
        return 0
    fi
    if sudo -n ip addr add "$NAVI_ALIAS_CIDR" dev "$iface" 2>/dev/null; then
        echo "added the NaVi alias $NAVI_ALIAS_CIDR to $iface"
        return 0
    fi
    echo "warning: could not add the NaVi alias $NAVI_ALIAS_CIDR to $iface -" >&2
    echo "         sudo -n refused. Autonomy cannot be armed until someone runs:" >&2
    echo "           sudo ip addr add $NAVI_ALIAS_CIDR dev $iface" >&2
    echo "         Everything else in this bring-up is unaffected." >&2
    return 0
}
```

  **(d)** In the cleanup block, widen the supervisor pattern:

```bash
    kill_stale "navi_supervisor nodes" "navi_supervisor/(mode_supervisor|navi_rpc_server)"
```

  and in the launch section, immediately after the `mode_supervisor` block and before `bema_bridge`:

```bash
if [ "$START_NAVI_RPC" -eq 1 ]; then
    ensure_navi_alias
    echo "starting navi_rpc_server (the coordinator's NaVi endpoint on :21021)"
    ros2 run navi_supervisor navi_rpc_server &
    BACKGROUND_PIDS+=("$!")
fi
```

- [ ] Run the alias gate: `bash rover/test/test_navi_alias.sh` → expect four `ok` lines and `all NaVi alias cases pass`.

- [ ] Run the existing launcher gate, which sources the same file: `bash rover/test/test_start_navi_gate.sh` → expect it still passes.

- [ ] Run both python suites once more → **116** and **88**.

- [ ] Commit: `git add rover/start_navi.sh rover/test/test_navi_alias.sh && git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "start_navi.sh adds the 192.168.178.18 alias idempotently and starts navi_rpc_server; no sudo warns loudly and brings the rover up anyway"`

### Deploying and verifying on the Orin

The Orin (`star@a_navi`, `192.168.178.33`) is reachable over ssh, has **no internet**, and currently has **no camera** — it is dark. None of that blocks this sub-project: `navi_rpc_server` needs no ZED, no localisation and no Nav2, so the whole of SP8 can be verified on the real machine today. `msgpack` is already installed there (`bema_bridge` uses it), so nothing needs downloading.

- [ ] Deploy and build: `./deploy_rover.sh` (rsync + `colcon build --symlink-install` on the Orin).
- [ ] Run the Orin-side suites: `./deploy_rover.sh --test` → the `navi_supervisor` and `navi_teleop` suites must show the same counts as here (**116** and **88**). Nothing in them binds `:21021` or the alias.
- [ ] Bring up only what this needs, on the Orin: `ssh star@a_navi 'cd navi && ./start_navi.sh --no-localization --no-video --no-drive-bridge'`. Expect either `added the NaVi alias 192.168.178.18/24 to <iface>` or the sudo warning — and in both cases `navi_rpc_server is serving 0.0.0.0:21021` in the log.
- [ ] Check the address and the socket: `ssh star@a_navi 'ip -o -4 addr show | grep 192.168.178; ss -ltnp | grep 21021'`. The alias must appear as a `secondary` address on the same interface as `.33`, and the listener must be there.
  If the alias is missing, add it by hand once — `ssh -t star@a_navi 'sudo ip addr add 192.168.178.18/24 dev <iface>'` — and, when it is wanted across reboots, record it in the Orin's netplan rather than in this script.
- [ ] Prove the endpoint from the laptop, on the address the primary uses — this is the acceptance check for SP8:

```
python3 - <<'PY'
import msgpack, socket
s = socket.create_connection(("192.168.178.18", 21021), timeout=2.0)
u = msgpack.Unpacker(raw=False)
def call(mid, method, *args):
    s.sendall(msgpack.packb([0, mid, method, list(args)], use_bin_type=True))
    while True:
        for m in u:
            return m
        u.feed(s.recv(4096))
print(call(1, "__sam__request"))                      # [1, 1, None, True]
print(call(2, "F0"))                                  # [1, 2, None, None]
print(call(3, "F3", [[1.0, 2.0, 0.0]]))               # [1, 3, None, None]
print(call(4, "F4"))                                  # [1, 4, None, None]
print(call(5, "F5"))                                  # [1, 5, None, False]
print(call(6, "F2"))                                  # error: "F2 (getPosition) is not served..."
print(call(7, "__sam__release"))                      # [1, 7, None, True]
PY
```

- [ ] Watch the state on the rover while that runs: `ssh star@a_navi 'source /opt/ros/humble/setup.bash && source navi/install/local_setup.bash && ros2 topic echo /navi_rpc/status --once'` → `navigation_requested: true`, `start_seq: 1`, `target_count: 1`.
- [ ] **Not verified until the rover LAN has the primary on it:** that `CoordinatorImpl::startNaViTask` actually reaches us. The check is the primary's own log — the absence of `"Cannot start navigation task: NaVi not reachable"` and a coordinator `getState` of `4` then `5` — and it belongs to the first SP11 integration run, not here. Everything on this side is verifiable without it.

---

## Done means

- `192.168.178.18:21021` answers `__sam__request` with `true`, and `F0`/`F3`/`F4`/`F5`/`F6`/`F7` behind it, on the Orin.
- A guarded call without the lease is answered `1`, exactly as `ServerProxy::checkAccess` does; `F1` is answered with a named refusal; the three unguarded `F2`/`F8`/`F9` answer benign, correctly-shaped values (a 4×4 identity, an empty ToF frame, nil) rather than an exception their clients do not catch; `F10`–`F12` are `unknown method`.
- Wrong arities, wrong types, unknown methods, oversized lists and non-msgpack garbage cost at most one connection, never a thread and never the node — with tests for each.
- `setTargets` puts the waypoints on `/navi_rpc/targets` as a latched `nav_msgs/Path` in `map`, and the run state on `/navi_rpc/status` as JSON, which is what SP11's `goal_relay` reads.
- `{"event": "waypoint_reached"}` on `/navi_rpc/progress` becomes `notifyTaskFinished(0x31)` on the coordinator, and `destination_reached` becomes `0x32`, through `bema_bridge`'s session.
- The other half of §8's SP8 row is built too: `{"action": "navi_task", "waypoints": [[x, y, w], ...]}`, `{"action": "pause_task"}` and `{"action": "resume_task"}` on `/drive_command` reach the coordinator as the guarded `F0`/`F4`/`F5`, taking the coordinator's lease just-in-time — so the operator's Go button has something to call.
- `stopNavigation` stops the wheels and bumps `stop_seq` in `/navi_rpc/status`. It never publishes `/mode_request`, so it can neither clear a latched e-stop nor turn the coordinator's *pause* into an *abort*; `test_the_server_never_publishes_a_mode_request` pins that.
- `start_navi.sh` adds the alias idempotently, survives having no sudo, and starts the server; `bash rover/test/test_navi_alias.sh` and `bash rover/test/test_start_navi_gate.sh` both pass.
- `navi_supervisor` **116 passed**, `navi_teleop` **88 passed**, `colcon build --packages-select navi_supervisor navi_teleop` clean, here and on the Orin.

## What SP8 deliberately leaves to others

- **SP11** publishes `/navi_rpc/progress` from `goal_relay`, reads `/navi_rpc/status` to know a run is armed, and asks for the coordinator's `F0 startNaViTask` when the operator presses Go. The *client* for that call is built here (Task 5, `BemaSession.start_navi_task` behind `{"action": "navi_task"}`), so SP11's Task 8 binds it rather than writing it; the schema of both topics is fixed here (Task 4's Interfaces) so SP11 has nothing to guess.
- **The stop path is SP11's to complete.** `F6 stopNavigation` bumps `stop_seq` on `/navi_rpc/status`; SP11's `goal_relay` watches that counter and cancels the Nav2 goal. Cancelling at the source is what actually stops `/autonomy_twist`, and it keeps mode authority in one place — the supervisor (SP5), exactly as SP11's own ruling requires (*"`goal_relay` never changes the supervisor's mode … One authority, one direction."*). Until SP11 lands, `F6` has no autonomy-stopping teeth beyond the chassis stop, which is harmless: nothing publishes `/autonomy_twist` before SP9.
- **`/navi_rpc/progress` is the only path from a completed leg to `notifyTaskFinished`.** SP11's `TaskControl.notify_waypoint_reached`/`notify_destination_reached` must be implemented as a publish onto that topic, not as a direct coordinator call — two paths to the same `F8` would desynchronise `/navi_rpc/status`'s `reached_index` from the coordinator's real state.
- **SP9** owns Nav2. Nothing in this plan starts, cancels or configures it; `stopNavigation` reaches Nav2 only indirectly, through SP11's `stop_seq` watcher above.
- `F1 setPosition` stays a named refusal, and the unguarded `F2 getPosition`, `F8 getTofData`, `F9 takeSnapshot` answer benign, correctly-shaped values, until something actually needs them (spec §3). The camera methods `F10`–`F12` stay unbound.
- **Spec §2's topic graph will be stale.** `/navi_rpc/targets` and `/navi_rpc/status` do not appear in it (§2 shows only `navi_rpc_server (:21021 …)`). Fixing their schema here is deliberate — SP11 needs it fixed — but §2 should be amended to carry both topics when SP11 lands.

---

## Self-review

- **Spec coverage.** §3's four demands are each somewhere concrete: the served subset (`F0`, `F3`–`F7`) is the method table in Global Constraints and Task 3; the not-served subset is `STUBS` (`F1` a named refusal, `F2`/`F8`/`F9` benign, correctly-shaped answers); the tags `0x31`/`0x32` are `TAG_WAYPOINT_REACHED`/`TAG_DESTINATION_REACHED` in Task 2 and asserted in Tasks 2, 4 and 5; the idempotent `ip addr add 192.168.178.18/24 dev <iface>` in `start_navi.sh` is Task 6. §3's "operator-initiated only" is design decision 4 and is enforced by `start_navigation` queueing no action at all — asserted in `test_start_navigation_arms_the_run_and_bumps_the_sequence`. §8's SP8 row is two halves and both are built here: *"only the methods `startNaViTask` needs"* is the served method table (Tasks 1–4), and *"coordinator client for task state"* is Task 5 — `BemaSession.start_navi_task`/`pause_task`/`resume_task`/`notify_task_finished` on top of `_coord_guarded`, reachable from the graph as `/drive_command` actions. SP11's Task 8 binds that client; it does not build it.
- **The one thing that would have sunk this** is the `__sam__` lease: without it the coordinator's `getCapability()` returns `nullptr` and logs the exact "NaVi not reachable" this sub-project exists to prevent, even with `:21021` bound and every `F` method served. It is in the Global Constraints table, implemented in Task 1, and asserted from the client side in Tasks 1 and 3.
- **Placeholder scan:** no `TODO`, no `...`, no `<fill in>`, no unnamed topic or type. Every command is runnable as written; every expected count is derived from a measured baseline (49 and 79, run 2026-08-31) plus the tests this plan adds.
- **Type consistency:** `F5` returns a Python `bool` (`is True` in the tests, because the client does `.as<bool>()`); the access refusal is the integer `1` and stub refusals are strings, and the tests assert the two are different; tags cross ROS as JSON ints and reach `F8` as `int(tag)`; targets are validated to exactly three finite non-bool numbers before they are stored; `/navi_rpc/status` is asserted JSON-serialisable.
- **Concurrency:** every `NaviRpcState` method takes one `RLock`, because socket threads call it while the executor thread drains it; publishers are touched only from timer and subscription callbacks, never from a socket thread.
- **Blast radius:** the only file shared with a concurrent planner is `rover/start_navi.sh` (one flag, one function, one launch block, one widened `kill_stale` pattern). The collision is not the file but the *anchor*: SP10 Task 5 (`2026-08-31-sp10-twist-shaper.md:1540-1553`) inserts its `twist_shaper` launch block at the identical position — after `mode_supervisor`, before `bema_bridge` — and SP9 Task 7 adds a `kill_stale` line beside the ones this plan widens. Whoever lands second resolves both by hand; neither conflict is semantic, and the order of the two launch blocks does not matter. Nothing under `sim/`, `ground_station/`, `navi_autonomy/` or `navi_localization/` is touched.
