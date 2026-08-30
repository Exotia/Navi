# BEMA bridge: driving the real wheels from the Navi ground station

Date: 2026-08-30. Status: approved in discussion, awaiting spec review.

## Goal

A gamepad twist from the Navi ground station reaches the rover's inverse
kinematics and drives the real wheels, with a stop reflex that lives on
the rover's own LAN. Nothing on the primary (the Raspberry Pi) or the
jetson changes.

## What exists

The rover has three computers on `192.168.178.0/24`:

| Host | IP | Role |
|---|---|---|
| `a_primary` | .26 | Raspberry Pi. `rpc_bema_unterboden` serves the BEMA drive RPC on **:21022** (also Mainbody :21021, telemetry :1234/:1235). `rpc_coord` serves the mission coordinator on **:21031**. Owns the IK (Simulink model 2.42 in `bemacontroller/src/betterIK`), the TMC4671 motor controllers, SPI/I2C. |
| `jetson` | .25 | Secondary stack (drill, gnss, imu). Also `onboard_cli`, a joystick-to-BEMA client we copy. |
| `a_navi` | .33 | The Orin: ZED, localisation, mapping, `navi_teleop`. |

The RPC framework is rpclib (msgpack-RPC over TCP) with an in-house lease
layer (`rpc_base`). Everything below was read from the sources on the Pi and
the jetson; the local untracked clone `bemacontroller/` is at the Pi's
commit `bfeae15`.

### The BEMA server (:21022)

Methods (`deps/interface/server/BEMAProxy.h`):

| RPC | Meaning | Moves wheels? |
|---|---|---|
| `F0` | `init()` — zeroes steering, starts odometry and the IK thread | **yes** |
| `F1(vX, vY, w)` | `drive()`: vX, vY in m/s rover frame, **w in deg/s**; server stores `u = -pi*w/180` | via IK |
| `F2` | `stopMovement()` — also the e-stop | stops |
| `F3` | `resetOdometry()` | no |
| `F4` | `resetEncoders()` — absolute-encoder zeroing routine | **yes** |
| `F5` | `changeDriveMode()` (Kinematic / Direct) | no |
| `F6` | `changeDriveState()` | can |
| `F7(bool)` | `setMovementEnabled()` | no |
| `F8` | `fetchOdometry()` → 3x3 float | no |

`drive()` returns silently unless `m_movementEnabled && m_initialized`
(`BemaServer.cpp:156`). The IK ticks every 60 ms. **There is no server-side
deadman**: the last `drive()` stands until something replaces it.

Lease (`rpc_base`): the TCP connection is the session. `__sam__request` →
bool grants exclusive access; `__sam__ping` → bool must be called (or any
guarded method) within 4 s or the lease lapses; `__sam__release` gives it
back; `__sam__force` seizes it. Every `F*` except `F8` requires the lease.
A lapse does **not** stop the wheels.

### The coordinator (:21031)

A state machine — `Disconnected`, `Idle`, `PrepareManual`, `Manual`,
`PrepareAutonomous`, `Autonomous` — that gates movement: `m_movementAllowed`
is true only in `Manual`/`Autonomous`. A loop per subsystem pushes
`setMovementEnabled(allowed)` to BEMA every 200 ms and uses `__sam__force`
to disable, so it always wins. Consequences for us:

- `startManual()` moves `Idle → PrepareManual → (5 s) → Manual`.
- `notifyConnected()` must arrive at least every 2 s or the coordinator
  drops to `Disconnected` and disables movement.
- `getState()` reports the state.

The coordinator also expects to *call* a NaVi RPC server (`init`,
`setTargets`, `stopNavigation`) at `192.168.178.18:21021` — an address
`a_navi` does not have. That is the autonomy integration and is out of
scope here.

### `onboard_cli` (the reference client, jetson `~/build/onboard-ground-station`)

`src/mappings/BemaConnection.cpp`: a thread sends `drive()` every 25 ms
unconditionally (zeros included); stick deadzone 0.25 rescaled;
`x = stick1.y`, `y = stick1.x`, `w = stick2.x * 45` (so w is ±45 deg/s);
`ensureCapability()` each tick; retries on `rpc::timeout` and
`AccessRevokedException`. Buttons: A `changeDriveState`, B `stopMovement`,
X `resetEncoders`, Y `resetOdometry`, LB `changeDriveMode`.

Operational rule (stated by the operator): only one driver is ever active,
so lease contention between `onboard_cli` and our bridge does not arise.

## Design

```
GS (laptop) --rosbridge--> a_navi ------------TCP-----------> a_primary
  gamepad -> /manual_twist    bema_bridge node                :21022 BEMA (F1 at 20 Hz, lease)
  DRIVE row -> /drive_command (navi_teleop, Python)           :21031 coordinator (heartbeat, startManual)
  status <-- /drive_status <--+
```

One new node, `bema_bridge`, in the existing `rover/src/navi_teleop`
package, written in Python with the `msgpack` package (to be installed on
the Orin: `pip install msgpack`). The Pi and jetson are untouched.

### Units and axes

- `vX = linear.x`, `vY = linear.y` (m/s). Same frame as the GS's twist:
  stick forward = +x. `onboard_cli` maps stick-forward to `vX` too.
- `w = -degrees(angular.z)`. The server computes `u = -pi*w/180`, so
  positive `angular.z` (counter-clockwise, ROS convention) reaches the IK
  as positive `u`. **The sign is verified against the simulator's IK and
  then on hardware with the rover on blocks before any free driving**; if
  the rover turns the wrong way the negation is dropped in one place.
- GS limits stay as they are: `MAX_LINEAR_SPEED = 0.5 m/s`,
  `MAX_ANGULAR_SPEED = 1.0 rad/s` (= 57 deg/s, inside `onboard_cli`'s ±45
  by a margin the operator accepts for now).

### Units of code

**`navi_teleop/msgpack_rpc.py`** — no ROS. `RpcClient(host, port,
timeout_s=1.0)`: one TCP socket; `call(method, *args)` sends
`[0, id, method, [args]]`, reads until the matching `[1, id, error,
result]`, raises `RpcError(error)` on a non-nil error, `RpcTimeout` on the
socket timeout, `RpcDisconnected` on EOF/socket error. `close()`.

**`navi_teleop/bema_session.py`** — no ROS. `BemaSession(host, bema_port,
coordinator_port, clock)`:

- `connect()`: opens both clients, `__sam__request` on BEMA, records
  `lease = True/False`.
- `tick(now)`: called by the node's 50 ms timer. Sends `__sam__ping` every
  500 ms, `notifyConnected()` every 1 s, and `F1(vx, vy, w)` with the
  current command every tick. Any `RpcDisconnected`/`RpcTimeout` marks
  the session down; reconnect is attempted with backoff (1 s, 2 s, 4 s,
  cap 5 s). If `__sam__ping` returns false, `__sam__request` again.
- `set_command(vx, vy, w_deg)`, `stop()` (= `F1(0,0,0)` then `F2`),
  `init()`, `reset_encoders()`, `reset_odometry()`, `change_drive_mode()`,
  `change_drive_state()`, `start_manual()`, `coordinator_state()`.
- `close()`: `stop()`, `__sam__release`, close sockets.
- `status()` → dict: `connected`, `lease`, `coordinator_state`,
  `last_error`, `reconnect_in_s`.

**`navi_teleop/bema_bridge.py`** — the rclpy node.

- Parameters: `bema_host` (`192.168.178.26`), `bema_port` (21022),
  `coordinator_port` (21031), `deadman_s` (1.0), `twist_topic`
  (`/manual_twist`).
- Subscribes `twist_topic` (depth 1): stores the command and the arrival
  time. Subscribes `/drive_command` (`std_msgs/String`, JSON
  `{"action": ...}` with actions `stop`, `manual`, `init`,
  `reset_encoders`, `reset_odometry`, `drive_mode`, `drive_state`).
- 50 ms timer: if the last twist is older than `deadman_s`, the command
  is zero and — once, on crossing the threshold — `stop()` is sent; zeros
  keep going until a fresh twist arrives. Then `session.tick(now)`.
- 1 Hz timer: publishes `/drive_status` (`std_msgs/String`, JSON:
  session status plus `twist_age_s`, `deadman_active`, `last_command`
  with its outcome).
- Shutdown: `session.close()`.

### Safety rules

1. Every callback catches `Exception`; the outcome goes into
   `/drive_status`, never out of the node. A failed send is treated as a
   disconnect, and the next successful reconnect starts with `stop()`.
2. Start-up calls nothing that moves: no `F0`, no `startManual`. The
   rover cannot move until the operator presses Manual and Init.
3. `F0` and `F4` are sent only on an explicit `/drive_command`, never as
   part of connect/reconnect.
4. The bridge never calls `__sam__force`.
5. The GS's own behaviour stays: on gamepad loss it publishes one zero
   twist and stops publishing, which the deadman then turns into `stop()`
   within `deadman_s`.

### Ground station

A **DRIVE** row next to the MAP row in the dashboard:

- **STOP** — large, always enabled, no confirmation (`stop`).
- **Manual** — `manual`; the status line shows "arming (5 s)" while the
  coordinator reports `PrepareManual`.
- **Init drive** — confirmation dialog stating the wheels will move
  (`init`).
- **Reset encoders** — same confirmation (`reset_encoders`).
- **Reset odometry**, **Drive mode**, **Drive state** — plain.
- Status line: BEMA connected/lease, coordinator state, twist age, last
  command and outcome, from `/drive_status`.

`ros_client.py` gains `subscribe_drive_status` / `publish_drive_command`;
`models.py` gains `parse_drive_status` (typed defaults per field, like
`parse_map_status`). No `rclpy` in `ground_station/`.

### Sim bench

`rover/src/navi_teleop/test/fake_bema_server.py`: a threaded msgpack-RPC
server speaking the BEMA and coordinator method sets, recording every call.
Run as a script with `--forward-twist`, it republishes each `F1` as a
`geometry_msgs/Twist` on `/sim_test_twist`, so the bridge drives the
Gazebo rover end to end before it sees the Pi. The same server backs the
unit tests.

### Testing

- `msgpack_rpc`, `bema_session`: pytest against the fake server —
  framing, lease request/ping/lapse, reconnect backoff, unit conversion,
  `stop()` ordering, `close()` releasing the lease.
- `bema_bridge`: node tests on a throwaway `ROS_DOMAIN_ID` (set in the
  fixture, not inherited) — deadman timing, `/drive_command` dispatch,
  `/drive_status` content, exceptions contained.
- Ground station: row wiring and `parse_drive_status`, like the MAP row.
- Sim: bridge + fake server `--forward-twist` + `sim.launch.py` in a
  throwaway domain; the Gazebo rover follows the gamepad.
- Hardware, in order: rover on blocks, STOP under a hand; sign of `w`
  confirmed; then a slow drive.

### Deploy

`deploy_rover.sh` as today plus a one-time `pip install msgpack` on the
Orin (documented in the launch file's comment and PROJECT_SUMMARY). The
node is added to the teleop launch and to `start_navi.sh`'s stale-cleanup
patterns.

## Out of scope

- The NaVi RPC server the coordinator wants for autonomy (`setTargets`,
  `stopNavigation`, `init`) and the stale `.18` address on the Pi.
- Re-vendoring the IK (model 2.42) into the simulator.
- Any change on `a_primary` or `jetson`.

## Open items to verify during implementation

- The exact msgpack-RPC framing rpclib expects (msgid width, whether
  `params` may be an empty array) — confirmed against the rpclib source
  in `bemacontroller/deps/rpclib` before the client is written.
- `getState()`'s return encoding (enum as int) for the status line.
