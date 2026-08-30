# Full Autonomy — Plan

Written 2026-08-30. Supersedes `2026-08-29-autonomy-design.md`, which was
written one day before the BEMA drive bridge existed and before anyone had
read the primary's source. That document's navigation reasoning survives
almost intact; its picture of the chassis does not. Differences from it are
marked **[CHANGED]**, and §1 lists them all so a reader of the old spec can
see what moved.

## Goal

The operator enters waypoints, presses Go, and the rover drives them on its
own: plans over the ground it has mapped, avoids what the front ZED sees,
follows the route with twists the four-wheel-steered chassis can execute,
and reports progress — while the operator can take over or stop instantly,
and localisation loss halts it.

Out of scope: the arm, the rear ZED, science and probing tasks, multi-rover,
anything needing sensing the rover does not have (rear, sides).

## 1. What changed since the old spec, and why

Five corrections. The first is a bug that would have stopped the rover dead.

**1.1 The manual-twist stream breaks `twist_mux`. [CHANGED]**
The old spec put `/manual_twist` at priority 200 and `/autonomy_twist` at
100, assuming manual twists appear only while the operator steers. They do
not: the ground station publishes at 20 Hz **continuously, zeros included**
(`main_window._poll_gamepad`), because `bema_bridge`'s 1 s deadman feeds on
that stream — silence means stop. Under the old design a constant
priority-200 zero stream outranks autonomy forever and **Nav2 could never
move the rover**. Resolution in §4: the ground station stops publishing when
not in manual mode, the deadman moves to a mode-aware supervisor, and
arbitration is done by the supervisor rather than by priority alone.

**1.2 The chassis endpoint is `bema_bridge`, not "bemacontroller". [CHANGED]**
The old diagram ended at the chassis as if it were a topic. The real chain
is `bema_bridge` → msgpack-RPC → BEMA server `:21022`, with an exclusive
access lease, `setMovementEnabled` gating, a 1 s deadman and a coordinator
heartbeat. Good news: the old spec's SP4 "rename the chassis input to
`/rover_twist`" is now one parameter (`twist_topic`), already present.

**1.3 The coordinator exists and holds a veto. [CHANGED]**
Unknown to the old spec. `rpc_coord` on the primary (`:21031`) runs a mission
state machine; only `Manual` and `Autonomous` permit movement, and it pushes
`setMovementEnabled` every 200 ms, force-taking the lease to disable. ERC
judging reads this state. Autonomy that does not drive this state machine is
vetoed by it. Contract in §3.

**1.4 The IK oracle is the wrong model. [CHANGED]**
`twist_shaper` was to clamp commands using the vendored `SimIkStepper`. That
is Simulink model **2.41 (ert, Merope geometry)**; the rover runs **2.42
(grt, Asterope geometry, runtime `hParams`)**. Acceptable for a simulation,
not for a load-bearing safety clamp. Re-vendoring `betterIK` from the local
`bemacontroller/` clone (same commit as the primary, no rover access needed)
becomes a prerequisite — SP4 below.

**1.5 Map facts. [CHANGED]**
Cells are **0.05 m**, not 0.10 m. There is no whole-map topic: the mapper
publishes 2.5 m **tiles** on `/localization/map_tile` and
`/localization/obstacle_tile`. `traversability_layer` must consume tiles.
Costmap resolution decision revisited in §5.

Also new since the old spec, and useful: `fake_bema_server.py` (a msgpack-RPC
double for the whole chassis), the height-banded terrain view, and a
ground-station DRIVE row with a latching STOP.

## 2. Architecture

```
Ground station (laptop)
  /manual_twist (only in manual/semi modes)   /nav_request  /mode_request  /estop_request
  ← /mode_status  /nav_status  /drive_status  /nav_path_summary
        │ rosbridge
Orin (a_navi, 192.168.178.33)
  ZED → localisation → /localization/pose, /localization/status, /localization/odom_local
                     → elevation_mapper → /localization/map_tile, /obstacle_tile
  tile_aggregator   → /autonomy/traversability (GridMap), /autonomy/costmap_seed (OccupancyGrid)
  cloud_filter      → /autonomy/points_filtered
  Nav2 (Theta*, RotationShim+RPP, smoother, collision monitor) → /autonomy_twist
  mode_supervisor   → /rover_twist  (single writer; owns arbitration + deadman)
  twist_shaper      → clamps to feasible ICR → /chassis_twist, /ik_feasibility
  bema_bridge       → msgpack-RPC → primary
  navi_rpc_server (:21021 on alias 192.168.178.18) ← coordinator calls in
Primary (a_primary, .26): rpc_coord :21031 (mission state) · rpc_bema :21022 (drive, IK, wheels)
```

Single-writer rule: **only `mode_supervisor` publishes `/rover_twist`.** No
`twist_mux`. See §4.

## 3. The coordinator and NaVi contract

The primary's coordinator is the rover's mission state machine and the
authority ERC observes.

States: `Disconnected(0) Idle(1) PrepareManual(2) Manual(3)
PrepareAutonomous(4) Autonomous(5) Waiting(6)`. Movement is permitted only in
`Manual` and `Autonomous`. `notifyConnected` (F10) must arrive at least every
2 s or it drops to `Disconnected`. Both arming transitions take **5 s**.

Coordinator RPC (`:21031`, guarded calls need the coordinator's own
`__sam__` lease — the lesson that cost us a live debugging session):
`F0 startNaViTask(waypoints)` · `F4 pause` · `F5 resume` · `F6 startManual` ·
`F7 abort` · `F8 notifyTaskFinished(tag)` · `F9 getState` · `F10 notifyConnected`.

`startNaViTask` puts the coordinator into `PrepareAutonomous` → `Autonomous`
**and** calls out to a NaVi RPC server the rover expects at
`192.168.178.18:21021`, handing it the waypoints. So autonomy requires us to
**serve** that interface: `F0 init` · `F1 setPosition(x,y)` ·
`F2 getPosition` · `F3 setTargets(vector<(float,float,float)>)` ·
`F4 startNavigation` · `F5 isTargetReached` · `F6 stopNavigation` ·
`F7 setMovementEnabled(bool)` · `F8 getTofData` · `F9 takeSnapshot`.
Progress is reported back with `notifyTaskFinished(tag)`:
`TAG_WaypointReached = 0x31`, `TAG_DestinationReached = 0x32`.

`.18` is verified free on the LAN. Serving it needs a second IP on the Orin's
ethernet (`ip addr add 192.168.178.18/24 dev <iface>`, added idempotently by
`start_navi.sh`) — **zero changes on the primary**, which is the point. The
alternative, rebuilding `rpc_coord` with `.33`, is rejected: it modifies the
flight computer of a competition rover for a cosmetic reason.

**Runs are operator-initiated only.** Ground station → `/nav_request` →
`goal_relay` starts Nav2 **and** calls `startNaViTask` so the mission state
and LEDs are truthful. Coordinator-initiated runs (the coordinator choosing
a task by itself) are **out of scope**; the plan does not build them.

That does **not** remove the NaVi server. `CoordinatorImpl::startNaViTask`
calls `setTargets` on the NaVi endpoint as part of servicing our own request,
and if the endpoint is unreachable it logs `"Cannot start navigation task:
NaVi not reachable"` and drops straight back to `Idle` — so without a served
`:21021`, our own Go button cannot put the coordinator into `Autonomous`.
The server is therefore built (SP8), but only the methods that path needs:
`F0 init`, `F3 setTargets`, `F4 startNavigation`, `F5 isTargetReached`,
`F6 stopNavigation`, `F7 setMovementEnabled`. `F1 setPosition`,
`F2 getPosition`, `F8 getTofData` and `F9 takeSnapshot` are stubbed to a
safe refusal until something needs them.

## 4. Arbitration and the deadman — the safety core

The old design's mux is replaced by one node, because the mux cannot solve
1.1 and because two components owning "stop" is how rovers run away.

**`mode_supervisor`** is the sole publisher of `/rover_twist` and the sole
owner of the deadman. Modes: `manual`, `semi_auto`, `autonomous`, `estop`.

| mode | source of `/rover_twist` | deadman |
|---|---|---|
| manual, semi_auto | `/manual_twist` | 1 s of silence → zero + stop |
| autonomous | `/autonomy_twist` | 0.5 s of Nav2 silence → zero + stop |
| estop | zeros, latched | always stopped |

Rules:
1. **Takeover wins instantly.** Any `/manual_twist` above the deadzone while
   autonomous → mode becomes `manual`, Nav2 goal cancelled, Nav2 lifecycle
   deactivated, coordinator `abort` then `startManual`. Not just muted:
   a still-running Nav2 that regains the output when the operator lets go is
   the dangerous case the old spec correctly identified.
2. **STOP is latched and local.** `/estop_request` → zeros forever until an
   explicit `/mode_request` back to manual. Survives rosbridge loss, because
   it lives on the Orin.
3. **Localisation loss halts autonomy.** `/localization/status` reporting
   `SEARCHING` or `OFF` → zero twist, goal cancelled, reason in
   `/mode_status`. No automatic resume; the operator re-issues Go. (The
   tracker's own pose freeze is not enough — the ZED's TF keeps moving.)
4. **Link loss does not stop autonomy** (ERC expects runs to survive it), but
   does stop manual driving via the deadman, and never clears an e-stop.
5. **The ground station publishes `/manual_twist` only in manual and
   semi_auto.** [CHANGED] In autonomous mode it publishes nothing, so rule 1
   is a real signal rather than a constant stream. The DRIVE row's STOP
   remains live in every mode and travels on `/estop_request`.

`bema_bridge` keeps its own 1 s deadman on whatever it is fed. Two
independent deadmen in series is deliberate: the supervisor protects against
Nav2 hanging, the bridge against the whole Orin-side graph dying.

## 5. Perception and planning

**`tile_aggregator`** [CHANGED] subscribes `/localization/map_tile`, stitches
tiles into a rolling window around the rover (48 m, so the 60 m map cap is
never the binding constraint), and publishes a whole-map GridMap for
downstream use. This is the piece the old spec assumed already existed.

**`traversability_layer`** reads that map and derives `slope`, `step`,
`roughness`, `valid`:
- **step lethal above 0.14 m** — just over the 0.125 m wheel radius. Computed
  as max neighbour difference, so it catches **negative** steps (holes) as
  well as positive ones. The obstacle voxels are positive-only, so holes are
  invisible today; this is where they become lethal cells.
- slope lethal above 25°, roughness scaled below that.
Publishes `/autonomy/traversability` (GridMap, for the view — it can also
(OccupancyGrid, latched).
drive the pit colouring in the sim) and `/autonomy/costmap_seed`  

**Costmap resolution 0.05 m** [CHANGED] to match the elevation map exactly.
Resampling smears the step edges that matter most. Cost: 4× the cells of the
old 0.10 m plan; a 48 m window at 0.05 m is 960², which Nav2 handles but
which must be measured on the Orin in SP10 — the fallback is a 24 m local
window at 0.05 m plus a 48 m global at 0.10 m.

**Planner: `nav2_theta_star_planner`**, `SmacPlanner2D` loaded as a second
named plugin for A/B. Unchanged and still right: any-angle paths mean few
ICR changes, which is what this steering chassis is bad at. Hybrid-A* and
lattice planners are rejected — they impose a turning radius the rover does
not have and forbid the point turn it does.

**Local sensing:** `cloud_filter` (0.05 m voxel, 8 m crop, 10 Hz) feeding a
forward-only `VoxelLayer`, plus a global `ObstacleLayer` from the same cloud
because the elevation map is only ~1 Hz. `nav2_collision_monitor` with
forward polygons only. **No reversing**: `allow_reversing: false`, `BackUp`
capped at 0.6 m, `vx_min ≥ −0.15`. Nothing looks backwards.

**Prior scan as a `StaticLayer`, OFF by default.** Enabled only when an
explicit alignment to the ZED's boot-time `map` frame exists. A wrong
alignment puts lethal cells metres from real rocks and looks entirely
plausible — still the single largest silent risk.

**Controller: `RotationShimController` wrapping
`RegulatedPurePursuitController`**, non-holonomic, `vy` pinned to 0 at the
smoother. RPP's curvature-continuous output is a slowly moving ICR — the
regime the IK never clips. MPPI configured but parked until CPU is measured.

## 6. Frames

One static `zed_front_camera_link → base_footprint` at `(−0.345, 0, −0.548)`
(the inverse of `CAMERA_IN_BASE_FOOTPRINT`), plus `base_footprint →
base_link` at `(0, 0, 0.409)`. The ZED wrapper stays the sole owner of
`map → odom → zed_front_camera_link`; no `robot_state_publisher` on the Orin,
which would give that link a second parent. `localization_status` gains
`/localization/odom_local` (odom frame) for Nav2's `odom_topic`.

## 7. Ground station

Autonomous mode becomes selectable. Additions, all JSON over rosbridge, no
`rclpy`:
- A **NAV row**: waypoint list (add by clicking the map view or typing),
  Go / Pause / Resume / Abort, and a status line fed by `/nav_status`
  (state, current waypoint, distance remaining, ETA, error).
- `/mode_status` drives a mode chip beside the DRIVE row's chips.
- The plan is drawn in the Gazebo mirror via `/nav_path_summary` (decimated;
  the raw `/plan` is too heavy for the field link).
- STOP stays live in every mode and is wired to `/estop_request` as well as
  the existing chassis stop.

## 8. Sub-projects

| SP | What | Depends on |
|---|---|---|
| **SP4** | Re-vendor `betterIK` (2.42, Asterope `hParams`) into `navi_sim_ik`; sim and rover run identical arithmetic | — |
| **SP5** | `mode_supervisor` + ground-station publish policy + `/rover_twist` single writer; `bema_bridge` fed from it | SP4 |
| **SP6** | Static frames, `/localization/odom_local` | — |
| **SP7** | `tile_aggregator` + `traversability_layer` (holes become lethal) | — |
| **SP8** | `navi_rpc_server` (:21021 on the `.18` alias, only the methods `startNaViTask` needs) + coordinator client for task state | SP5 |
| **SP9** | Nav2 bringup and offline planning against a recorded map | SP6, SP7 |
| **SP10** | `twist_shaper` (feasibility clamp on the real 2.42 IK) + path following in the sim | SP4, SP9 |
| **SP11** | Ground-station NAV row, `goal_relay`, plan drawing | SP5, SP8 |
| **SP12** | Yard tuning: steering slew, CPU, thresholds against real rocks, bag set | everything + rover |

SP4 first because it is a prerequisite for a safety component and needs no
rover. SP5 before any Nav2 node exists, so there is never a moment when two
things can publish to the chassis.

## 9. Testing ladder

1. **Pure functions** (laptop): traversability maths on synthetic grids
   including holes; supervisor state machine with fake clocks; NaVi RPC
   server against a fake coordinator client; `twist_shaper` clamping against
   the real 2.42 IK.
2. **Chassis double**: the whole supervisor → bridge → `fake_bema_server`
   chain, asserting wire traffic — including that autonomy commands stop at
   the bridge when the coordinator is not in `Autonomous`.
3. **Offline planning**: Nav2 plans on a recorded elevation map; asserts a
   path exists and avoids seeded lethal cells, including a pit.
4. **Kinematic sim**: the controller follows a plan in Gazebo on a throwaway
   domain; closure error measured; the height-banded view shows the plan.
5. **Sim avoidance**: a Gazebo depth camera at the ZED mount; the rover
   detours around a spawned box and refuses a pit.
6. **Rover day**: on blocks first (wheel-corner mapping and turn direction —
   still unverified), then yard. Measure steering slew and Orin CPU; replace
   the starting numbers.

## 10. Speeds

Manual is currently capped at **0.05 m/s / 0.1 rad/s** (a deliberate tenth
for the first hardware sessions). Autonomy starts at the same cap for SP10's
first runs, then rises **only** on measured evidence, in stages:
0.05 → 0.15 → 0.30 → 0.45 m/s, with `wz ≤ 0.4 rad/s` and `wz` acceleration  //only rize that if i specifically Tell you to
≤ 0.5 rad/s² until the steering slew is measured. Each stage needs one clean
sim run and one clean yard run. The cap lives in one place per path
(`gamepad_input.py` for manual, the velocity smoother for autonomy).

## 11. Risks

1. **VIO loss with TF still moving** → supervisor halts on status, not on TF.
2. **Coordinator veto** → the supervisor drives the mission state machine; if
   the state is not `Autonomous`, autonomy does not start and says why.
3. **Steering lag oscillation** → smoother limits plus `twist_shaper`;
   measure slew before raising speed.
4. **Nothing behind or beside** → forward-only monitor, no reversing.
5. **Two writers to the chassis** → single-writer rule, SP5 before Nav2.
6. **Orin CPU** at 0.05 m costmaps → measured in SP9; documented fallback.
7. **Prior scan misaligned** → layer off by default.
8. **Wheel-corner mapping still unverified** → blocks every autonomous run;
   the on-blocks check is a gate, not a formality.
9. **`.18` alias collides** if the LAN changes → the supervisor reports the
   NaVi server's bind failure instead of silently not serving.

## 12. Deferred

Rear and side sensing; reversing beyond 0.6 m; MPPI; prior-scan alignment
tooling; automatic resume after localisation recovery; probing, science and
maintenance tasks (the coordinator's other three task types);
**coordinator-initiated runs** — the NaVi server exists (§3) but is driven
only by our own Go button, and the unused half of its interface stays
stubbed.

## 13. Operator actions

`sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
ros-humble-grid-map-costmap-2d` on the Orin. No `twist-mux` [CHANGED] — the
supervisor replaces it. The Orin has no internet: wheels or debs must be
carried over, as `msgpack` was.
