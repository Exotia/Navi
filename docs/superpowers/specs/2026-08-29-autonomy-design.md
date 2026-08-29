# Full Autonomy — Design

Written 2026-08-29, from `docs/research/2026-08-29-autonomy-research.md`
(the research brief; read it for the option analysis and sources — this spec
records the decisions). Builds on the three localisation sub-projects (SP1
rover localisation, SP2 ground station and view, SP3 built map), all merged.

## Goal

The operator enters a list of waypoints in the organisers' frame, presses
Go, and the rover drives them on its own: plans a route over the ground it
has mapped (and, once an alignment exists, the organisers' scan), avoids
what the front ZED sees, follows the route with twists the 4-wheel-steered
chassis can actually execute, and reports progress — while the operator can
take over or stop it at any instant, and localisation loss halts it.

Out of scope: the arm, the rear ZED, science tasks, multi-rover, anything
needing sensing the rover does not have (rear, sides).

## Decisions

Each is the research brief's recommendation, adopted as a decision.

1. **Global planner: `nav2_theta_star_planner`**, with `SmacPlanner2D` loaded
   as a second named plugin for A/B. Any-angle paths mean few segments and
   therefore few ICR changes, which is what the steering chassis is bad at.
   Hybrid-A*/lattice are rejected: they impose a turning radius the rover
   does not have and forbid the point turn it does.
2. **Traversability from the elevation map, computed by us.** A
   `traversability_layer` node reads `/localization/map` (grid_map,
   `elevation`, 0.10 m) and derives `slope`, `step`, `roughness`, `valid`
   layers with thresholds: step lethal above **0.14 m** (just over the
   0.125 m wheel radius), slope lethal above 25°, roughness scaled. Output
   `/autonomy/traversability` (GridMap, for the view) and
   `/autonomy/costmap_seed` (`nav_msgs/OccupancyGrid`, latched, via
   `grid_map_ros`'s converter). The ROS 1 `traversability_estimation` package
   is the reference for the maths, not a dependency.
3. **Prior scan as a `StaticLayer` that is OFF by default.** It is rasterised
   offline with the same code as `terrain_writer` and enabled only when an
   explicit alignment to the ZED's boot-time `map` frame exists. A wrong
   alignment puts lethal cells metres from the rocks and looks plausible —
   the single biggest risk in the brief.
4. **Local sensing: `VoxelLayer` forward-only** from a downsampled
   (0.05 m voxel, 8 m crop) ZED cloud, plus a global-costmap `ObstacleLayer`
   because the elevation map is only 0.5 Hz. `nav2_collision_monitor` with
   forward polygons only. **No reversing**: `allow_reversing: false`,
   `BackUp` recovery capped at 0.6 m, `vx_min ≥ −0.15`. There is no rear
   sensing; the costmap's memory behind the rover rests on a pose that may
   have jumped.
5. **Path following: `RotationShimController` wrapping
   `RegulatedPurePursuitController`, non-holonomic, `vy` pinned to 0** at the
   velocity smoother. RPP's curvature-continuous output is a slowly moving
   ICR, the regime the IK's `feasable_ICR` never clips. MPPI is configured
   with `vy_max: 0.0` but parked; DWB is not used.
6. **Steering lag is handled outside the controller**: low `wz`
   acceleration and `scale_velocities: true` in the velocity smoother, and a
   `twist_shaper` node between the mux and the chassis that runs the
   vendored IK (`SimIkStepper`, ROS-free) as a feasibility oracle, clamping
   the twist to what `eta_dot_constrained` says the steering can follow and
   publishing `/ik_feasibility` (JSON) for the view.
7. **TF: one static `zed_front_camera_link → base_footprint`** at
   `(−0.345, 0, −0.548)` — the same constant as `CAMERA_IN_BASE_FOOTPRINT`,
   inverted — plus `base_footprint → base_link` (0, 0, 0.409). The ZED
   wrapper stays the sole owner of `map → odom → zed_front_camera_link`; no
   `robot_state_publisher` on the Orin (it would give `zed_front_camera_link`
   a second parent). `localization_status` gains `/localization/odom_local`
   (odom frame) for Nav2's `odom_topic`.
8. **Arbitration: rename the chassis input to `/rover_twist`; `twist_mux`
   in front of it** with inputs `/estop_twist` (255), `/manual_twist` (200),
   `/autonomy_twist` (100) and locks `/estop_lock`, `/autonomy_lock`;
   **and** a `mode_supervisor` state machine (`manual` / `semi_auto` /
   `autonomous` / `estop`) that on operator takeover cancels the Nav2 goal
   and deactivates Nav2's lifecycle. The mux alone would hand control back
   to a still-running Nav2 when the operator lets go of the stick — the
   dangerous case. The supervisor also watches `/localization/status` and
   halts on `SEARCHING`/`OFF`. Nav2 `autostart: false`; power-on state is
   manual.
9. **Goals over rosbridge as JSON on `/nav_request`**, relayed on the rover
   by `goal_relay` into `NavigateThroughPoses` (`NavigateToPose` kept for
   single-goal debugging), with `/nav_status` JSON back. Not
   `roslibpy.ActionClient`: roslibpy documents async cancel as unsupported
   over rosbridge, and cancel is the safety path. The ground station keeps
   speaking only `Twist` and `String`.
10. **Testing ladder** (brief §7): rung 1 pure functions; rung 2 Nav2 plans
    offline against the scan; rung 3 the controller follows a plan in the
    kinematic sim (which runs the real Simulink IK); rung 4 avoidance in the
    sim with a Gazebo depth camera at the ZED mount. Rover days measure the
    steering slew and CPU and replace the starting numbers.
11. **Costmap resolution 0.10 m**, equal to the elevation map — resampling
    smears the step edges we cannot afford to lose.
12. **One new package `navi_autonomy`** (ament_cmake with Python nodes, the
    `navi_sim_bringup` pattern, so `twist_shaper` can link the IK).

## Constraints

- The operator always wins, instantly: a `/manual_twist` message pre-empts
  autonomy at the mux and the supervisor cancels the goal within one tick.
- STOP is always live in every mode, even with rosbridge dropping: the
  mux's estop lock latches on the rover and is cleared only by an explicit
  `/mode_request` back to manual.
- Halt on localisation `SEARCHING` or `OFF` (supervisor publishes a zero
  `/autonomy_twist` and cancels the goal; the pose freeze from SP1 is not
  enough because the ZED's TF does not freeze).
- Velocity limits for the yard: `vx ≤ 0.45 m/s`, `wz ≤ 0.4 rad/s`,
  `wz` accel `≤ 0.5 rad/s²` until the steering slew is measured.
- Nothing under `ground_station/` imports `rclpy`; the simulation stays a
  consumer (bridged topics grow by `/rover_twist`, `/plan`, `/nav_status`).
- The vendored IK is read-only; `twist_shaper` calls it, never edits it.
- `/manual_twist` never receives test traffic; tests use throwaway domains.

## Architecture

See the brief §(a) for the full diagram. In one line per hop:

```
GS (Twist+JSON over rosbridge) → /manual_twist, /nav_request, /estop_request, /mode_request
Orin: ZED → localisation (pose, odom_local, status) → static TF → Nav2 (global/local costmaps,
      Theta*, BT navigator, RotationShim+RPP, smoother, collision monitor) → /autonomy_twist
      twist_mux(/estop_twist 255, /manual_twist 200, /autonomy_twist 100) → /rover_twist
      → twist_shaper (IK feasibility oracle) → chassis (bemacontroller)
      goal_relay (/nav_request ↔ NavigateThroughPoses ↔ /nav_status)
      mode_supervisor (/mode_request, /estop_request, /localization/status → locks, lifecycle, /mode_status)
Laptop sim (domain 42): sim_bridge adds /rover_twist, /plan, /nav_status; the Gazebo view draws the plan.
```

## Components and interfaces

| component | in | out | notes |
|---|---|---|---|
| `twist_mux` (upstream pkg) | `/estop_twist` 255, `/manual_twist` 200, `/autonomy_twist` 100; locks `/estop_lock`, `/autonomy_lock` | `/rover_twist` | timeouts 0.5 s on every input |
| `twist_shaper` (navi_autonomy) | `/rover_twist` | chassis topic, `/ik_feasibility` JSON | reuses `SimIkStepper`; clamps to feasible ICR |
| `mode_supervisor` | `/mode_request`, `/estop_request` (String JSON), `/localization/status`, `/manual_twist` activity | `/mode_status` JSON `{mode, reason, nav_active, estop}`, `/estop_lock`, `/autonomy_lock`, `/estop_twist`, lifecycle transitions, goal cancel | 20 Hz tick; takeover = any `/manual_twist` with non-zero magnitude while autonomous |
| `static_frames` | — | `/tf_static`: `zed_front_camera_link→base_footprint`, `base_footprint→base_link` | one constant, imported from `pose_composition` |
| `localization_status` (+) | `/zed_front/zed_node/odom` | `/localization/odom_local` (odom→base_footprint) | SP5 |
| `cloud_filter` | `/zed_front/zed_node/point_cloud/cloud_registered` | `/autonomy/points_filtered` | voxel 0.05 m, crop 8 m, 10 Hz |
| `traversability_layer` | `/localization/map` | `/autonomy/traversability` (GridMap), `/autonomy/costmap_seed` (OccupancyGrid, latched) | thresholds as parameters |
| `prior_map` (offline tool + map_server) | scan mesh + alignment file | `/autonomy/prior_map` (latched) | disabled unless alignment present |
| Nav2 (bringup with our YAML) | above | `/plan`, `/cmd_vel` → smoother → collision monitor → `/autonomy_twist` | `autostart: false` |
| `goal_relay` | `/nav_request` JSON `{action: go|cancel, waypoints:[{x,y,yaw}], frame}` | `/nav_status` JSON `{state, current_waypoint, distance_remaining, eta, error}` | validation like `video_request.py` |
| ground station | `/mode_status`, `/nav_status`, `/plan` (decimated via `/nav_path_summary`) | `/nav_request`, `/mode_request`, `/estop_request` | Autonomous mode enabled; STOP button always live |

## Error handling

- Localisation lost while autonomous: supervisor → estop-like halt
  (`/autonomy_lock`, zero twist), goal cancelled, `/mode_status.reason =
  "localisation SEARCHING"`; automatic resume is **not** done — the operator
  re-issues Go after the marker returns to `LOCALISED`.
- Rosbridge drop while autonomous: Nav2 keeps driving the current plan (it
  lives on the rover) unless the supervisor's `/manual_twist` watchdog or
  localisation halts it; the GS shows the last `/nav_status` greyed.
  Decision: autonomy does **not** stop on link loss alone (ERC expects
  autonomous runs to survive link loss), but the estop lock does.
- Planner failure / no path: `/nav_status.state = "no_path"`, rover
  stationary, operator decides.
- Collision monitor stop: reported in `/nav_status.error`; recovery is
  Nav2's (spin, wait), never a reverse beyond 0.6 m.
- twist_shaper infeasible command: clamped, `/ik_feasibility` shows it; a
  persistent clamp above 50 % for 2 s is logged as a warning.

## Testing

Rung 1 (pure, laptop): traversability maths on synthetic grids; mode
supervisor state machine with fake clocks; goal_relay JSON validation;
twist_shaper clamping against the real IK. Rung 2: Nav2 planning offline on
the rasterised scan (laptop has nav2_bringup); asserts a plan exists and
avoids seeded lethal cells. Rung 3: the kinematic sim on throwaway domains
follows a plan with `twist_topic:=/rover_twist`; closure error measured.
Rung 4: Gazebo depth camera → voxel layer → the sim rover detours around a
spawned box. Rover day: steering slew, CPU under full stack, thresholds
against real rocks, bag set.

## Sub-projects

SP4 arbitration → SP5 frames → SP6 traversability → SP7 offline planning →
SP8 following → SP9 GS goals → SP10 yard tuning. Dependencies as the brief's
graph: SP4 and SP5 need only SP1; SP6 needs SP3; SP7 needs SP5+SP6; SP8 needs
SP4+SP7 (+SP2 for the view); SP9 needs SP4+SP7+SP2; SP10 needs everything and
a rover.

## Deferred

- Rear ZED and side sensing; reversing beyond 0.6 m.
- MPPI activation (needs the measured slew and a CPU budget).
- Prior-scan alignment tooling (ArUco or manual) — SP3's deferred item; the
  layer exists but stays off.
- Autonomous resume after localisation recovery.

## Risks (top 8, from the brief)

1. VIO loss with TF still moving → supervisor halt on status (SP4).
2. Steering lag oscillation → smoother limits + twist_shaper; measure slew (SP10).
3. Nothing behind/beside → forward-only monitor, no reversing, 0.6 m cap.
4. Two chassis publishers during transition → `/rover_twist` rename lands
   first, in SP4, before any Nav2 node exists.
5. 0.5 Hz map → global `ObstacleLayer` from the live cloud too.
6. Orin CPU → RPP not MPPI; voxel 0.05; measure (SP10).
7. Prior scan misaligned → layer off by default.
8. Operator takeover leaves Nav2 running → supervisor cancels + deactivates.

## User actions

`sudo apt install ros-humble-twist-mux ros-humble-grid-map-costmap-2d` on
the Orin (and twist_mux on the laptop for the sim closed loop).
