# Full Autonomy for Asterope — Technical Research Brief

Date: 2026-08-29. Status: **research only**. This document is input to a later
design session; it decides nothing on its own.

Scope: what it would take to make the Asterope rover drive itself to
operator-given waypoints in the ERC Mars yard, on top of the localisation work
(sub-projects 1–3 of `docs/superpowers/specs/2026-08-29-localisation-design.md`)
and the kinematic simulation
(`docs/superpowers/specs/2026-08-28-rover-simulation-design.md`).

---

## 0. What already exists, stated as interfaces

Everything below is read off this repo, not assumed.

| Producer | Topic | Type | Notes |
|---|---|---|---|
| `localization_status` (Orin) | `/localization/pose` | `nav_msgs/Odometry` | `frame_id: map`, `child_frame_id: base_footprint`, ~30 Hz, covariance from ZED. **Frozen stamp while SEARCHING.** |
| `localization_status` (Orin) | `/localization/status` | `std_msgs/String` (JSON) | `{"state":"OK"\|"SEARCHING"\|"OFF","seconds_since_ok":f,"source":"zed_vio","distance_travelled":f,"mount_offset_verified":b}` at 2 Hz + on change |
| `elevation_mapper` (Orin, SP3) | `/localization/map` | `grid_map_msgs/GridMap` | one `elevation` layer, 0.10 m cells, `map` frame, grows to 60×60 m, 0.5 Hz **only when changed** |
| `zed_wrapper` (Orin) | `/tf` | — | `publish_tf: true`, `publish_map_tf: true` → `map→odom` and `odom→zed_front_camera_link`. Tracking base frame is hard-coded to `zed_front_camera_link` (wrapper 4.2). |
| `zed_wrapper` (Orin) | `/zed_front/zed_node/point_cloud/cloud_registered`, `/zed_front/zed_node/depth/depth_registered`, `.../mapping/fused_cloud` | `sensor_msgs/PointCloud2`, `Image` | depth 0.3–10 m |
| Ground station (laptop) | `/manual_twist` | `geometry_msgs/Twist` | published over rosbridge from the gamepad; **this is what drives the physical rover today** |
| GS ↔ Orin | `/video_request`, `/video_status` | `std_msgs/String` (JSON) | the established "JSON in a String" convention, chosen so rosbridge needs no custom type discovery |
| `sim_ik_node` (laptop) | in `/manual_twist`; out `/set_joint_trajectory`, `/sim_cmd_vel`, `/sim_odom`, `/sim_ik_debug` | — | ticks at `kTimestepSeconds = 0.06` on the node's own (sim) clock |

**Chassis, from `sim/build/navi_sim_bringup/asterope_base.urdf`:**
steer modules at `(±0.455, ±0.455, −0.284)` relative to `base_link` →
**wheelbase 0.91 m, track 0.91 m**, half-diagonal 0.644 m.
`base_footprint` is 0.409 m below `base_link` origin, so the wheel axle sits
0.125 m above ground → **wheel radius ≈ 0.125 m**. Steer joints are revolute
±π with a URDF-nominal velocity limit of 6.0 rad/s (nominal, not hardware).
Body box 0.604 × 0.410 × 0.254 m; a conservative circumscribed radius including
wheels is **0.70 m**.

**IK model I/O** (`sim/src/navi_sim_ik/vendor/ert_rtw/kinematics.h`):
```
in : VX_out, VY_out, U_p, beta_hat[4], beta_dot_hat[4], TS
out: Beta_dot[4], omega[4], beta_next[4], indirect_mode,
     input_ICR, controller_ICR, feasable_ICR, current_ICR, border_ICR,
     eta_dot_constrained[3], eta_dot_ref_init[3]
```
The single most important output for autonomy is **`eta_dot_constrained`** —
`SimIkStepper::achieved_velocity()` exposes it as `(vx, vy, yaw_rate)` in the
body frame. It is *what the chassis will actually do* after the ICR feasibility
optimisation has clipped the request. Any autonomy stack that ignores it will
plan against a vehicle model that does not exist.

**Installed on the laptop** (`ls /opt/ros/humble/share`): the whole Nav2 set
including `nav2_smac_planner`, `nav2_mppi_controller`,
`nav2_regulated_pure_pursuit_controller`, `nav2_rotation_shim_controller`,
`nav2_theta_star_planner`, `nav2_collision_monitor`, `nav2_velocity_smoother`,
`nav2_route`, `slam_toolbox`, `grid_map_core/cv/msgs/ros`, `pcl_conversions`.
**Not installed: `grid_map_costmap_2d`, `twist_mux`, `nav2_simple_commander`
is present but `grid_map_costmap_2d` is not** — it *is* released for Humble
(v2.0.1, `ros-humble-grid-map-costmap-2d`), so it is one `apt install`, but the
design must not assume it is already there.
Sources: [grid_map_costmap_2d on index.ros.org](https://index.ros.org/p/grid_map_costmap_2d/),
[grid_map](https://github.com/ANYbotics/grid_map).

**ERC context.** The Navigation task is a traverse to a set of checkpoints,
scored higher when done autonomously and without video feedback; markers on
site are ArUco. Sources:
[ERC robotics competition](https://roverchallenge.eu/robotics-competition/),
[URC 2026 rules](https://www.scribd.com/document/933955225/University-Rover-Challenge-Rules-2026)
(sibling competition, useful for marker/tolerance conventions: 4x4_50 ArUco,
"within 2 m of the post" counts as reached).

---

## 1. Global path planning on a 2.5-D elevation map

### Options

**(a) Feed the elevation grid into a costmap layer.** No stock Nav2 layer eats
`grid_map_msgs/GridMap`. Either convert grid_map → `nav_msgs/OccupancyGrid`
(`grid_map_ros`'s `GridMapRosConverter::toOccupancyGrid(map, layer, dataMin, dataMax, msg)`,
already installed) and use a stock `StaticLayer`; or write a custom
`nav2_costmap_2d::Layer` that subscribes the GridMap directly — more code, one
fewer hop, and access to several layers at once.

**(b) Where the traversability number comes from.** The elevation layer alone
is not a cost. The established recipe (ANYbotics `elevation_mapping` +
`leggedrobotics/traversability_estimation`) is a `grid_map` **filter chain**:
surface normals → **slope** (normal tilt vs a critical angle), **roughness**
(deviation from the fitted plane), **step** (max Δz to neighbours in a window),
combined as a weighted product or min. `traversability_estimation` is **ROS 1
only** — copy the maths, not the dependency. Sources:
[traversability_estimation](https://github.com/leggedrobotics/traversability_estimation),
[elevation_mapping](https://github.com/ANYbotics/elevation_mapping).

**(c) Planner choice.**
- `NavFn` (Dijkstra/A* on the 2-D grid) — fast, ignores kinematics, produces
  paths with corners the chassis cannot take without stopping to re-steer.
- `nav2_theta_star_planner` — any-angle, produces long straight runs. On open
  sand this is attractive: fewer, longer segments = fewer steering reversals.
- `SmacPlanner2D` — cost-aware 8-connected A*, circular robots, still
  kinematically unconstrained.
- `SmacPlannerHybrid` — Hybrid-A* with Dubins/Reeds-Shepp, `minimum_turning_radius`,
  `reverse_penalty`, SE2 collision checking against the real footprint.
- `SmacPlannerLattice` — state lattice with a supplied minimum control set,
  including an omnidirectional set.
Source: [Smac Planner README](https://docs.ros.org/en/ros2_packages/humble/api/nav2_smac_planner/),
[Smac Hybrid config](https://ros.ncnynl.com/en/nav2/configuration/packages/smac/configuring-smac-hybrid.html).

### Trade-offs for this rover

The chassis is *not* Ackermann and *not* diff-drive. It can rotate in place
(ICR at the centre) and it can crab, but every change of ICR costs time while
the four steer modules slew. That means:

- A **kinematically-unconstrained planner is not wrong** here — the chassis can
  in principle execute any 2-D path — but a path with many curvature changes is
  slow, because each one is a steering transient the IK will limit
  (`feasable_ICR` clips it, and `eta_dot_constrained` reports the clipped
  result). What we want is not "feasible" so much as **"few segments"**.
- Hybrid-A* would impose a minimum turning radius the rover does not actually
  have, which throws away the rover's best trick (point turns at waypoints)
  and produces long sweeping curves on open sand for no reason.
- The map is 0.10 m cells over ≤60×60 m = 360 000 cells. Any of these planners
  handles that in well under a second on an Orin.

### Recommendation

**`nav2_theta_star_planner` as the primary global planner, `SmacPlanner2D`
configured as a second named plugin for comparison.** Theta* gives long
straight-line segments across open sand — exactly the shape the 4WS chassis
executes fastest — and its any-angle property means far fewer curvature changes
than NavFn's grid-aligned output. Keep `SmacPlanner2D` loaded under a second
plugin name so a design session can A/B them from the BT without a rebuild.
**Do not use Hybrid-A*** unless the yard turns out to be tight enough that
pose-based footprint collision checking matters; record it as the fallback.

**Traversability derivation: a new node `traversability_layer`** (Python or C++
in a new `navi_autonomy` package) that:
1. subscribes `/localization/map` (`grid_map_msgs/GridMap`, `elevation` layer),
2. computes, per cell, from a 3×3 (0.30 m) and 5×5 (0.50 m) window:
   - `slope` = angle of the plane fitted over the window,
   - `step` = max |Δz| to any neighbour in the window,
   - `roughness` = RMS residual of the points about that fitted plane,
   - `valid` = fraction of non-NaN cells in the window,
3. maps them to cost 0–254 with hard thresholds and a soft ramp between,
4. publishes **both** `/autonomy/traversability` (`grid_map_msgs/GridMap`, extra
   layers, for the ground station and for debugging) and
   `/autonomy/costmap_seed` (`nav_msgs/OccupancyGrid`, latched
   `transient_local`) via `GridMapRosConverter::toOccupancyGrid`.

Starting thresholds for a Mars yard with ±1.5 m relief over 40 m (mean grade
~4°, so local slope is what matters, not global):

```
slope:      free < 10°,   ramp 10°→20°,  lethal > 20°
step:       free < 0.06 m, ramp 0.06→0.14 m, lethal > 0.14 m   # wheel r = 0.125 m
roughness:  free < 0.02 m, ramp 0.02→0.06 m, lethal > 0.06 m
valid:      < 0.4 of window observed  -> NO_INFORMATION (255), not lethal
cost = 254 * max(norm_slope, norm_step, norm_roughness)   # worst-of, not average
```
`step` is the binding constraint and it is deliberately just above half the
wheel radius: a 0.14 m rock is a stop for a 0.125 m wheel.

**Seeding from the organisers' prior scan.** The repo already carries
`Model3D_mesh2.obj` (161 MB, 840 753 verts, 37.4 × 43.8 m, z ∈ [−1.50, +1.03]).
Offline, before the run, rasterise it to the same 0.10 m grid and save it as a
`map_server`-loadable pair (`.pgm` + `.yaml`) — the same pipeline
`terrain_writer` (SP3) already uses to make a Gazebo heightmap PNG, so the code
is mostly shared. Then the **global costmap has two static-ish inputs**:

```
global_costmap plugins: ["prior_layer", "live_layer", "obstacle_layer", "inflation_layer"]
  prior_layer : StaticLayer,  map_topic: /autonomy/prior_map        (from the scan, latched)
  live_layer  : StaticLayer,  map_topic: /autonomy/costmap_seed     (from /localization/map, latched)
```
Nav2's `StaticLayer` overwrites rather than merges, so **two StaticLayers in
sequence gives exactly "live beats prior"** as long as the live layer publishes
`NO_INFORMATION` (255) where it has seen nothing — which is why `valid` above
maps to 255 and not to free space. That is the whole trick and it needs a test.

### Concrete starting configuration

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 1.0
    planner_plugins: ["GridBased", "SmacAlt"]
    GridBased:
      plugin: "nav2_theta_star_planner::ThetaStarPlanner"
      how_many_corners: 8
      w_euc_cost: 1.0 ; w_heuristic_cost: 1.0
      w_traversal_cost: 2.0     # raise to hug low-cost sand, lower to go straight
      allow_unknown: true       # the yard is mostly unseen at t=0
      use_final_approach_orientation: false
    SmacAlt:
      plugin: "nav2_smac_planner::SmacPlanner2D"
      tolerance: 0.5 ; allow_unknown: true ; max_planning_time: 2.0
      cost_travel_multiplier: 2.0 ; use_final_approach_orientation: false

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0 ; publish_frequency: 0.5
      global_frame: map ; robot_base_frame: base_footprint
      rolling_window: false
      width: 64 ; height: 64 ; origin_x: -32.0 ; origin_y: -32.0
      resolution: 0.10          # matches /localization/map exactly - do not resample
      robot_radius: 0.70        # circumscribed, from the URDF
      track_unknown_space: true
      plugins: ["prior_layer", "live_layer", "obstacle_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 2.0
        inflation_radius: 1.20   # ~1.7 x robot_radius; sand, no tight aisles
      always_send_full_costmap: false
```
Resolution 0.10 m is chosen to equal the elevation map's cell size. Resampling
a 2.5-D map into a different costmap resolution silently smears step edges,
which is exactly the feature we need preserved.

---

## 2. Local planning and obstacle handling with one forward stereo camera

### Options

- **`ObstacleLayer` with `data_type: "PointCloud2"`** — 2-D raycast
  marking/clearing. Cheap, but flattens everything between
  `min_obstacle_height` and `max_obstacle_height` into one plane and clears
  overhanging geometry badly.
- **`VoxelLayer`** — full 3-D raycast (`z_voxels` / `z_resolution`), correct
  clearing above and below obstacles, squashed to 2-D for planning. More CPU;
  needs a well-behaved `sensor_frame`.
  Sources: [obstacle layer](https://ros.ncnynl.com/en/nav2/configuration/packages/costmap-plugins/obstacle.html),
  [voxel layer](https://ros.ncnynl.com/en/nav2/configuration/packages/costmap-plugins/voxel.html).
- **`nav2_collision_monitor`** — an independent safety node filtering the
  controller's `cmd_vel` against polygon/circle zones fed from *raw* sensor
  data (stop / slowdown / approach; most-aggressive zone wins). Documented at
  ~4–5 ms per 24 K-point cloud for stop/slowdown polygons, ~20 ms for
  approach-with-footprint. Source:
  [Collision Monitor README](https://docs.ros.org/en/ros2_packages/humble/api/nav2_collision_monitor/).

### Trade-offs for this rover

- **The ZED 2i is the only sensor and it points forward** (HFOV ~110°, depth
  0.3–10 m). Nothing behind, nothing beyond ~55° off-axis, nothing inside 0.3 m.
- Mars-yard rocks are the voxel layer's case: a 0.2 m rock at 4 m is a handful
  of points, and a 2-D raytrace from a camera at 0.548 m will re-clear it next
  frame if the heights are slightly wrong.
- **Reversing is unsafe by construction** — no rear sensing, and the costmap's
  rear memory is only as good as a pose with no wheel-odometry fallback. 1 m of
  reverse is a bounded risk; 5 m is not.
- The local costmap can be **rolling and small**: at ≤0.5 m/s an 8 × 8 m window
  is far more lookahead than the controller's horizon.

### Recommendation

- **Local costmap: `VoxelLayer`, not `ObstacleLayer`**, fed from the ZED's
  registered cloud, downsampled first. Add a `voxel_grid`/`pcl_ros` filter node
  (`pcl_conversions` is installed; confirm `pcl_ros` on the Orin) at 0.05 m leaf
  and a range crop to 8 m, so the layer and the collision monitor both see a
  few thousand points instead of ~1 M.
- **Also put a plain `ObstacleLayer` on the *global* costmap** from the same
  filtered cloud, so newly-seen rocks affect replanning, not only the local
  window. Global inflation then does the rest.
- **Run `nav2_collision_monitor` as the last stage before the chassis.** Its
  `cmd_vel_in_topic`/`cmd_vel_out_topic` design is exactly the shape we need,
  and it works on raw sensor data — it is not fooled by a stale costmap.
- **Reversing: allow, but bound it.** `backup_dist` no more than 0.6 m in the
  recovery behaviours, `vx_min` in the controller no lower than −0.15 m/s, and
  a `PolygonStop` zone that only covers the front. Behind the rover we have no
  evidence, so the rule is "reverse slowly, briefly, and only as a recovery" —
  never as a planned manoeuvre. That in turn argues for `allow_reversing: false`
  in the path follower and for **not** using Reeds-Shepp in any planner.

### Concrete starting configuration

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0 ; publish_frequency: 2.0
      global_frame: odom            # REP-105: local costmap in odom, not map
      robot_base_frame: base_footprint
      rolling_window: true
      width: 8 ; height: 8 ; resolution: 0.10 ; robot_radius: 0.70
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        publish_voxel_map: true       # false once tuned; it is expensive
        origin_z: 0.0 ; z_resolution: 0.10 ; z_voxels: 12   # 1.2 m vertical extent
        max_obstacle_height: 1.20 ; mark_threshold: 1 ; combination_method: 1
        observation_sources: zed_cloud
        zed_cloud:
          topic: /autonomy/points_filtered      # the downsampled cloud, not the raw one
          data_type: "PointCloud2"
          sensor_frame: zed_front_left_camera_frame
          min_obstacle_height: 0.10   # below this is ground on sand
          max_obstacle_height: 1.20
          obstacle_max_range: 6.0     # ZED depth is honest to ~10 m; trust 6
          obstacle_min_range: 0.4
          raytrace_max_range: 7.0 ; raytrace_min_range: 0.3
          marking: true ; clearing: true
          expected_update_rate: 0.5   # warns if the ZED cloud dies
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 2.5
        inflation_radius: 1.00

collision_monitor:
  ros__parameters:
    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    cmd_vel_in_topic: "cmd_vel_smoothed"     # after the velocity smoother
    cmd_vel_out_topic: "autonomy_twist"      # into the mode supervisor, not the chassis
    transform_tolerance: 0.3
    source_timeout: 1.0
    base_shift_correction: true
    stop_pub_timeout: 2.0
    polygons: ["PolygonStop", "PolygonSlow"]
    # Both forward-only: with no rear sensing a rear zone could only ever be
    # triggered by stale data.
    PolygonStop:
      type: "polygon" ; action_type: "stop" ; max_points: 4
      points: [1.10, 0.60, 1.10, -0.60, 0.55, -0.60, 0.55, 0.60]
    PolygonSlow:
      type: "polygon" ; action_type: "slowdown" ; slowdown_ratio: 0.35 ; max_points: 4
      points: [1.80, 0.85, 1.80, -0.85, 0.55, -0.85, 0.55, 0.85]
    observation_sources: ["zed_cloud"]
    zed_cloud:
      type: "pointcloud" ; topic: "/autonomy/points_filtered"
      min_height: 0.12 ; max_height: 1.20 ; enabled: true
```
(Shape and parameter names taken from the installed
`/opt/ros/humble/share/nav2_collision_monitor/params/collision_monitor_params.yaml`.)

---

## 3. Path following → twist

### How the three candidates produce `cmd_vel`

- **Regulated Pure Pursuit (RPP).** Carrot on the path at `lookahead_dist`
  (optionally velocity-scaled) → arc curvature → `angular.z = linear.x * curvature`.
  Then *regulates* `linear.x` down on high curvature
  (`use_regulated_linear_velocity_scaling`, `regulated_linear_scaling_min_radius`,
  `..._min_speed`) and near cost. Can rotate in place first
  (`use_rotate_to_heading`, `rotate_to_heading_min_angle`). Purely
  non-holonomic — never emits `vy`. Source:
  [RPP config](https://ros.ncnynl.com/en/nav2/configuration/packages/configuring-regulated-pp.html).
- **DWB.** Grid-samples `(vx, vy, wz)` inside the acceleration window, rolls out
  over `sim_time`, scores with critics. Holonomic-capable. Its motion model is a
  **first-order acceleration limit** — no notion of "this turn needs 0.4 s of
  steering slew first".
- **MPPI.** Samples `batch_size` noised control sequences over
  `time_steps × model_dt` under `motion_model` (`DiffDrive`/`Omni`/`Ackermann`),
  scores with plugin critics, softmax-weights into one control. `model_dt`
  should equal the control period and never exceed it. Sources:
  [MPPI config](https://ros.ncnynl.com/en/nav2/configuration/packages/configuring-mppic.html),
  [MPPI README](https://docs.ros.org/en/ros2_packages/humble/api/nav2_mppi_controller/).

### The real problem: none of them models steering lag

The chassis's dynamics are not `(vx, vy, wz)` with acceleration limits. They are
a requested ICR, a current ICR, and a bounded rate of movement between them
(`Beta_dot`; `feasable_ICR` is the projection of the request onto what is
reachable this tick). Two consequences:

1. **A commanded twist is a request, not a promise.** The IK reports what it
   will actually deliver in `eta_dot_constrained`. Any controller with a
   velocity-feedback input should be given *that*, not the command.
2. **Discontinuous `vy` is the worst case.** `vy = 0 → 0.2` slews every module
   ~90°, during which the rover barely moves and the controller sees no
   response — the classic setup for oscillation. Continuous `vx` with
   slowly-varying `wz` is what the chassis is good at.

### Recommendation

**Run non-holonomic (`vy = 0`) in the controller, and get holonomy from the
recovery/alignment behaviours instead.**

Concretely: **`nav2_rotation_shim_controller` wrapping
`RegulatedPurePursuitController` as the primary.** Justification:

- RPP's output is a *smooth, curvature-continuous* `(vx, wz)`. Fed to the ICR
  controller, a slowly-varying curvature is a slowly-moving ICR, which is
  exactly the regime `feasable_ICR` never has to clip. This is the single
  biggest fit argument.
- The rotation shim solves the one thing RPP does badly for a 4WS rover: at the
  start of a path, or after a replan, the heading error can be large, and RPP
  would drive a long arc. The shim rotates in place first — which this chassis
  does natively (ICR at the body centre) — then hands over. `angular_dist_threshold`
  is the knob.
- MPPI is the more capable controller and the better long-term answer, but it
  needs `motion_model: "Omni"` to exploit the chassis, and Omni sampling is the
  exact thing that produces the `vy` chatter the steering cannot follow. Keep it
  configured as a second `controller_plugins` entry, with `vy_max: 0.0`
  initially, and only open `vy` after the steering-lag behaviour is measured on
  the real rover.
- DWB is strictly dominated by MPPI here; do not spend time on it.

**Encode the steering-rate constraint in three places, none of them the controller:**

1. **`nav2_velocity_smoother`** with deliberately low `max_accel`/`max_decel`
   on `wz`. A steering module slewing at ~1 rad/s across a 0.455 m half-track
   means the achievable `dwz/dt` at 0.4 m/s is roughly 0.6–0.9 rad/s². Set it
   conservatively at 0.5 rad/s² until measured.
2. **`vy` clamped to exactly 0** at the smoother (`max_velocity[1] = 0.0`),
   so nothing downstream can emit a crab command by accident.
3. **A `twist_shaper` node** (ours) between the smoother and the chassis, which
   runs the *same* `kinematics` model at TS = 0.06 as a **feasibility oracle**:
   step it with the requested twist, read `eta_dot_constrained` and
   `feasable_ICR`, and if the delivered twist differs from the request by more
   than a threshold, publish the *request scaled down along the achievable
   direction* rather than the raw request, and say so on a diagnostics topic.
   `sim/src/navi_sim_ik` is already a ROS-free library (`SimIkStepper`) that
   does exactly this — it can be reused verbatim on the Orin. **This is the
   highest-leverage piece of custom code in the whole stack** and it exists.

### Concrete starting configuration

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 16.667      # 1/0.06 - match the IK's TS exactly
    min_x_velocity_threshold: 0.02 ; min_theta_velocity_threshold: 0.02
    min_y_velocity_threshold: 1.0     # effectively disable vy reporting
    failure_tolerance: 0.5
    odom_topic: "/localization/odom_local"     # see section 4
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath", "MppiAlt"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.30
      movement_time_allowance: 20.0   # sand + steering transients are slow

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.50         # ERC-style "within 2 m" is far looser
      yaw_goal_tolerance: 6.28        # heading at a waypoint is not scored
      stateful: true

    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      primary_controller: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      angular_dist_threshold: 0.60          # ~34 deg: rotate rather than arc
      forward_sampling_distance: 1.0
      rotate_to_heading_angular_vel: 0.45
      max_angular_accel: 0.5 ; simulate_ahead_time: 1.0
      rotate_to_goal_heading: false

      # RPP parameters (read by the shim's primary controller)
      desired_linear_vel: 0.45
      lookahead_dist: 1.20 ; min_lookahead_dist: 0.80 ; max_lookahead_dist: 2.00
      lookahead_time: 2.5 ; use_velocity_scaled_lookahead_dist: true
      use_rotate_to_heading: true ; rotate_to_heading_min_angle: 0.60
      allow_reversing: false                # no rear sensing - see section 2
      use_regulated_linear_velocity_scaling: true
      regulated_linear_scaling_min_radius: 1.20   # > wheelbase; sand, not aisles
      regulated_linear_scaling_min_speed: 0.15
      use_cost_regulated_linear_velocity_scaling: true
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 1.5
      min_approach_linear_velocity: 0.08 ; approach_velocity_scaling_dist: 1.0
      max_angular_accel: 0.5 ; transform_tolerance: 0.3

    MppiAlt:                                # parked, for later evaluation
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 40
      model_dt: 0.06                        # == controller period == IK TS
      batch_size: 1000                      # Orin CPU; 2000 is the doc default
      vx_max: 0.45 ; vx_min: -0.15 ; wz_max: 0.60
      vy_max: 0.0                           # HOLD AT ZERO until steering lag is measured
      vx_std: 0.15 ; vy_std: 0.05 ; wz_std: 0.25
      ax_max: 0.6 ; ax_min: -0.6 ; az_max: 0.5
      motion_model: "Omni"                  # Omni + vy_max 0 == diff-drive that can later crab
      prune_distance: 2.5 ; temperature: 0.3 ; gamma: 0.015
      visualize: false ; reset_period: 1.0
      critics: ["ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic",
                "PathAlignCritic", "PathFollowCritic", "PathAngleCritic",
                "PreferForwardCritic", "TwirlingCritic"]
      # Deltas from the documented defaults, which are otherwise good:
      TwirlingCritic:     {cost_weight: 12.0}   # punish steering churn - the key change
      PreferForwardCritic:{cost_weight: 8.0, threshold_to_consider: 0.6}
      PathFollowCritic:   {cost_weight: 5.0, offset_from_furthest: 5}

velocity_smoother:
  ros__parameters:
    smoothing_frequency: 16.667
    scale_velocities: true            # keep the (vx,wz) ratio when clamping - preserves curvature
    feedback: "OPEN_LOOP"             # no wheel odometry to close the loop with
    max_velocity: [0.50, 0.0, 0.60]   # vy pinned to zero at the gate
    min_velocity: [-0.15, 0.0, -0.60]
    deadband_velocity: [0.02, 0.0, 0.03]
    velocity_timeout: 0.5             # stop 0.5 s after cmd_vel stops arriving
    max_accel: [0.40, 0.0, 0.50]      # wz accel is the steering-rate proxy
    max_decel: [-0.60, 0.0, -0.80]
    odom_topic: "/localization/odom_local"
    odom_duration: 0.1
```
`scale_velocities: true` matters more than it looks: when the smoother has to
clamp, scaling both components preserves the commanded curvature, so the ICR
does not move as a side effect of a speed limit.
Source: [velocity smoother config](https://ros.ncnynl.com/en/nav2/configuration/packages/configuring-velocity-smoother.html).

---

## 4. Nav2 architecture fit — and the TF problem

### The TF problem, stated precisely

Nav2 needs `map → odom → base_link` published on `/tf` (REP-105), plus the
robot's static tree from `robot_state_publisher`. The costmaps look up
`global_frame ← robot_base_frame` at every update; the controller does it at
every control cycle. A topic is not a substitute.
Source: [Nav2 transforms setup](https://ros.ncnynl.com/en/nav2/setup_guides/transformation/setup_transforms.html).

Today on the Orin:
- the ZED wrapper **already publishes** `map → odom` and `odom → zed_front_camera_link`
  (`publish_tf: true`, `publish_map_tf: true`);
- the wrapper's tracking base is **hard-coded to `zed_front_camera_link`**, so the
  chain ends at the camera, not at `base_footprint`;
- **there is deliberately no `robot_state_publisher` on the Orin** — the SP1
  design says so explicitly, because a second parent for `zed_front_camera_link`
  would break the tree;
- `/localization/pose` is a *derived* topic (`T_map_footprint = T_map_camera · inv(T_footprint_camera)`),
  not a TF frame.

### Options

**(a) A static transform, keeping the ZED as TF owner.** Publish the *inverse*
mount offset as a static `zed_front_camera_link → base_footprint` at
`(−0.345, 0, −0.548)`. The tree becomes
`map → odom → zed_front_camera_link → base_footprint` — one root, valid, and
`robot_base_frame: base_footprint` resolves. `base_footprint` ends up a *child*
of the camera, which is upside down relative to the URDF but topologically fine.

**(b) Turn off the wrapper's TF and publish everything ourselves.** A
`localization_tf` node publishes `map → base_footprint` from
`/localization/pose`, plus `robot_state_publisher`. Cleaner semantics, but
throws away the wrapper's `odom` frame — and the local costmap needs exactly
that continuous, non-jumping frame, which a loop-closing `map` pose is not.

**(c) Hybrid: keep `map → odom`, republish `odom → base_footprint` ourselves.**
Wrapper 4.2 does not expose the odom TF separately (`publish_tf` covers the
whole odom chain, `publish_map_tf` the map chain), so this means disabling
`publish_tf` and re-publishing both — most of (b)'s cost for less of its clarity.

### Recommendation

**Option (a) — a single static transform publisher, and add
`robot_state_publisher` only for the links the ZED does not own.**

- Add to `localization.launch.py` (or a new `autonomy.launch.py`):
  `tf2_ros static_transform_publisher --x -0.345 --y 0 --z -0.548 --frame-id zed_front_camera_link --child-frame-id base_footprint`
  with the numbers read from the **same constant** `localization_status` uses,
  so a wrong mount offset stays "one number in one place" as SP1 requires.
- **Do not add a full `robot_state_publisher`** on the Orin publishing the URDF
  root — it would re-parent `zed_front_camera_link` and break the tree, exactly
  as SP1 warns. If Nav2 later needs `base_link` too, publish
  `base_footprint → base_link` as a second static transform (`z = +0.409`),
  which adds children rather than parents and is safe.
- Keep `/localization/pose` exactly as it is. It stays the ground station's and
  the simulation's interface; Nav2 never reads it.
- Nav2 also wants an **`odom_topic`** for the controller server and velocity
  smoother. `/localization/pose` is in `map`, which is the wrong frame for that
  role. Publish a second topic **`/localization/odom_local`**
  (`nav_msgs/Odometry`, `frame_id: odom`, `child_frame_id: base_footprint`) from
  the wrapper's `/zed_front/zed_node/odom`, re-expressed through the same mount
  offset. This is a ~30-line addition to the existing `localization_status`
  node and is the honest answer to "which odometry does Nav2 use" when there is
  no wheel odometry.

### The rest of the Nav2 architecture

- **`bt_navigator` with both navigators.** `NavigateToPose` for a single goal
  (`nav2_msgs/action/NavigateToPose`), `NavigateThroughPoses`
  (`nav2_msgs/action/NavigateThroughPoses`) for an ERC checkpoint list. The
  latter is the one that matters — the task *is* a list of checkpoints, and
  `NavigateThroughPoses` replans across the whole remaining list rather than
  stopping at each one.
  Source: [BT navigator config](https://ros.ncnynl.com/en/nav2/configuration/packages/configuring-bt-navigator.html).
- **`nav2_lifecycle_manager`** with `autostart: false`. The rover must come up
  in **manual** mode with autonomy configured-but-not-active; activation is an
  explicit operator act. This also gives the mode supervisor a clean way to
  kill autonomy: deactivate the lifecycle nodes.
- **Custom BT XML** rather than the default: the default recovery set
  (`Spin`, `BackUp`, `Wait`, `ClearCostmap`) includes a `BackUp` that we want
  short, and it lacks any check on `/localization/status`. A
  `navi_autonomy/behavior_trees/navigate_asterope.xml` should wrap the
  navigation subtree in a condition on a blackboard flag that the mode
  supervisor sets from the localisation watchdog.
- **Rates.** Costmap `update_frequency` 1 Hz global / 5 Hz local; controller
  16.667 Hz (= 1/TS); `transform_tolerance` 0.3 s everywhere (the pose is 30 Hz
  but arrives over a shared link, and `SEARCHING` freezes stamps — a tight
  tolerance would produce extrapolation errors that read as TF failures).
- **The frozen-stamp behaviour is a booby trap.** SP1 freezes
  `/localization/pose`'s stamp while `SEARCHING`. If the TF chain is fed from
  the wrapper directly, TF does *not* freeze, so Nav2 would keep navigating on
  a diverging pose. This is exactly why the localisation watchdog (section 5) is
  mandatory and not a nicety.

```yaml
bt_navigator:
  ros__parameters:
    global_frame: map ; robot_base_frame: base_footprint
    odom_topic: /localization/odom_local
    transform_tolerance: 0.3 ; bt_loop_duration: 10 ; default_server_timeout: 20
    navigators: ["navigate_to_pose", "navigate_through_poses"]
    navigate_to_pose:       {plugin: "nav2_bt_navigator::NavigateToPoseNavigator"}
    navigate_through_poses: {plugin: "nav2_bt_navigator::NavigateThroughPosesNavigator"}
    default_nav_to_pose_bt_xml: $(find-pkg-share navi_autonomy)/behavior_trees/navigate_asterope.xml
    default_nav_through_poses_bt_xml: $(find-pkg-share navi_autonomy)/behavior_trees/navigate_asterope_through.xml

behavior_server:
  ros__parameters:
    local_frame: odom ; global_frame: map ; robot_base_frame: base_footprint
    behavior_plugins: ["spin", "backup", "wait"]
    # NOTE: 'drive_on_heading' deliberately omitted - no rear/side sensing
    max_rotational_vel: 0.45 ; min_rotational_vel: 0.10 ; rotational_acc_lim: 0.5

lifecycle_manager_navigation:
  ros__parameters:
    autostart: false          # manual mode is the power-on state
    node_names: ["controller_server", "smoother_server", "planner_server",
                 "behavior_server", "bt_navigator", "velocity_smoother",
                 "collision_monitor"]
```

---

## 5. Command arbitration and safety

### The requirement

The strict hierarchy is **autonomous → semi-autonomous → manual**, and the
operator must win *instantly*, at any time. Today `/manual_twist` goes straight
from rosbridge to the chassis with nothing in between; adding Nav2 means two
publishers for one physical actuator, which is unacceptable as-is.

### Options

- **`twist_mux`** (`ros-teleop/twist_mux`): N `Twist` inputs with priorities and
  timeouts, plus `std_msgs/Bool` **locks** that disable lower-priority inputs.
  Not installed; `ros-humble-twist-mux` exists.
  Source: [twist_mux](https://github.com/ros-teleop/twist_mux).
- **A custom `mode_supervisor` node** — more code, but it can do what
  `twist_mux` cannot: hold the mode as explicit state, watch
  `/localization/status`, call Nav2's `CancelGoal`, drive the lifecycle manager,
  and publish one JSON status the GS already knows how to read.
- **`nav2_collision_monitor`** as the last gate — orthogonal and complementary
  (it filters on *sensor* data, not on *mode*).

### Trade-offs

`twist_mux`'s priority-plus-timeout model gets "the operator wins" right and
"and Nav2 must actually stop planning" wrong. If the operator grabs the stick
while Nav2 keeps running its BT, the moment they let go the mux falls back to
Nav2's `cmd_vel` and the rover lurches back onto the path the operator was
deliberately deviating from — a genuinely dangerous failure at a competition.

### Recommendation

**Both, layered — `twist_mux` for the millisecond-scale arbitration,
a `mode_supervisor` for the state machine.**

```
/manual_twist        (prio 200, timeout 0.5) ─┐
/autonomy_twist      (prio 100, timeout 0.3) ─┼─ twist_mux ─► /rover_twist ─► chassis
/estop_twist         (prio 255, timeout 0.0) ─┘      ▲
                                            locks:  │
                                              /autonomy_lock  (Bool, prio 254)
                                              /estop_lock     (Bool, prio 255)
```

**Critical rename.** The chassis today listens on `/manual_twist`. It must be
changed to listen on **`/rover_twist`**, with `/manual_twist` becoming the
*operator input to the mux*. This is a one-line change on the rover side and a
zero-line change on the ground station side, and it is the change that makes
the whole hierarchy possible. Note `sim_ik_node` and `sim_bridge` also
subscribe `/manual_twist` — `sim.launch.py` already takes a `twist_topic`
argument, so the simulation is already parameterised for this.

**`mode_supervisor` (Python, `navi_autonomy`)** — one node, four jobs:

1. **Owns the mode.** Subscribes `/mode_request` (`std_msgs/String` JSON, the
   `/video_request` convention: `{"mode":"manual"|"semi_auto"|"autonomous","reason":"..."}`)
   and publishes `/mode_status` (`std_msgs/String` JSON:
   `{"mode":..., "autonomy":"IDLE"|"NAVIGATING"|"PAUSED"|"FAULT", "goal_id":..., "detail":...}`)
   at 2 Hz and on change.
2. **Operator override is a hardware-grade path, not a state transition.**
   Any `/manual_twist` message with |v| above the deadband while in
   `autonomous` immediately: raises `/autonomy_lock`, calls Nav2's
   `NavigateThroughPoses` **cancel**, and sets mode to `manual`. The twist_mux
   priority means the operator's twist is already reaching the wheels before
   any of that completes — the cancel is cleanup, not the mechanism. **This
   ordering is the safety argument.**
3. **Localisation watchdog.** Subscribes `/localization/status`. On
   `SEARCHING` for more than `hold_seconds` (start at 1.0) or on `OFF`
   immediately: raise `/autonomy_lock`, publish one zero twist on
   `/estop_twist`, cancel the goal, mode → `manual`, `autonomy: FAULT`. Also
   watch the *age* of `/localization/status` itself — a dead
   `localization_status` node is indistinguishable from a healthy one if you
   only look at the last message's contents.
4. **Drives the lifecycle manager.** Entering `autonomous` activates the Nav2
   nodes; leaving it deactivates them. Deactivated Nav2 publishes nothing, which
   makes the "no second publisher" property structural rather than a matter of
   priorities.

**Estop.** `/estop_request` (`std_msgs/String` JSON `{"stop":true}`) from the
ground station, and a gamepad button bound to it locally. It sets
`/estop_lock` true (which locks *both* other inputs at priority 255) and
publishes zeros on `/estop_twist` at 20 Hz until cleared. Clearing requires an
explicit `{"stop":false}` — never a timeout.

**Velocity limits for the ERC yard.** Start at `vx_max 0.45 m/s`,
`wz_max 0.60 rad/s`, `vy = 0`. Rationale: at 0.45 m/s the 6 m local costmap is
13 s deep, the collision monitor's 1.10 m stop polygon gives ~2.4 s of reaction
at full speed, and the ZED's honest depth range (6 m used of 10 m nominal) is
13 s ahead. Raise only after a full yard run with no collision-monitor triggers.

**How Nav2 pauses and cancels.** `NavigateToPose` / `NavigateThroughPoses` are
`rclcpp_action` servers; a cancel is `action_msgs/srv/CancelGoal` on
`<action_name>/_action/cancel_goal`. The BT navigator halts the tree and the
controller server publishes a zero `cmd_vel` on halt. **Nav2 has no "pause"** —
pause is implemented as cancel-and-remember-the-goal-list, which the
`mode_supervisor` should do explicitly (store the remaining poses, re-send them
on resume). Do not invent a pause inside Nav2.

```yaml
twist_mux:
  ros__parameters:
    topics:
      estop:    {topic: estop_twist,    timeout: 0.0, priority: 255}
      manual:   {topic: manual_twist,   timeout: 0.5, priority: 200}
      autonomy: {topic: autonomy_twist, timeout: 0.3, priority: 100}
    locks:
      estop_lock:    {topic: estop_lock,    timeout: 0.0, priority: 255}
      autonomy_lock: {topic: autonomy_lock, timeout: 0.0, priority: 254}
```

---

## 6. Ground station integration over rosbridge

### Options for sending a goal

- **`roslibpy.ActionClient`** (ROS 2 flavour): `ActionClient(ros, name, action_type)`,
  `send_goal(goal, resultback, feedback, errback)`, `cancel_goal(goal_id)`. It
  exists and works — **but the docs state plainly: "Async cancelation is not yet
  supported on rosbridge (rosbridge_suite issue #909)."**
  Source: [roslibpy API reference](https://roslibpy.readthedocs.io/en/latest/reference/index.html).
- **A `goal_relay` node on the Orin** taking JSON in a `std_msgs/String` — the
  `/video_request` pattern this repo already uses twice, for the reason
  `video_request.py`'s docstring records (a custom `.msg` would force an
  `ament_cmake` package and rosbridge type discovery).

### Trade-offs for this project

The cancel limitation is decisive: **cancel is the operator's safety
interface**, and a design where "go" is reliable while "stop" depends on an
upstream rosbridge issue is the wrong way round. Secondly,
`NavigateThroughPoses` goals carry `geometry_msgs/PoseStamped[]` with
quaternions and stamps; hand-assembling those in the GS means it learns ROS
message layouts, which the architecture has so far avoided (`ground_station/`
may not import `rclpy`, and its client speaks only `Twist` and `String`).

### Recommendation

**A `goal_relay` node on the Orin, JSON over `std_msgs/String`, matching
`video_request.py`'s validation discipline.** The ground station stays a
rosbridge client that knows two message types.

```
GS -> /nav_request   std_msgs/String, JSON:
  {"action": "goto",
   "waypoints": [{"x": 12.4, "y": -3.1, "yaw": null, "label": "CP1"}, ...],
   "frame": "map",
   "max_speed": 0.45,
   "request_id": "a3f1"}
  {"action": "cancel", "request_id": "a3f1"}
  {"action": "pause"}    /  {"action": "resume"}

Orin -> /nav_status  std_msgs/String, JSON, 2 Hz + on change:
  {"request_id": "a3f1",
   "state": "IDLE"|"PLANNING"|"NAVIGATING"|"PAUSED"|"SUCCEEDED"|"FAILED"|"CANCELLED",
   "current_waypoint": 1, "total_waypoints": 5,
   "distance_remaining": 18.3, "eta_seconds": 52.0,
   "detail": "..."}
```
`goal_relay` validates exactly as `video_request.parse_request` does — every
field checked for presence, type and range, nothing downstream assuming
anything — converts to `NavigateThroughPoses`, and is a normal `rclcpp_action`
client, so cancel is a first-class local call with no rosbridge involvement.
`yaw: null` means "any heading at this waypoint", which matches ERC scoring and
lets the goal checker use `yaw_goal_tolerance: 6.28`.

### Showing the plan

Nav2 publishes `nav_msgs/Path` on `/plan` (global) and `/local_plan`. Two
consumers:

- **In the Gazebo view (SP2's `sim_bridge`).** Add `/plan` and
  `/nav_status` to the domain-0→sim-domain forwarding list, then a small node
  in `navi_sim_bringup` that renders the path as a Gazebo `visual` — a thin
  ribbon of boxes, or (simpler and more robust) publishes a
  `visualization_msgs/MarkerArray` and lets an RViz panel show it. The path is
  40 m at 0.10 m ≈ 400 poses; downsample to every 5th pose before sending it
  over the bridge.
- **In the ground station.** The GS has no ROS, so the path must arrive as
  something roslibpy can carry. Two options: subscribe `nav_msgs/Path` directly
  over rosbridge (roslibpy handles it; ~400 poses at 1 Hz is ~50 KB/s, fine on
  a wired link, questionable on WiFi), or have `goal_relay` publish a
  decimated `/nav_path_summary` (`std_msgs/String` JSON, every 5th pose, 1 Hz).
  **Recommend the decimated JSON**, consistent with everything else and
  bandwidth-honest.

### Autonomous mode's UI needs

`dashboard_page.py` currently has two radios (`Manual`, `Semi-autonomous`); SP2
adds `Simulation` and a disabled `Autonomous`. Enabling `Autonomous` needs:

1. **A waypoint list widget** — add/remove/reorder `(x, y, label)` rows, load
   from a CSV the organisers hand out, plus a "pick on the map" affordance in
   the Gazebo/plan view if that view becomes clickable.
2. **A prominent STOP** that is always enabled in every mode and publishes
   `/estop_request`. Bind it to a gamepad button too — a mouse is not an
   emergency interface.
3. **Progress**: current waypoint N of M, distance remaining, ETA, and the
   `nav_status.state` word, from `/nav_status`.
4. **The localisation marker already specified in SP2**, plus a rule that
   `Autonomous` cannot be *selected* while `/localization/status` ≠ `OK`, and
   is *left automatically* when it stops being `OK` (the supervisor already
   forces this; the UI must reflect it rather than fight it).
5. **A takeover indicator** — when the operator moves the stick in
   `autonomous`, the UI must say, immediately and unmistakably, that autonomy
   was cancelled by the takeover. A silent takeover is how operators end up
   surprised when they let go.

---

## 7. Testing without the rover

### What already exists to build on

- `start_sim.sh` → `sim.launch.py map_mesh:=... twist_topic:=...` — the twist
  topic is **already parameterised**, so pointing the simulated IK at
  `/rover_twist` is a launch argument, not a code change.
- `sim_ik_node` publishes `/sim_odom` (dead-reckoned from
  `eta_dot_constrained`), `/sim_cmd_vel`, `/set_joint_trajectory`,
  `/sim_ik_debug`.
- `sim_bridge` (SP2) already holds two `rclpy` contexts on two domains and
  forwards a fixed list of topics one way.
- `terrain_writer` (SP3) already turns a grid into a Gazebo heightmap.
- `mock/ros_bridge.py` already fakes rosbridge for GS tests, including a
  `/localization/pose` that walks a square.

### Recommended test ladder

**Rung 1 — traversability, no ROS graph at all.** A pytest/gtest harness that
loads `Model3D_mesh2.obj`, rasterises it to a 0.10 m grid (the same code the
prior-scan seeder uses), runs `traversability_layer`'s pure functions over it,
and asserts specific cells: a known rock is lethal, a known 8° slope is free, an
unobserved region is 255. Milliseconds per run, and it is the layer most likely
to be subtly wrong.

**Rung 2 — Nav2 against a faked costmap, no Gazebo.** `planner_server` +
`bt_navigator` + a `map_server` publishing the rasterised prior scan, a static
`map→odom→base_footprint` TF from a scripted pose, a `NavigateThroughPoses`
goal; assert `/plan` exists, stays outside lethal cells, and reaches within
tolerance. Catches frame, lifecycle and plugin-name errors in seconds, no
physics needed.

**Rung 3 — closed loop in the existing kinematic simulation.** This is the
important one, and the existing sim is *better* for it than a physics sim,
because it runs the **real Simulink IK**. Wiring:

```
        ┌──────────────── laptop, sim domain (42) ──────────────────┐
        │                                                            │
  Nav2 (planner+controller+smoother+collision_monitor)               │
        │ /cmd_vel -> velocity_smoother -> collision_monitor         │
        │                     -> /autonomy_twist -> twist_mux        │
        │                                  -> /rover_twist           │
        │                                        │                   │
        │                                  sim_ik_node               │
        │                                  (twist_topic:=/rover_twist)│
        │                                        │                   │
        │                                  /sim_odom ──────────┐     │
        │                                  Gazebo model pose   │     │
        │                                        │             │     │
        │            pose_republisher: /sim_odom -> /localization/pose (map frame)
        │                             + fake /localization/status = OK │
        │                                        │                   │
        │            ZED substitute: Gazebo depth camera or a         │
        │            replayed recorded cloud -> /autonomy/points_filtered
        └────────────────────────────────────────────────────────────┘
```
Two new small pieces, both trivially testable:
- **`pose_republisher`** — `/sim_odom` → `/localization/pose` + a fake
  `/localization/status`, plus the `map→odom→base_footprint` static TF. This is
  the mirror image of SP2's `sim_ik_node` pose-override path and reuses the
  same frame maths.
- **a depth source.** Either add a Gazebo depth camera to
  `asterope_sim.urdf.xacro` at the ZED's mount pose (best fidelity, costs
  simulation performance), or replay a recorded `/zed_front/.../cloud_registered`
  bag on a loop (cheap, uncorrelated with where the rover actually is —
  adequate for testing that the layer *plumbs*, useless for testing that it
  *avoids*). **Recommend the Gazebo depth camera**, because obstacle avoidance
  is precisely the behaviour the simulation exists to verify.

**Rung 4 — faults.** Scripted: drop `/localization/status` to `SEARCHING`
mid-traverse and assert the rover stops within 1.5 s; inject a `/manual_twist`
mid-traverse and assert the goal is cancelled and the operator's twist reaches
`/rover_twist` within one control period; kill `localization_status` entirely
and assert the same.

### Replayable bags

Record on the Orin during every yard run, with `ros2 bag record` and an
explicit topic list (never `-a`; the ZED's raw image and cloud topics will fill
a disk in minutes):

```
/localization/pose  /localization/status  /localization/map
/localization/odom_local
/tf  /tf_static
/zed_front/zed_node/point_cloud/cloud_registered   # decimated if possible
/manual_twist  /autonomy_twist  /rover_twist  /estop_twist
/plan  /local_plan  /nav_request  /nav_status  /mode_status
/global_costmap/costmap  /local_costmap/costmap
```
Store the small ones (`test_data/`, a few MB) in the repo, as SP3 already does
for the fused cloud. Keep clouds out of git.

**One caution on bag replay:** `use_sim_time` plus `/clock` plus the IK's
0.06 s tick is already a documented trap in `sim_ik_node.cpp` (the `/clock`
period must divide `kTimestepSeconds`). Replay for *costmap and planner*
testing, where timing is loose. Do not replay for *controller* testing — use
the live kinematic simulation, where the clock relationship is already correct.

---

## (a) Proposed architecture

```
 GROUND STATION (laptop, PySide6, roslibpy, NO rclpy)
 ┌──────────────────────────────────────────────────────────────────────────┐
 │ dashboard: MODE [ manual | semi_auto | autonomous | simulation ]         │
 │ waypoint list   STOP (always live)   progress   localisation marker      │
 │ gamepad ──► Twist                                                        │
 └───┬────────────────┬──────────────┬──────────────┬──────────────┬────────┘
     │ /manual_twist  │ /nav_request │ /estop_req   │ /mode_request│  (subs)
     │ Twist          │ String JSON  │ String JSON  │ String JSON  │ /nav_status
     │                │              │              │              │ /mode_status
     │                │              │              │              │ /localization/status
     └────────────────┴──────────────┴──────────────┴──────────────┴─── rosbridge :9090
                                       │
 ═════════════════════════════════════ │ ══════════════════════════════════════
 ORIN (Jetson Orin Nano, ROS 2 Humble) │
                                       ▼
   ZED 2i ──► zed_wrapper ──┬─► /tf : map→odom→zed_front_camera_link
                            │   + static tf: zed_front_camera_link→base_footprint
                            │                (mount offset, ONE constant)
                            ├─► /zed_front/zed_node/odom
                            │        └─► localization_status ─┬─► /localization/pose   (map, 30 Hz)
                            │                                 ├─► /localization/odom_local (odom)
                            │                                 └─► /localization/status (JSON)
                            ├─► point_cloud/cloud_registered
                            │        └─► cloud_filter (voxel 0.05, crop 8 m)
                            │                 └─► /autonomy/points_filtered
                            └─► mapping/fused_cloud
                                     └─► elevation_mapper ─► /localization/map (GridMap)
                                              │
                                     traversability_layer   (slope | step | roughness | valid)
                                              ├─► /autonomy/traversability (GridMap, for the GS)
                                              └─► /autonomy/costmap_seed   (OccupancyGrid, latched)
   prior scan (offline) ─► /autonomy/prior_map (OccupancyGrid, latched)
                                              │
   ┌──────────────────────── NAV2 (lifecycle, autostart: false) ───────────────────────┐
   │  global_costmap [prior_layer | live_layer | obstacle_layer | inflation]  map, 0.10 m│
   │  planner_server  ThetaStarPlanner (alt: SmacPlanner2D)      ──► /plan               │
   │  bt_navigator    NavigateToPose | NavigateThroughPoses                              │
   │  local_costmap  [voxel_layer | inflation]                   odom, rolling 8x8 m     │
   │  controller_server  RotationShim( RegulatedPurePursuit )  16.667 Hz  ──► /cmd_vel   │
   │  velocity_smoother   vy pinned 0, wz accel 0.5             ──► /cmd_vel_smoothed    │
   │  collision_monitor   PolygonStop/PolygonSlow (forward only)──► /autonomy_twist      │
   └────────────────────────────────────────────────────────────────────────────────────┘
                │                                     ▲
                │ goal / cancel (rclcpp_action)       │ lifecycle activate/deactivate
                │                                     │
        goal_relay  ◄── /nav_request        mode_supervisor ◄── /mode_request, /estop_request
             └──► /nav_status                    │      └──► /mode_status
                                                 │      └──► /autonomy_lock, /estop_lock, /estop_twist
                                                 └── watches /localization/status
                                       │
   /estop_twist (255) ┐                │
   /manual_twist (200)├──► twist_mux ──┴──► /rover_twist ──► twist_shaper (kinematics oracle)
   /autonomy_twist(100)┘                                            └──► chassis (bemacontroller)
                                                                    └──► /ik_feasibility (JSON)
```

**Simulation view** (laptop, sim domain 42) receives `/localization/pose`,
`/localization/status`, `/localization/map`, `/rover_twist`, `/plan`,
`/nav_status` through `sim_bridge`, one way.

---

## (b) Open decisions, with a recommended answer for each

| # | Decision | Recommendation | Why |
|---|---|---|---|
| 1 | Where does `base_footprint` enter TF? | Static transform `zed_front_camera_link → base_footprint` at `(−0.345, 0, −0.548)`, from the same constant `localization_status` uses. No `robot_state_publisher` on the Orin. | Keeps the ZED as sole TF owner (SP1's rule), one number in one place, zero new failure modes. |
| 2 | Global planner | `ThetaStarPlanner`, with `SmacPlanner2D` loaded as a second named plugin. | Any-angle → few segments → few ICR changes. Second plugin costs nothing and makes A/B a parameter change. |
| 3 | Path follower | `RotationShimController` wrapping `RegulatedPurePursuitController`. MPPI configured but parked with `vy_max: 0.0`. | Smooth curvature suits the ICR controller; the shim gives point-turns the chassis does natively; MPPI is the upgrade path, not the starting point. |
| 4 | Holonomic or not? | **Non-holonomic. `vy` pinned to 0 at the velocity smoother.** | Steering slew makes `vy` steps a latency the controllers cannot model. Revisit only after measuring the real slew rate. |
| 5 | Traversability: reuse or write? | Write `traversability_layer` ourselves; copy the *maths* from `traversability_estimation`. | That package is ROS 1 only. The filters are ~200 lines and we control the thresholds. |
| 6 | grid_map → costmap route | `GridMapRosConverter::toOccupancyGrid` + two `StaticLayer`s (prior, then live). | Uses installed `grid_map_ros`; no custom costmap plugin; "live beats prior" falls out of StaticLayer's overwrite semantics. |
| 7 | Chassis input topic | Rename to `/rover_twist`; `/manual_twist` becomes the mux's operator input. | Without this there is no arbitration point. GS unchanged; `sim.launch.py` already has `twist_topic`. |
| 8 | Arbitration mechanism | `twist_mux` (priority + lock) **plus** a `mode_supervisor` state machine. | Mux alone lets the rover resume autonomy when the operator lets go — dangerous. Supervisor alone is too slow to be the wire-level guarantee. |
| 9 | Goal transport over rosbridge | JSON `std_msgs/String` on `/nav_request` + on-rover `goal_relay`. Not `roslibpy.ActionClient`. | roslibpy documents that async cancel is unsupported on rosbridge; cancel is the safety path and must not be the flaky one. Also keeps GS free of ROS types. |
| 10 | `NavigateToPose` or `NavigateThroughPoses`? | `NavigateThroughPoses` as the primary; `NavigateToPose` kept for single-goal debugging. | ERC is a checkpoint list; through-poses replans across the whole remainder rather than stopping at each. |
| 11 | Reversing | `allow_reversing: false`; `BackUp` recovery capped at 0.6 m; `vx_min ≥ −0.15`. | No rear sensing, and the costmap's rear memory rests on a pose with no wheel-odometry fallback. |
| 12 | Depth source in simulation | Add a Gazebo depth camera at the ZED mount pose. | Replayed clouds test plumbing, not avoidance. Avoidance is what the sim is for. |
| 13 | Costmap resolution | 0.10 m, equal to `/localization/map`. | Resampling smears step edges — the one feature we cannot afford to lose. |
| 14 | Nav2 autostart | `autostart: false`; the supervisor activates on entering `autonomous`. | Power-on state is manual. Deactivated Nav2 publishes nothing, making "one publisher" structural. |
| 15 | New package layout | One new `navi_autonomy` (ament_python, or ament_cmake if `twist_shaper` reuses `SimIkStepper` in C++) alongside `navi_localization` and `navi_teleop`. | Matches the existing `rover/src/navi_*` convention. `twist_shaper` in C++ argues for ament_cmake. |

---

## (c) Top 8 risks

| # | Risk | Why it bites here | Mitigation |
|---|---|---|---|
| 1 | **VIO loses tracking mid-traverse and Nav2 keeps driving.** | SP1 freezes `/localization/pose`'s stamp on `SEARCHING`, but the ZED's *TF* does not freeze — Nav2 would navigate on a diverging pose without noticing. | The `mode_supervisor` watchdog is mandatory, not optional: `SEARCHING` > 1.0 s or `OFF` → estop + cancel + mode `manual`. Also watch the *age* of `/localization/status`. Test as a scripted fault (rung 4). |
| 2 | **Steering lag makes the controller oscillate.** | RPP and MPPI both assume the commanded twist is achieved this tick. Four modules slewing across a curvature change means a dead time the controller reads as "no response" and amplifies. | Non-holonomic operation; low `wz` accel in the smoother; `scale_velocities: true`; the `twist_shaper` oracle. Measure the real slew rate on the first rover day and set `max_accel[2]` from it, not from a guess. |
| 3 | **Nothing sees behind or beside the rover.** | ZED HFOV ~110°, forward only. A rock passed at 1 m to the side is unknown the moment it leaves the frame, and any rotation-in-place or reverse is on memory alone. | Forward-only collision polygons; `allow_reversing: false`; short `BackUp`; large `inflation_radius` (1.20 m global) so paths do not graze obstacles in the first place; `Spin` recovery collision-checked via the shim's `simulate_ahead_time`. |
| 4 | **Two publishers on the chassis topic during the transition.** | Today `/manual_twist` goes straight to the chassis. The day Nav2 first runs, both will publish unless the rename lands first. | Land the `/rover_twist` rename + `twist_mux` as its **own sub-project, before any Nav2 code exists**, and verify with `ros2 topic info /rover_twist` that there is exactly one publisher. |
| 5 | **Elevation map arrives at 0.5 Hz and only on change.** | A rover at 0.45 m/s covers 0.9 m between map updates. If the global costmap's only obstacle evidence is the elevation map, it plans into things it has already seen but not yet published. | The `ObstacleLayer` on the *global* costmap from the live filtered cloud (not just the local one) gives sub-second obstacle evidence. The elevation map is for *terrain*; the cloud is for *obstacles*. Do not conflate them. |
| 6 | **Orin CPU budget.** | ZED SDK tracking + spatial mapping already use the GPU and a good deal of CPU. Adding voxel layer + a 1000-batch MPPI + collision monitor on 24 K points could push past real time. | Start with RPP (cheap) not MPPI; downsample the cloud *before* it reaches the layer and the monitor; `publish_voxel_map: false` and `visualize: false` once tuned; measure with `ros2 topic hz /cmd_vel` and the `sim_ik_node`-style tick-rate warning pattern. Budget the measurement into the first sub-project. |
| 7 | **The prior scan and the live map disagree in frame or origin.** | The ZED's `map` frame origin is wherever the rover booted, with gravity-aligned Z. The organisers' scan is in its own frame. Overlaying them wrongly puts lethal cells 3 m from where the rocks are — and it will look plausible. | Do not seed the prior at all until an explicit `map`-frame alignment exists (ArUco-anchored or a manual 2-D alignment the operator confirms in the GS). Make the prior layer *opt-in per run*, defaulting off. This is the single most plausible way to fail confidently. |
| 8 | **Operator takeover leaves autonomy running.** | With `twist_mux` alone, releasing the stick hands control straight back to a Nav2 that never stopped. | Takeover raises `/autonomy_lock` *and* cancels the goal *and* changes mode, in that order; the mux priority is already carrying the operator's twist before any of it completes. The GS must show the takeover unmistakably. Scripted test (rung 4). |

---

## (d) Proposed sub-projects

Numbering continues from the localisation work (SP1 rover localisation, SP2
ground station and view, SP3 built map). Each is meant to be independently
useful and independently testable, in the style of the existing designs.

**SP4 — Command arbitration.** *Goal:* exactly one publisher reaches the
chassis, and the operator always wins.
Rename the chassis input to `/rover_twist`; add `twist_mux` with the three
inputs and two locks; add `mode_supervisor` (mode state, `/mode_request` /
`/mode_status`, estop, localisation watchdog); GS gains a live STOP button and
a mode indicator that reflects forced transitions.
*Depends on:* SP1 (needs `/localization/status`). *Independent of* SP2, SP3.
*Ships value alone:* the rover gets an estop and a watchdog it does not have
today, with no autonomy anywhere near it.

**SP5 — Frames and odometry for Nav2.** *Goal:* a TF tree Nav2 can navigate in.
Static `zed_front_camera_link → base_footprint` (and `base_footprint → base_link`);
`/localization/odom_local` added to `localization_status`; a `frames` check in
`start_navi.sh` that fails loudly if the tree has two roots or a missing link.
*Depends on:* SP1. *Blocks:* SP7, SP8.
*Ships value alone:* `tf2_echo map base_footprint` works, which is independently
useful for debugging localisation.

**SP6 — Traversability and the costmap seed.** *Goal:* the elevation map becomes
a cost.
`traversability_layer` node (slope / step / roughness / valid), publishing
`/autonomy/traversability` (GridMap) and `/autonomy/costmap_seed`
(OccupancyGrid, latched); the offline scan rasteriser sharing code with
`terrain_writer`; rung-1 tests against `Model3D_mesh2.obj`.
*Depends on:* SP3 (needs `/localization/map`). *Blocks:* SP7.
*Ships value alone:* the GS/Gazebo view can colour the terrain by traversability
long before anything drives on it.

**SP7 — Nav2 planning, offline.** *Goal:* a plan exists and is correct, with
nothing moving.
`navi_autonomy` package; costmap configuration (global with prior + live +
obstacle + inflation, local with voxel); `planner_server` with Theta*;
`bt_navigator` with the custom BT XML; `lifecycle_manager` with
`autostart: false`; the cloud downsampler; rung-2 tests.
*Depends on:* SP5, SP6. *Blocks:* SP8.
*Ships value alone:* the operator can ask "what route would you take?" and see
`/plan` in the view — genuinely useful in semi-autonomous mode.

**SP8 — Following the plan.** *Goal:* the rover drives the plan in simulation.
`controller_server` (RotationShim + RPP), `velocity_smoother`,
`collision_monitor`, `twist_shaper` (reusing `SimIkStepper` as the feasibility
oracle) → `/autonomy_twist` into SP4's mux; simulation wiring
(`pose_republisher`, Gazebo depth camera, `twist_topic:=/rover_twist`);
rung-3 and rung-4 tests.
*Depends on:* SP4, SP7, and SP2 for the view.
*Ships value alone:* a full closed loop on the laptop, no rover required.

**SP9 — Goals and progress from the ground station.** *Goal:* the operator
commands and watches an autonomous traverse.
`goal_relay` (JSON `/nav_request` → `NavigateThroughPoses`, `/nav_status` out,
`video_request.py`-grade validation); GS waypoint list, progress panel,
takeover indicator, `Autonomous` mode enabled and gated on
`/localization/status`; decimated `/nav_path_summary`; `/plan` forwarded to the
sim view through `sim_bridge`.
*Depends on:* SP4 (mode), SP7 (the action servers), SP2 (the view).
*Ships value alone:* this is the mode the competition is scored on.

**SP10 — Yard tuning and measurement.** *Goal:* the numbers in this brief are
replaced by measured ones.
Measure the steering slew rate and set `max_accel[2]`; measure Orin CPU under
the full stack and decide RPP vs MPPI; tune the traversability thresholds
against real rocks; decide whether the prior scan can be aligned safely enough
to enable; record the bag set.
*Depends on:* SP8, SP9, and rover access. *Blocks:* nothing — it is the
never-finished one.

**Dependency graph:**
```
SP1 ──┬─► SP4 ──────────────┐
      └─► SP5 ──┐           │
SP3 ──► SP6 ──┬─┴─► SP7 ────┴─► SP8 ──┐
SP2 ──────────┴──────────────────────┴─► SP9 ──► SP10
```

---

## Sources

- Nav2 Costmap 2D / plugin parameters — https://ros.ncnynl.com/en/nav2/configuration/packages/costmap-plugins/obstacle.html , https://ros.ncnynl.com/en/nav2/configuration/packages/costmap-plugins/voxel.html
- Nav2 Smac Planner (README + Hybrid config) — https://docs.ros.org/en/ros2_packages/humble/api/nav2_smac_planner/ , https://ros.ncnynl.com/en/nav2/configuration/packages/smac/configuring-smac-hybrid.html
- Nav2 MPPI Controller — https://ros.ncnynl.com/en/nav2/configuration/packages/configuring-mppic.html , https://docs.ros.org/en/ros2_packages/humble/api/nav2_mppi_controller/
- Nav2 Regulated Pure Pursuit — https://ros.ncnynl.com/en/nav2/configuration/packages/configuring-regulated-pp.html , https://docs.ros.org/en/ros2_packages/humble/api/nav2_regulated_pure_pursuit_controller/
- Nav2 Rotation Shim Controller — https://docs.ros.org/en/ros2_packages/humble/api/nav2_rotation_shim_controller/
- Nav2 Collision Monitor — https://docs.ros.org/en/ros2_packages/humble/api/nav2_collision_monitor/ , and the installed `/opt/ros/humble/share/nav2_collision_monitor/params/collision_monitor_params.yaml`
- Nav2 Velocity Smoother — https://ros.ncnynl.com/en/nav2/configuration/packages/configuring-velocity-smoother.html
- Nav2 BT Navigator — https://ros.ncnynl.com/en/nav2/configuration/packages/configuring-bt-navigator.html
- Nav2 transforms / REP-105 — https://ros.ncnynl.com/en/nav2/setup_guides/transformation/setup_transforms.html , https://www.ros.org/reps/rep-0105.html
- grid_map and grid_map_costmap_2d — https://github.com/ANYbotics/grid_map , https://index.ros.org/p/grid_map_costmap_2d/
- Elevation mapping and traversability filters — https://github.com/ANYbotics/elevation_mapping , https://github.com/leggedrobotics/traversability_estimation
- twist_mux — https://github.com/ros-teleop/twist_mux
- roslibpy (ROS 2 ActionClient; cancel limitation) — https://roslibpy.readthedocs.io/en/latest/reference/index.html
- ERC / URC context — https://roverchallenge.eu/robotics-competition/ , https://www.scribd.com/document/933955225/University-Rover-Challenge-Rules-2026
- In-repo: `docs/superpowers/specs/2026-08-29-localisation-design.md`,
  `docs/superpowers/specs/2026-08-28-rover-simulation-design.md`,
  `sim/src/navi_sim_ik/**`, `sim/build/navi_sim_bringup/asterope_base.urdf`,
  `ground_station/ros_client.py`, `ground_station/ui/main_window.py`,
  `ground_station/ui/dashboard_page.py`, `rover/src/navi_teleop/navi_teleop/video_request.py`,
  `rover/start_navi.sh`, `start_sim.sh`
