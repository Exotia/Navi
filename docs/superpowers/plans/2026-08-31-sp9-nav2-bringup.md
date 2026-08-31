# SP9: Nav2 Bringup and Offline Planning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Work on the `autonomy` branch in `/home/ole/star/Navi/.worktrees/autonomy`. One commit per task, never push.

**Goal:** Nav2 comes up on the rover as one launch file of six lifecycle nodes, plans over the traversability seed SP7 publishes, and writes its velocity to `/autonomy_twist` and nowhere else. The proof is rung 3 of the testing ladder, run headless on the laptop: with the fixture elevation window latched on `/autonomy/costmap_seed`, Nav2 plans from `(0, 0)` to `(12, 0)` around a 0.5 m pit and asserts that no point of the path — densified to the costmap resolution — comes within the rover's inscribed radius of a lethal cell.

**Architecture:** One new ament_python package `rover/src/navi_nav2` holding data and one bench node: the parameter file, two behaviour-tree XMLs, the bringup launch file, a deterministic fixture generator (pure numpy, built out of SP7's own modules) and `fixture_seed_publisher`, which latches that fixture on `/autonomy/costmap_seed` and — only when explicitly asked — fakes the frames and odometry the ZED would otherwise own. Nav2's six servers are `planner_server` (Theta\* plus SmacPlanner2D for A/B), `controller_server` (RotationShim wrapping Regulated Pure Pursuit), `behavior_server`, `bt_navigator`, `velocity_smoother` and `collision_monitor`, sequenced by one `nav2_lifecycle_manager`. The velocity chain is `controller_server → cmd_vel_nav → velocity_smoother → cmd_vel_smoothed → collision_monitor → /autonomy_twist`; `mode_supervisor` (SP5) is the only thing downstream of that, and the only publisher of `/rover_twist`. In `navi_supervisor`, `RosNav2Control` replaces `NullNav2Control`: it cancels every `NavigateToPose` goal through the action's own `cancel_goal` service and deactivates the stack through the lifecycle manager's `manage_nodes` service, both `call_async`, never waiting.

**Tech Stack:** ROS 2 Humble (`navigation2` 1.1.20, identical version on laptop and Orin), Python 3.10, numpy, `nav2_common.launch.RewrittenYaml`, colcon (ament_python), pytest.

**Spec:** `docs/superpowers/specs/autonomy-plan.md` — §5 (planner, controller, local sensing, collision monitor blocks), §6 (frames), §8 (SP9 row: depends on SP6, SP7), §9 rung 3, §10 (speed caps), §11 risks 4/6/7, §13 (operator actions).

---

## What is already on the `autonomy` branch (read, not assumed)

| Thing | Where | What SP9 uses |
|---|---|---|
| `/autonomy/costmap_seed` cost curve | `rover/src/navi_autonomy/navi_autonomy/traversability.py` | `LETHAL = 100`, `UNKNOWN = -1`, `MAX_SCALED_COST = 99`, `STEP_LETHAL_M = 0.14`, `SLOPE_LETHAL_DEG = 25.0`, `ROUGHNESS_REF_M = 0.05`, `seed_from_elevation(elevation, resolution) -> (layers, int8 cost)` |
| Window geometry | `.../navi_autonomy/window.py` | `WINDOW_M = 48.0`, `WINDOW_CELLS = 960`, `RollingWindow` |
| OccupancyGrid encoding | `.../navi_autonomy/grid_map_io.py` | `build_occupancy_grid(cost, origin_ix, origin_iy, resolution, frame_id, stamp)` — row-major from the corner, `info.origin` is the **corner** of cell (0, 0), x fastest, y ascending |
| Resolution | `rover/src/navi_localization/navi_localization/elevation_grid.py` | `RESOLUTION = 0.05` |
| Odometry for Nav2 | `.../navi_localization/odom_local.py`, published by `localization_status.py` | topic `/localization/odom_local`, `ODOM_FRAME`/`BASE_FRAME` constants (`odom` → `base_footprint`) |
| Static frames | `.../navi_localization/launch/localization.launch.py` | `zed_front_camera_link → base_footprint`, `base_footprint → base_link`; the ZED wrapper owns `map → odom → zed_front_camera_link` |
| The stub SP9 fills | `rover/src/navi_supervisor/navi_supervisor/nav2_control.py` | `class Nav2Control` with exactly `cancel_goal(reason)` and `deactivate(reason)`, both returning `None`. `NullNav2Control.calls` is the ordered `(method, reason)` list the SP5 tests assert on. **Do not change the two names.** |
| Where the supervisor calls them | `.../navi_supervisor/mode_supervisor.py` `_run_actions()` | `rules.CANCEL_GOAL → self._nav2.cancel_goal(reason)`, `rules.DEACTIVATE_NAV2 → self._nav2.deactivate(reason)`; the supervisor has already published a zero twist by then |
| Autonomy input | `.../mode_supervisor.py` line 64 | `self.create_subscription(Twist, "/autonomy_twist", ...)` — this is the only topic Nav2 may write |
| Launcher | `rover/start_navi.sh` | `START_*` flags, `--no-x` parsing, `wait_for_localization`, `BACKGROUND_PIDS` |
| Deploy | `deploy_rover.sh` | rsync `rover/` → `star@a_navi:~/navi/`, then `colcon build --symlink-install`; `--test` runs every `src/*/test` with pytest |

**SP7 has landed since this plan was drafted.** `window.py`, `traversability.py`, `grid_map_io.py` and `tile_aggregator.py` were already committed, and `traversability_layer.py` (the node that actually publishes `/autonomy/costmap_seed`) and `launch/autonomy_perception.launch.py` are now committed too (`6a09156 Launch both autonomy perception nodes…`). **Re-read both files rather than trusting this paragraph** — it has been wrong once. What this changes: Task 3's `perception:=true` include and the bare `ros2 launch navi_nav2 nav2_bringup.launch.py` that Task 7 adds to `start_navi.sh` now work against a real launch file, not a placeholder. SP7 deliberately does not touch `start_navi.sh` (SP7 plan lines 26, 102), so there is no double-launch of the perception nodes. SP9's own tests still depend on none of it — the fixture publisher stands in for the node, and every test here passes `perception:=false`.

---

## Environment facts (verified today, 2026-08-31 — bind the plan to these)

- **Nav2 is already installed on BOTH machines** — and here is the output the claim rests on, so a later reader, or a re-run against a re-flashed Orin, can check it rather than inherit it:

  ```
  $ dpkg -l | grep -c "ros-humble-nav2\|ros-humble-navigation2"            # laptop
  31
  $ ssh star@a_navi 'dpkg -l | grep -c "ros-humble-nav2\|ros-humble-navigation2"'
  31
  $ comm -23 laptop-pkgs.txt orin-pkgs.txt | wc -l   # in laptop, not Orin
  0
  $ comm -13 laptop-pkgs.txt orin-pkgs.txt | wc -l   # in Orin, not laptop
  0
  $ dpkg -l ros-humble-navigation2 | tail -1                               # laptop
  ii  ros-humble-navigation2 1.1.20-1jammy.20260804.223401 amd64  ROS2 Navigation Stack
  $ ssh star@a_navi 'dpkg -l ros-humble-navigation2 | tail -1'
  ii  ros-humble-navigation2 1.1.20-1jammy.20260326.171707 arm64  ROS2 Navigation Stack
  $ ssh star@a_navi 'lsb_release -ds; uname -m; df -h / | tail -1'
  Ubuntu 22.04.5 LTS
  aarch64
  /dev/mmcblk0p1  116G   25G   87G  23% /
  ```

  Same 31 packages, same upstream version 1.1.20 (different build dates, different architectures), empty set difference in both directions. **§13's `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup` is already done.** **This is a snapshot, not a standing guarantee** — Task 7 re-runs Task 1's test on the Orin itself before it launches anything. `ros-humble-grid-map-costmap-2d` is *not* installed and is *not needed*: SP7 delivers the seed as a `nav_msgs/OccupancyGrid`, which a stock `StaticLayer` reads (see Rulings).
- **The Orin has no camera attached and no internet.** Everything in this plan is sensor-independent and can be smoke-tested there today; the live-perception items are written as MUST-DOs for the next camera session (Task 7), not as commit gates.
- **No recorded map or bag exists anywhere in the tree.** `find` over `.worktrees/autonomy`, `rover/`, `sim/`, `tests/` for `*.npz`, `*.bag`, `*.db3`, `metadata.yaml`, `*.pgm`, `*.tif` returns nothing. The fixture is therefore *generated* (see Rulings).
- **Empirically verified on the laptop today** (domain 92, both nodes reached `active`, then killed with `pkill -x`):
  - `velocity_smoother` accepts `max_velocity: [0.05, 0.0, 0.1]` with `min_velocity: [-0.15, 0.0, -0.1]` and `max_accel: [0.5, 0.0, 0.5]` — a zero y band does **not** trip a validation error.
  - `collision_monitor` accepts forward-only polygons plus one `pointcloud` source with `enabled: False`, logs `[points_filtered]: Creating PointCloud`, and activates. `cmd_vel_out_topic: /autonomy_twist` produces `/autonomy_twist` on the graph.
- **Parameter names verified against the installed binaries** (`strings` over the plugin `.so`s), because a mistyped Nav2 parameter is silently ignored:
  - `nav2_theta_star_planner`: `how_many_corners`, `w_euc_cost`, `w_traversal_cost`, `allow_unknown`, `use_final_approach_orientation`. **There is no `w_heuristic_cost` parameter in this build** — do not set it.
  - `nav2_smac_planner` (2D): `tolerance`, `downsample_costmap`, `downsampling_factor`, `allow_unknown`, `max_iterations`, `max_on_approach_iterations`, `max_planning_time`, `cost_travel_multiplier`, and `use_final_approach_orientation` (which **does** exist on `SmacPlanner2D` in 1.1.20 — `strings libnav2_smac_planner_2d.so` emits `.use_final_approach_orientation`. This plan leaves it unset on `SmacBased`, which is a choice, not a forced absence).
  - `nav2_rotation_shim_controller`: `primary_controller`, `angular_dist_threshold`, `forward_sampling_distance`, `rotate_to_heading_angular_vel`, `max_angular_accel`, `simulate_ahead_time`, `rotate_to_goal_heading`.
  - `nav2_regulated_pure_pursuit_controller`: `desired_linear_vel`, `lookahead_dist`, `min_lookahead_dist`, `max_lookahead_dist`, `lookahead_time`, `use_velocity_scaled_lookahead_dist`, `rotate_to_heading_angular_vel`, `transform_tolerance`, `min_approach_linear_velocity`, `approach_velocity_scaling_dist`, `use_collision_detection`, `max_allowed_time_to_collision_up_to_carrot`, `use_regulated_linear_velocity_scaling`, `use_cost_regulated_linear_velocity_scaling`, `regulated_linear_scaling_min_radius`, `regulated_linear_scaling_min_speed`, `use_rotate_to_heading`, `rotate_to_heading_min_angle`, `allow_reversing`, `max_angular_accel`, `max_robot_pose_search_dist`, `cost_scaling_dist`, `cost_scaling_gain`, `inflation_cost_scaling_factor`, `use_interpolation`. **No `use_fixed_curvature_lookahead` / `curvature_lookahead_dist` in 1.1.20.**
  - `controller_server`: `controller_frequency`, `odom_topic`, `min_x_velocity_threshold`, `min_y_velocity_threshold`, `min_theta_velocity_threshold`, `failure_tolerance`, `progress_checker_plugin` (**singular**), `goal_checker_plugins`, `controller_plugins`, `speed_limit_topic`.
  - `planner_server`: `expected_planner_frequency`, `planner_plugins`.
  - `bt_navigator`: `odom_topic`, `global_frame`, `robot_base_frame`, `transform_tolerance`, `bt_loop_duration`, `default_server_timeout`, `default_nav_to_pose_bt_xml`, `plugin_lib_names`, `goal_blackboard_id`, `path_blackboard_id`.
  - `behavior_server`: `behavior_plugins`, `cycle_frequency`, `global_frame`, `robot_base_frame`, `transform_tolerance`, `costmap_topic`, `footprint_topic`. **It has no `odom_topic` parameter** — its `OdomSmoother` hard-codes the topic name `odom`, so the launch file must remap `odom → /localization/odom_local` on that node.
  - `collision_monitor`: `base_frame_id`, `odom_frame_id`, `cmd_vel_in_topic`, `cmd_vel_out_topic`, `transform_tolerance`, `source_timeout`, `base_shift_correction`, `stop_pub_timeout`, `polygons`, `observation_sources`.
  - Plugin class names, from the installed plugin XMLs: `nav2_theta_star_planner/ThetaStarPlanner`, `nav2_smac_planner/SmacPlanner2D`, `nav2_rotation_shim_controller::RotationShimController`, `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`.
  - `nav2_bt_navigator`'s stock `navigate_to_pose_w_replanning_and_recovery.xml` and `navigate_through_poses_w_replanning_and_recovery.xml` already contain `<BackUp backup_dist="0.30" backup_speed="0.05"/>` — inside the 0.6 m cap, so the shipped copies need no edit, only a test that pins them.
  - `ros2 param dump` writes a *file* unless given `--print`. Every parameter assertion in this plan uses `ros2 param dump --print <node>` and `yaml.safe_load`.
- **Rover geometry**, from `sim/src/navi_sim_ik/include/navi_sim_ik/asterope_params.hpp` and `asterope_iiI.urdf`: wheel centres at `(±0.45527, ±0.44385)`, chassis box `0.604 × 0.410 × 0.254`, `base_footprint → base_link` at `(0, 0, 0.409)`, wheel radius 0.125 m. Circumscribed radius over the wheel centres is `hypot(0.455, 0.444) = 0.636`; with wheel half-width and mounting hardware the honest circle is **0.80 m**.

---

## Global Constraints

Verbatim from the spec and from the task brief; non-negotiable.

### Topics and writers

- **Nav2's velocity output is remapped to `/autonomy_twist`.** It is produced by `collision_monitor`'s `cmd_vel_out_topic`, and nothing else in the stack publishes it.
- **NOTHING publishes `/rover_twist` except `mode_supervisor`.** No `twist_mux`.
- **NOTHING publishes `/manual_twist`, ever** — not a node, not a test, not a debugging one-liner.
- ROS-graph tests use throwaway `ROS_DOMAIN_ID` **91/92/93**, never domain 0. Task 4's new `test_ros_nav2_control.py` uses **93** — *not* 91, which `test_mode_supervisor.py` already owns, because two agents working this tree at once would otherwise cross-talk — and the supervisor suite command below exports 93 for the whole pytest process, since `rclpy` reads `ROS_DOMAIN_ID` once per process and both files run in the same one. Task 6's offline planning test uses **92**. `pkill -x` only; never `pkill -f` with a pattern that matches your own shell.
- Nav2's `odom_topic` is **`/localization/odom_local`** on `controller_server`, `bt_navigator` and `velocity_smoother`; `behavior_server` gets the same stream by remapping its hard-coded `odom`.

### Exact parameter values (these numbers, not approximations)

| Parameter | Value | Source |
|---|---|---|
| `velocity_smoother.max_velocity` | `[0.05, 0.0, 0.1]` | §10 cap, vy pinned at the smoother (§5) |
| `velocity_smoother.min_velocity` | `[-0.15, 0.0, -0.1]` | §5 `vx_min ≥ −0.15` |
| `velocity_smoother.max_accel` | `[0.5, 0.0, 0.5]` | §10 `wz` accel `≤ 0.5 rad/s²` |
| `velocity_smoother.max_decel` | `[-0.5, 0.0, -0.5]` | same, signed as Nav2 requires |
| `FollowPath.desired_linear_vel` | `0.05` | §10 |
| `FollowPath.rotate_to_heading_angular_vel` | `0.1` | §10 |
| `FollowPath.max_angular_accel` | `0.5` | §10 |
| `FollowPath.allow_reversing` | `false` | §5 "No reversing" |
| `BackUp` in both BT XMLs | `backup_dist="0.30"`, `backup_speed="0.05"` | §5 "BackUp capped 0.6 m" — 0.30 is inside the cap; the test pins `≤ 0.6` and `≤ 0.15` |
| both costmaps `resolution` | `0.05` | §5 "Costmap resolution 0.05 m … resampling smears the step edges" |
| global costmap `width`/`height` | `48` / `48` | §5 48 m window = SP7's `WINDOW_M` |
| both costmaps `robot_radius` | `0.80` | measured geometry above |
| both costmaps `track_unknown_space` | `true` | SP7: "Unseen ground is not driveable ground" |
| `static_layer.map_topic` | `/autonomy/costmap_seed` | §2, SP7 |
| `static_layer.trinary_costmap` | `false` | otherwise the 0–99 scaled band collapses to free/lethal |
| `static_layer.lethal_cost_threshold` | `100` | SP7's `LETHAL` |
| `static_layer.unknown_cost_value` | `-1` | SP7's `UNKNOWN` |
| `static_layer.map_subscribe_transient_local` | `true` | the seed is latched |
| cloud layers (`obstacle_layer`, `voxel_layer`) | `enabled: false` | `cloud_filter` does not exist; no camera |
| collision monitor polygons | forward only, `x ≥ 0.10` for every point | §5, §11 risk 4 |
| collision monitor source | `points_filtered`, `enabled: False` | no cloud yet |
| `GridBased.plugin` | `nav2_theta_star_planner/ThetaStarPlanner` | §5 |
| `SmacBased.plugin` | `nav2_smac_planner/SmacPlanner2D` | §5 "loaded as a second named plugin for A/B" |
| `allow_unknown` (both planners) | `false` | §5 + SP7's unknown policy |

### Process

- **All lifecycle nodes come up via a single launch file**, `navi_nav2/launch/nav2_bringup.launch.py`, with one `nav2_lifecycle_manager` (`autostart: true`). **The deactivation path must work**: `ManageLifecycleNodes.PAUSE` on `/lifecycle_manager_navigation/manage_nodes` takes every node from `active` to `inactive`, and the supervisor calls it on takeover.
- Commits: `git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit`, explicit `git add <paths>`, never `git add -A`, **never push**. On `index.lock`, wait 2 s and retry — other agents work in this tree.
- **Do not touch** anything under `sim/`, `ground_station/`, `rover/src/navi_teleop/`, `rover/src/navi_localization/`, `rover/src/navi_autonomy/`, or any plan file other than this one. SP9 edits exactly two things outside its own package: `rover/src/navi_supervisor/` (Task 4) and `rover/start_navi.sh` (Task 7).
- Every task ends with the whole affected test suite green, not just the new file.

### Commands

Pure tests (no ROS graph; `nav_msgs`/`grid_map_msgs` must be importable):

```
bash -c 'source /opt/ros/humble/setup.bash &&
  PYTHONPATH=$PWD/rover/src/navi_nav2:$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization:$PYTHONPATH \
  python3 -m pytest rover/src/navi_nav2/test -q -p no:cacheprovider'
```

`test_offline_planning.py` needs no `--ignore`: its module-level `skipif` (Task 6) skips the whole file whenever `ROS_DOMAIN_ID` is not `92`, which is also what keeps `deploy_rover.sh --test` from erroring on the Orin. Adding `--ignore=rover/src/navi_nav2/test/test_offline_planning.py` is harmless if you want the file out of the collection report entirely.

Supervisor tests (Task 4, throwaway domain 93 — one domain for the whole process, away from the 91 another agent may be using):

```
bash -c 'source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=93 \
  PYTHONPATH=$PWD/rover/src/navi_supervisor:$PYTHONPATH \
  python3 -m pytest rover/src/navi_supervisor/test -q -p no:cacheprovider'
```

Offline planning, rung 3 (Task 6, throwaway domain 92, needs the workspace built):

```
bash -c 'source /opt/ros/humble/setup.bash && cd rover && colcon build --packages-select navi_nav2 &&
  source install/local_setup.bash && cd .. && ROS_DOMAIN_ID=92 \
  python3 -m pytest rover/src/navi_nav2/test/test_offline_planning.py -q -p no:cacheprovider -s'
```

Build: `bash -c 'source /opt/ros/humble/setup.bash && cd rover && colcon build --packages-up-to navi_nav2'`.

---

## Rulings on spec ambiguities

1. **"A recorded elevation map" (§9 rung 3) does not exist.** There are no bags, no `.npz`, no camera tonight. **Ruling:** the fixture is *generated* deterministically from SP7's own modules — `numpy` elevation window → `traversability.seed_from_elevation` → `grid_map_io.build_occupancy_grid` — and that generator is the committed artefact, not a binary. A 960² `int8` seed is 0.92 MB and its `float32` elevation is 3.7 MB; committing either would put binary churn in a repo whose whole rover side is text. The generator is seeded, so the fixture is the same terrain on every machine, and Task 5's test pins that structurally — the lethal-cell count, the pit's cell index and the elevation extremes of the int8 seed the planner actually consumes, *not* a hash of the float32 elevation, which numpy's SIMD-dispatched trig makes non-bit-stable between amd64 and arm64. When SP12 records a real yard bag, `fixture.py` gains a `from_npz(path)` loader and the same assertions run against it unchanged.
2. **"cloud_filter" (§5) does not exist and no SP built it.** **Ruling:** the `ObstacleLayer` (global) and `VoxelLayer` (local) are written out in full, pointed at `/autonomy/points_filtered`, and shipped **`enabled: false`**. The collision monitor's `points_filtered` source is likewise `enabled: False`. Turning three booleans to `true` is the whole of the camera-session work, and until then the costmaps are the seed plus inflation, exactly as the brief requires.
3. **"measured in SP9" (§11 risk 6) vs "measured on the Orin in SP10" (§5).** **Ruling:** SP9 measures what a camera-less Orin can honestly measure — the CPU and RAM of two 0.05 m costmaps fed by the latched fixture seed, which is the 960² load itself, since the seed is what sizes the global costmap. Task 7 records the numbers. SP10 re-measures with live perception on top. The documented fallback (24 m local at 0.05 m + 48 m global at 0.10 m) stays written down in the params file header either way.
4. **"smoother" (§2's node list).** **Ruling:** the *velocity* smoother, not a path smoother. §5 pins the meaning — "`vy` pinned to 0 at the smoother". No `smoother_server` is launched: Theta\*'s any-angle output is already the slowly-moving ICR RPP wants, and a path smoother would round exactly the corners the planner chose. Six lifecycle nodes, not seven.
5. **`ros-humble-grid-map-costmap-2d` (§13).** **Ruling:** not installed, not needed. §13 predates SP7's decision to publish the seed as an `OccupancyGrid`; a stock `StaticLayer` consumes that. The deb-carry runbook in Task 7 covers it if a future layer ever wants it.
6. **The rover's footprint.** **Ruling:** a circle, `robot_radius: 0.80`, not a polygon. This chassis point-turns, so it sweeps its circumscribed circle; a rectangle would give orientation-dependent clearance the IK cannot actually promise.
7. **`waypoint_follower`.** **Ruling:** not launched. SP11's waypoint list goes through `NavigateThroughPoses`, which is why both BT XMLs are shipped and pinned.
8. **Autonomy's speed cap.** §10 says autonomy starts at the manual cap and rises "only on measured evidence". **Ruling:** `0.05 m/s` / `0.1 rad/s` are written in one place — the velocity smoother — and the RPP `desired_linear_vel` matches so the controller never asks for more than the smoother will pass. Raising them is an SP10/SP12 decision with a sim run and a yard run behind it, and the spec's own margin note ("only raise that if I specifically tell you to") applies.
9. **"Prior scan as a `StaticLayer`, OFF by default" (§5).** **Ruling:** no prior scan is shipped, and none is loaded — there is no recorded map (Ruling 1). The `StaticLayer` SP9 *does* enable is a different thing entirely: it is how SP7's live `/autonomy/costmap_seed` reaches the costmaps, which is the whole point of the plan. §5's clause is about a *prior* map layered under the live one; that layer does not exist yet and is not written out. This is an explicit omission, not an oversight.
10. **"MPPI configured but parked until CPU is measured" (§5).** **Ruling:** no MPPI block at all, not even a parked one. §5 makes the CPU measurement the precondition, and Task 7 is the first time this stack's CPU is measured on the Orin — with no camera. Writing a parked MPPI configuration now would mean writing numbers no one has evidence for into a file whose credibility rests on every number being sourced. RotationShim + RPP is the shipped controller; MPPI is an SP10 decision, taken against the numbers Task 7 records. Also an explicit omission.

---

## Task 1: The `navi_nav2` package, and proof the stack it needs is actually there

**Files:** `rover/src/navi_nav2/{package.xml,setup.py,setup.cfg}`, `rover/src/navi_nav2/resource/navi_nav2`, `rover/src/navi_nav2/navi_nav2/__init__.py`, `rover/src/navi_nav2/test/test_nav2_available.py`

A Nav2 plugin that is not installed does not fail at launch with a clear message — it fails inside `pluginlib` at the *configure* transition, tens of seconds in, with a class-loader error most people read as a typo. This test is the loud version of that, and it is also the check that a fresh Orin flash still has what it needs.

- [ ] Write `rover/src/navi_nav2/test/test_nav2_available.py`:

```python
"""Every Nav2 package this bringup names must actually be installed.

A missing plugin package does not fail at launch - it fails inside
pluginlib during the configure transition, tens of seconds later, with a
class-loader message that reads like a typo in the parameter file. This
test is that failure, moved to the front and given a name.

Verified 2026-08-31: laptop and Orin both carry navigation2 1.1.20, the
same 31 ros-humble-nav2* packages, arm64 and amd64 respectively.
"""

import shutil
import subprocess

import pytest

REQUIRED_PACKAGES = [
    'nav2_lifecycle_manager',
    'nav2_planner',
    'nav2_controller',
    'nav2_behaviors',
    'nav2_bt_navigator',
    'nav2_velocity_smoother',
    'nav2_collision_monitor',
    'nav2_costmap_2d',
    'nav2_theta_star_planner',
    'nav2_smac_planner',
    'nav2_regulated_pure_pursuit_controller',
    'nav2_rotation_shim_controller',
    'nav2_msgs',
    'nav2_common',
]


@pytest.mark.parametrize('package', REQUIRED_PACKAGES)
def test_the_package_is_installed(package):
    assert shutil.which('ros2') is not None, "source /opt/ros/humble/setup.bash first"
    found = subprocess.run(['ros2', 'pkg', 'prefix', package],
                           capture_output=True, text=True)
    assert found.returncode == 0, (
        f"{package} is missing. On a machine with internet: "
        f"sudo apt install ros-humble-{package.replace('_', '-')}. "
        f"On the Orin, carry the debs over - see the runbook in "
        f"rover/src/navi_nav2/launch/nav2_bringup.launch.py.")


def test_the_planner_and_controller_plugins_load_from_pluginlib():
    """The four plugin classes named in params/nav2_rover.yaml, spelled the
    way pluginlib spells them - checked against the installed plugin XMLs,
    not against memory."""
    import os
    wanted = {
        'nav2_theta_star_planner': 'nav2_theta_star_planner/ThetaStarPlanner',
        'nav2_smac_planner': 'nav2_smac_planner/SmacPlanner2D',
        'nav2_rotation_shim_controller':
            'nav2_rotation_shim_controller::RotationShimController',
        'nav2_regulated_pure_pursuit_controller':
            'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
    }
    for package, class_name in wanted.items():
        share = subprocess.run(['ros2', 'pkg', 'prefix', '--share', package],
                               capture_output=True, text=True, check=True).stdout.strip()
        blob = ''
        for entry in os.listdir(share):
            if entry.endswith('.xml') and entry != 'package.xml':
                with open(os.path.join(share, entry)) as handle:
                    blob += handle.read()
        assert class_name in blob, f"{class_name} not declared by {package}"
```

- [ ] Run it — it passes on this laptop today (verified: all 31 packages present). If it fails on some other machine, that machine installs `ros-humble-navigation2` and `ros-humble-nav2-bringup` from apt (laptop, has internet) or by carried debs (Orin — Task 7's runbook).
- [ ] Write `rover/src/navi_nav2/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://relaxng.org/ns/structure/1.0"?>
<package format="3">
  <name>navi_nav2</name>
  <version>0.1.0</version>
  <description>Nav2 for the Asterope rover: the parameter file, the two
  no-reversing behaviour trees, the single bringup launch, and the
  deterministic elevation fixture the offline planning test plans
  against. Nav2's velocity leaves this package on /autonomy_twist and is
  read by mode_supervisor, which remains the only publisher of
  /rover_twist.</description>
  <maintainer email="oxe.pxs@gmail.com">star</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>navi_autonomy</depend>
  <depend>navi_localization</depend>

  <exec_depend>nav2_lifecycle_manager</exec_depend>
  <exec_depend>nav2_planner</exec_depend>
  <exec_depend>nav2_controller</exec_depend>
  <exec_depend>nav2_behaviors</exec_depend>
  <exec_depend>nav2_bt_navigator</exec_depend>
  <exec_depend>nav2_velocity_smoother</exec_depend>
  <exec_depend>nav2_collision_monitor</exec_depend>
  <exec_depend>nav2_costmap_2d</exec_depend>
  <exec_depend>nav2_theta_star_planner</exec_depend>
  <exec_depend>nav2_smac_planner</exec_depend>
  <exec_depend>nav2_regulated_pure_pursuit_controller</exec_depend>
  <exec_depend>nav2_rotation_shim_controller</exec_depend>
  <exec_depend>nav2_msgs</exec_depend>
  <exec_depend>nav2_common</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <exec_depend>python3-yaml</exec_depend>

  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] Write `rover/src/navi_nav2/setup.py` (the `data_files` are the point — params, launch and behaviour trees must reach `share/`):

```python
import os
from glob import glob

from setuptools import setup

package_name = 'navi_nav2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'behavior_trees'),
         glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='star',
    maintainer_email='oxe.pxs@gmail.com',
    description='Nav2 bringup, parameters and the offline planning fixture.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fixture_seed_publisher = navi_nav2.fixture_seed_publisher:main',
        ],
    },
)
```

- [ ] Write `rover/src/navi_nav2/setup.cfg`:

```
[develop]
script_dir=$base/lib/navi_nav2
[install]
install_scripts=$base/lib/navi_nav2
```

- [ ] Create the empty marker `rover/src/navi_nav2/resource/navi_nav2` and `rover/src/navi_nav2/navi_nav2/__init__.py`.
- [ ] `colcon build --packages-select navi_nav2` succeeds (it will warn that `fixture_seed_publisher` does not exist yet — write a one-line placeholder module `navi_nav2/fixture_seed_publisher.py` with `def main(): raise NotImplementedError("Task 5")` so the build is honest, and Task 5 replaces it).
- [ ] Run the pure suite. Commit: `git add rover/src/navi_nav2 && git commit -m "navi_nav2: the package, and a test that the Nav2 plugins it names are installed"`.

---

## Task 2: The parameter file, and a test that reads every constrained number back out of it

**Files:** `rover/src/navi_nav2/params/nav2_rover.yaml`, `rover/src/navi_nav2/test/test_params.py`

Nav2 ignores an unknown parameter key silently. A file that says `allow_reversal: false` configures a stack that reverses, and nothing anywhere says so. This task pins the file; Task 6 pins the *running node's* view of the file, which is the half that catches the typo.

- [ ] Write the test first, `rover/src/navi_nav2/test/test_params.py`:

```python
"""Every number the spec fixes, read back out of params/nav2_rover.yaml.

This is the file-level half of the guard. The graph-level half is in
test_offline_planning.py, which asks the running nodes what they think
their parameters are - because Nav2 ignores a key it does not know, and a
typo here would otherwise configure the stack with the plugin's defaults
and say nothing at all.
"""

import os

import yaml

PARAMS = os.path.join(os.path.dirname(__file__), '..', 'params', 'nav2_rover.yaml')


def params():
    with open(PARAMS) as handle:
        return yaml.safe_load(handle)


def node(name):
    return params()[name]['ros__parameters']


def costmap(which):
    return params()[which][which]['ros__parameters']


# -- the speed caps (spec section 10) ---------------------------------------

def test_the_velocity_smoother_carries_the_manual_cap():
    smoother = node('velocity_smoother')
    assert smoother['max_velocity'] == [0.05, 0.0, 0.1]
    assert smoother['min_velocity'] == [-0.15, 0.0, -0.1]


def test_vy_is_pinned_to_zero_at_the_smoother():
    smoother = node('velocity_smoother')
    assert smoother['max_velocity'][1] == 0.0
    assert smoother['min_velocity'][1] == 0.0
    assert smoother['max_accel'][1] == 0.0
    assert smoother['max_decel'][1] == 0.0


def test_the_angular_acceleration_limit_is_the_one_the_spec_names():
    smoother = node('velocity_smoother')
    assert smoother['max_accel'][2] == 0.5
    assert smoother['max_decel'][2] == -0.5


def test_the_controller_never_asks_for_more_than_the_smoother_passes():
    follow = node('controller_server')['FollowPath']
    smoother = node('velocity_smoother')
    assert follow['desired_linear_vel'] == smoother['max_velocity'][0] == 0.05
    assert follow['rotate_to_heading_angular_vel'] == smoother['max_velocity'][2] == 0.1


# -- no reversing (spec section 5) ------------------------------------------

def test_the_controller_may_not_reverse():
    assert node('controller_server')['FollowPath']['allow_reversing'] is False


def test_reverse_speed_is_capped_at_the_spec_floor():
    assert node('velocity_smoother')['min_velocity'][0] >= -0.15


def test_every_collision_polygon_looks_only_forwards():
    monitor = node('collision_monitor')
    assert monitor['polygons'], "at least one polygon, or nothing is watched"
    for name in monitor['polygons']:
        points = monitor[name]['points']
        xs = points[0::2]
        assert min(xs) >= 0.10, \
            f"{name} has a point behind the x >= 0.10 line: {points}"
        assert max(xs) > 0.0, f"{name} is degenerate: {points}"


# -- the seed contract (SP7) ------------------------------------------------

def test_both_costmaps_read_the_seed_at_the_elevation_resolution():
    for which in ('global_costmap', 'local_costmap'):
        layer = costmap(which)['static_layer']
        assert costmap(which)['resolution'] == 0.05
        assert layer['map_topic'] == '/autonomy/costmap_seed'
        assert layer['map_subscribe_transient_local'] is True
        assert layer['subscribe_to_updates'] is False


def test_the_scaled_cost_band_survives_the_static_layer():
    """trinary_costmap true would collapse SP7's 0..99 band to free/lethal
    and throw away every gradient the traversability layer computed."""
    for which in ('global_costmap', 'local_costmap'):
        layer = costmap(which)['static_layer']
        assert layer['trinary_costmap'] is False
        assert layer['lethal_cost_threshold'] == 100
        assert layer['unknown_cost_value'] == -1


def test_unseen_ground_is_not_driveable_ground():
    for which in ('global_costmap', 'local_costmap'):
        assert costmap(which)['track_unknown_space'] is True
    assert node('planner_server')['GridBased']['allow_unknown'] is False
    assert node('planner_server')['SmacBased']['allow_unknown'] is False


def test_the_global_costmap_is_the_48_m_window():
    globals_ = costmap('global_costmap')
    assert globals_['width'] == 48 and globals_['height'] == 48
    assert globals_['rolling_window'] is True


# -- the cloud layers are wired but off (no cloud_filter, no camera) --------

def test_the_cloud_layers_are_configured_and_disabled():
    assert costmap('global_costmap')['obstacle_layer']['enabled'] is False
    assert costmap('local_costmap')['voxel_layer']['enabled'] is False
    assert node('collision_monitor')['points_filtered']['enabled'] is False


def test_the_cloud_layers_point_at_the_topic_cloud_filter_will_publish():
    assert (costmap('global_costmap')['obstacle_layer']['points_filtered']['topic']
            == '/autonomy/points_filtered')
    assert (costmap('local_costmap')['voxel_layer']['points_filtered']['topic']
            == '/autonomy/points_filtered')
    assert node('collision_monitor')['points_filtered']['topic'] == '/autonomy/points_filtered'


# -- frames and odometry (spec section 6, SP6) ------------------------------

def test_nav2_reads_the_odometry_sp6_publishes():
    assert node('controller_server')['odom_topic'] == '/localization/odom_local'
    assert node('bt_navigator')['odom_topic'] == '/localization/odom_local'
    assert node('velocity_smoother')['odom_topic'] == '/localization/odom_local'


def test_every_node_stands_on_base_footprint():
    assert node('bt_navigator')['robot_base_frame'] == 'base_footprint'
    assert node('behavior_server')['robot_base_frame'] == 'base_footprint'
    assert node('collision_monitor')['base_frame_id'] == 'base_footprint'
    for which in ('global_costmap', 'local_costmap'):
        assert costmap(which)['robot_base_frame'] == 'base_footprint'
    assert costmap('global_costmap')['global_frame'] == 'map'
    assert costmap('local_costmap')['global_frame'] == 'odom'


# -- the velocity chain ------------------------------------------------------

def test_nav2_writes_only_to_autonomy_twist():
    monitor = node('collision_monitor')
    assert monitor['cmd_vel_in_topic'] == 'cmd_vel_smoothed'
    assert monitor['cmd_vel_out_topic'] == '/autonomy_twist'


# -- the planners (spec section 5) ------------------------------------------

def test_theta_star_plans_and_smac_is_loaded_beside_it_for_ab():
    planner = node('planner_server')
    assert planner['planner_plugins'] == ['GridBased', 'SmacBased']
    assert planner['GridBased']['plugin'] == 'nav2_theta_star_planner/ThetaStarPlanner'
    assert planner['SmacBased']['plugin'] == 'nav2_smac_planner/SmacPlanner2D'


def test_no_parameter_this_build_does_not_have_is_set():
    """Verified against the installed 1.1.20 binaries on 2026-08-31 by
    dumping the plugin .so strings: none of these keys is declared, and
    Nav2 would ignore them without a word.

    (SmacPlanner2D's use_final_approach_orientation IS declared in 1.1.20 -
    it is simply not set here, which is a choice, not an absence.)"""
    assert 'w_heuristic_cost' not in node('planner_server')['GridBased']
    follow = node('controller_server')['FollowPath']
    assert 'use_fixed_curvature_lookahead' not in follow
    assert 'curvature_lookahead_dist' not in follow
    assert 'odom_topic' not in node('behavior_server')


def test_the_controller_is_the_shim_wrapping_pure_pursuit():
    follow = node('controller_server')['FollowPath']
    assert follow['plugin'] == 'nav2_rotation_shim_controller::RotationShimController'
    assert (follow['primary_controller']
            == 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController')


def test_the_inflation_factors_agree_between_layer_and_controller():
    """RPP scales speed by cost using its own copy of the inflation curve;
    if the two disagree it slows down in the wrong places and says nothing."""
    follow = node('controller_server')['FollowPath']
    for which in ('global_costmap', 'local_costmap'):
        assert (costmap(which)['inflation_layer']['cost_scaling_factor']
                == follow['inflation_cost_scaling_factor'] == 3.0)


def test_the_footprint_is_the_circle_a_point_turning_rover_sweeps():
    for which in ('global_costmap', 'local_costmap'):
        assert costmap(which)['robot_radius'] == 0.80
        assert 'footprint' not in costmap(which)
```

- [ ] Watch it fail (no params file yet). Then write `rover/src/navi_nav2/params/nav2_rover.yaml`:

```yaml
# Nav2 for the Asterope rover.  Spec: docs/superpowers/specs/autonomy-plan.md
# sections 5, 6, 9 and 10.
#
# Speeds are the section 10 cap: 0.05 m/s and 0.1 rad/s, the same tenth
# manual drive is capped to.  They live here and in exactly one other place
# (gamepad_input.py, for manual); raising them needs one clean sim run and
# one clean yard run per stage, and an explicit instruction.
#
# Nothing here reverses.  allow_reversing is false, the reverse floor at the
# smoother is -0.15 m/s, the collision monitor's polygons start at x = +0.10
# and the BackUp behaviour in both behaviour trees is 0.30 m - the spec caps
# it at 0.6 m.  Nothing on this rover looks backwards.
#
# The cloud layers (global obstacle_layer, local voxel_layer, and the
# collision monitor's points_filtered source) are written out in full and
# shipped disabled: cloud_filter does not exist yet and no camera has been
# on the rover since these were written.  Enabling them is three booleans.
#
# CPU fallback, if two 0.05 m costmaps prove too heavy on the Orin (spec
# section 11 risk 6): local 24 m at 0.05 m, global 48 m at 0.10 m.  Measure
# before changing anything - the numbers from Task 7 are in the launch file.

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_footprint
    odom_topic: /localization/odom_local
    bt_loop_duration: 10
    default_server_timeout: 20
    wait_for_service_timeout: 1000
    transform_tolerance: 0.5
    # Both are rewritten to this package's share directory by the launch
    # file (nav2_common.launch.RewrittenYaml); the value here is a marker
    # so the rewrite has a key to land on.
    default_nav_to_pose_bt_xml: REWRITTEN_AT_LAUNCH
    default_nav_through_poses_bt_xml: REWRITTEN_AT_LAUNCH

planner_server:
  ros__parameters:
    use_sim_time: false
    expected_planner_frequency: 1.0
    planner_plugins: ["GridBased", "SmacBased"]
    GridBased:
      plugin: "nav2_theta_star_planner/ThetaStarPlanner"
      # Any-angle: few ICR changes, which is what this steering chassis is
      # bad at.  Hybrid-A* and lattice are rejected in the spec - they
      # impose a turning radius the rover does not have and forbid the
      # point turn it does.
      how_many_corners: 8
      w_euc_cost: 1.0
      w_traversal_cost: 2.0
      allow_unknown: false
      use_final_approach_orientation: false
    SmacBased:
      # The A/B second opinion (spec section 5).  Same costmap, same goal.
      plugin: "nav2_smac_planner/SmacPlanner2D"
      tolerance: 0.25
      downsample_costmap: false
      downsampling_factor: 1
      allow_unknown: false
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 5.0
      cost_travel_multiplier: 2.0

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 10.0
    odom_topic: /localization/odom_local
    min_x_velocity_threshold: 0.002
    # vy is pinned to zero at the smoother, so a y reading is noise and must
    # never be read as movement.
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.005
    failure_tolerance: 0.5
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      # 0.25 m in 30 s is 0.008 m/s - a twelfth of the cap, so a slow but
      # real crawl over rough ground is not read as being stuck.
      required_movement_radius: 0.25
      movement_time_allowance: 30.0
    general_goal_checker:
      stateful: true
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.35
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      primary_controller: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      # -- the shim: turn to face the path before pursuing it, because this
      # chassis point-turns far better than it arcs.
      angular_dist_threshold: 0.5
      forward_sampling_distance: 0.30
      rotate_to_goal_heading: false
      simulate_ahead_time: 1.0
      # -- shared by shim and RPP (one key, both read it)
      rotate_to_heading_angular_vel: 0.1
      max_angular_accel: 0.5
      # -- regulated pure pursuit
      desired_linear_vel: 0.05
      lookahead_dist: 0.45
      min_lookahead_dist: 0.25
      max_lookahead_dist: 0.70
      lookahead_time: 6.0
      use_velocity_scaled_lookahead_dist: true
      transform_tolerance: 0.3
      min_approach_linear_velocity: 0.02
      approach_velocity_scaling_dist: 0.60
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 2.0
      use_regulated_linear_velocity_scaling: true
      use_cost_regulated_linear_velocity_scaling: true
      regulated_linear_scaling_min_radius: 0.90
      regulated_linear_scaling_min_speed: 0.02
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 0.5
      allow_reversing: false
      use_interpolation: true
      max_robot_pose_search_dist: 10.0
      cost_scaling_dist: 0.60
      cost_scaling_gain: 1.0
      # Must equal inflation_layer.cost_scaling_factor on both costmaps.
      inflation_cost_scaling_factor: 3.0

behavior_server:
  ros__parameters:
    use_sim_time: false
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    # No assisted_teleop (a second cmd_vel source), no drive_on_heading
    # (a negative distance is a reversal).  The behaviour trees call
    # exactly these three.
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
    wait:
      plugin: "nav2_behaviors/Wait"
    global_frame: odom
    robot_base_frame: base_footprint
    transform_tolerance: 0.5
    simulate_ahead_time: 2.0
    max_rotational_vel: 0.1
    min_rotational_vel: 0.05
    rotational_acc_lim: 0.5
    # NOTE: this node has no odom_topic parameter - its OdomSmoother
    # hard-codes the name "odom".  The launch file remaps it to
    # /localization/odom_local.

velocity_smoother:
  ros__parameters:
    use_sim_time: false
    smoothing_frequency: 20.0
    scale_velocities: false
    feedback: "OPEN_LOOP"
    # Spec section 10.  The y band is zero in both directions: this is
    # where vy is pinned for the whole stack.
    max_velocity: [0.05, 0.0, 0.1]
    min_velocity: [-0.15, 0.0, -0.1]
    max_accel: [0.5, 0.0, 0.5]
    max_decel: [-0.5, 0.0, -0.5]
    odom_topic: "/localization/odom_local"
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0

collision_monitor:
  ros__parameters:
    use_sim_time: false
    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    cmd_vel_in_topic: "cmd_vel_smoothed"
    # The single point at which Nav2's velocity reaches the rest of the
    # rover.  mode_supervisor reads this and is the only publisher of
    # /rover_twist.
    cmd_vel_out_topic: "/autonomy_twist"
    transform_tolerance: 0.5
    source_timeout: 5.0
    base_shift_correction: true
    stop_pub_timeout: 2.0
    # Forward only (spec section 5, risk 4).  Every point has x >= 0.10:
    # nothing looks behind or beside, so nothing may claim to.
    polygons: ["ForwardStop", "ForwardSlow"]
    ForwardStop:
      type: "polygon"
      points: [0.90, 0.45, 0.90, -0.45, 0.10, -0.45, 0.10, 0.45]
      action_type: "stop"
      max_points: 2
      visualize: true
      polygon_pub_topic: "collision_stop_polygon"
      enabled: true
    ForwardSlow:
      type: "polygon"
      points: [1.60, 0.60, 1.60, -0.60, 0.10, -0.60, 0.10, 0.60]
      action_type: "slowdown"
      slowdown_ratio: 0.3
      max_points: 2
      visualize: true
      polygon_pub_topic: "collision_slowdown_polygon"
      enabled: true
    observation_sources: ["points_filtered"]
    points_filtered:
      type: "pointcloud"
      topic: "/autonomy/points_filtered"
      min_height: 0.10
      max_height: 1.20
      # OFF until cloud_filter exists and a camera is on the rover.  The
      # source is declared rather than omitted so the wiring is written
      # down and turning it on is one boolean.
      enabled: false

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: false
      global_frame: map
      robot_base_frame: base_footprint
      update_frequency: 1.0
      publish_frequency: 0.5
      # 0.05 m to match the elevation map exactly.  Resampling smears the
      # step edges that matter most (spec section 5).
      resolution: 0.05
      # Rolling, 48 m: the same window tile_aggregator keeps, so the seed
      # never has to resize the master grid when the window recentres.
      rolling_window: true
      width: 48
      height: 48
      track_unknown_space: true
      transform_tolerance: 0.5
      robot_radius: 0.80
      footprint_padding: 0.03
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        enabled: true
        map_topic: /autonomy/costmap_seed
        map_subscribe_transient_local: true
        subscribe_to_updates: false
        # trinary false, or SP7's 0..99 band collapses to free/lethal and
        # every gradient below the thresholds is thrown away.
        trinary_costmap: false
        lethal_cost_threshold: 100
        unknown_cost_value: -1
        transform_tolerance: 0.5
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: false
        footprint_clearing_enabled: true
        observation_sources: points_filtered
        points_filtered:
          topic: /autonomy/points_filtered
          data_type: "PointCloud2"
          marking: true
          clearing: true
          min_obstacle_height: 0.10
          max_obstacle_height: 1.20
          obstacle_max_range: 8.0
          obstacle_min_range: 0.30
          raytrace_max_range: 8.0
          raytrace_min_range: 0.30
          expected_update_rate: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: true
        inflation_radius: 1.20
        cost_scaling_factor: 3.0
        inflate_unknown: false
        inflate_around_unknown: false

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: false
      global_frame: odom
      robot_base_frame: base_footprint
      update_frequency: 5.0
      publish_frequency: 2.0
      resolution: 0.05
      rolling_window: true
      width: 8
      height: 8
      track_unknown_space: true
      transform_tolerance: 0.5
      robot_radius: 0.80
      footprint_padding: 0.03
      # The seed is in the local costmap too, deliberately: it is the only
      # thing that knows about holes, and a controller that could not see
      # the rim would happily follow a stale plan into one.
      plugins: ["static_layer", "voxel_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        enabled: true
        map_topic: /autonomy/costmap_seed
        map_subscribe_transient_local: true
        subscribe_to_updates: false
        trinary_costmap: false
        lethal_cost_threshold: 100
        unknown_cost_value: -1
        transform_tolerance: 0.5
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: false
        footprint_clearing_enabled: true
        publish_voxel_map: false
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 24
        max_obstacle_height: 1.20
        mark_threshold: 0
        observation_sources: points_filtered
        points_filtered:
          topic: /autonomy/points_filtered
          data_type: "PointCloud2"
          marking: true
          clearing: true
          min_obstacle_height: 0.10
          max_obstacle_height: 1.20
          obstacle_max_range: 8.0
          obstacle_min_range: 0.30
          raytrace_max_range: 8.0
          raytrace_min_range: 0.30
          expected_update_rate: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: true
        inflation_radius: 1.20
        cost_scaling_factor: 3.0
        inflate_unknown: false
        inflate_around_unknown: false

lifecycle_manager_navigation:
  ros__parameters:
    use_sim_time: false
    autostart: true
    bond_timeout: 4.0
    attempt_respawn_reconnection: true
    bond_respawn_max_duration: 10.0
    node_names:
      - controller_server
      - planner_server
      - behavior_server
      - bt_navigator
      - velocity_smoother
      - collision_monitor
```

- [ ] Run the pure suite; every assertion above must pass. Commit: `git add rover/src/navi_nav2/params rover/src/navi_nav2/test/test_params.py && git commit -m "navi_nav2: the parameter file, with every number the spec fixes pinned by a test"`.

---

## Task 3: The behaviour trees and the single bringup launch

**Files:** `rover/src/navi_nav2/behavior_trees/{navigate_to_pose_no_reverse.xml,navigate_through_poses_no_reverse.xml}`, `rover/src/navi_nav2/launch/nav2_bringup.launch.py`, `rover/src/navi_nav2/test/test_bringup_launch.py`

- [ ] Copy the stock trees (they already carry a 0.30 m BackUp, inside the spec's 0.6 m cap — copying rather than referencing means the file that is tested is the file that runs, and a Nav2 upgrade cannot change it under us):

```
cp /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml \
   rover/src/navi_nav2/behavior_trees/navigate_to_pose_no_reverse.xml
cp /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml \
   rover/src/navi_nav2/behavior_trees/navigate_through_poses_no_reverse.xml
```

- [ ] Write the test first, `rover/src/navi_nav2/test/test_bringup_launch.py`:

```python
"""The launch file's wiring, asserted without launching anything.

generate_launch_description() is not called: it resolves package share
directories and would need the workspace installed.  Everything asserted
here is a module-level constant or a small pure helper, which is why they
exist as such.
"""

import os
import xml.etree.ElementTree as ET

import pytest

from navi_nav2 import bringup

TREES = os.path.join(os.path.dirname(__file__), '..', 'behavior_trees')


# -- the behaviour trees ----------------------------------------------------

@pytest.mark.parametrize('tree', ['navigate_to_pose_no_reverse.xml',
                                  'navigate_through_poses_no_reverse.xml'])
def test_backup_stays_inside_the_spec_cap(tree):
    root = ET.parse(os.path.join(TREES, tree)).getroot()
    backups = list(root.iter('BackUp'))
    for backup in backups:
        assert float(backup.get('backup_dist')) <= 0.6, "spec section 5: BackUp capped 0.6 m"
        assert float(backup.get('backup_speed')) <= 0.15


@pytest.mark.parametrize('tree', ['navigate_to_pose_no_reverse.xml',
                                  'navigate_through_poses_no_reverse.xml'])
def test_nothing_else_in_the_tree_drives_backwards(tree):
    root = ET.parse(os.path.join(TREES, tree)).getroot()
    for node in root.iter('DriveOnHeading'):
        assert float(node.get('dist_to_travel', '0')) >= 0.0
    assert not list(root.iter('AssistedTeleop')), \
        "assisted teleop is a second velocity source; the supervisor owns that job"


# -- the launch wiring ------------------------------------------------------

def test_the_lifecycle_manager_owns_exactly_the_six_servers():
    assert bringup.LIFECYCLE_NODES == [
        'controller_server', 'planner_server', 'behavior_server',
        'bt_navigator', 'velocity_smoother', 'collision_monitor']


def test_the_velocity_chain_ends_at_autonomy_twist_and_starts_nowhere_else():
    """controller and behaviours -> cmd_vel_nav -> smoother ->
    cmd_vel_smoothed -> collision monitor -> /autonomy_twist.  The
    intermediate names are remapped away from Nav2's default 'cmd_vel' so
    a stray subscriber cannot pick up an unmonitored velocity."""
    assert bringup.remappings('controller_server') == [
        ('cmd_vel', 'cmd_vel_nav')]
    assert bringup.remappings('behavior_server') == [
        ('cmd_vel', 'cmd_vel_nav'), ('odom', '/localization/odom_local')]
    assert bringup.remappings('velocity_smoother') == [('cmd_vel', 'cmd_vel_nav')]
    # The smoother's output keeps its own name, and the collision monitor
    # takes it by parameter (cmd_vel_in_topic), not by remap.
    assert bringup.remappings('collision_monitor') == []


def test_the_lifecycle_manager_and_the_launch_agree_on_the_node_list():
    """Two places name the six nodes - the parameter file and this module -
    and a manager waiting for a node nobody starts hangs the whole bringup
    with a message about bonds."""
    import os

    import yaml
    params = os.path.join(os.path.dirname(__file__), '..', 'params', 'nav2_rover.yaml')
    with open(params) as handle:
        managed = yaml.safe_load(handle)[
            'lifecycle_manager_navigation']['ros__parameters']['node_names']
    assert managed == bringup.LIFECYCLE_NODES


def test_no_node_is_remapped_onto_a_chassis_topic():
    for node in bringup.LIFECYCLE_NODES:
        for _, destination in bringup.remappings(node):
            assert destination not in ('/rover_twist', '/manual_twist'), \
                f"{node} would write to the chassis; only mode_supervisor may"


def test_the_behaviour_trees_are_rewritten_into_the_parameters():
    rewrites = bringup.bt_rewrites('/share/navi_nav2')
    assert rewrites['default_nav_to_pose_bt_xml'].endswith(
        '/behavior_trees/navigate_to_pose_no_reverse.xml')
    assert rewrites['default_nav_through_poses_bt_xml'].endswith(
        '/behavior_trees/navigate_through_poses_no_reverse.xml')


def test_the_bringup_can_run_without_perception_for_the_offline_test():
    assert 'perception' in bringup.LAUNCH_ARGUMENTS
    assert 'bench_fixture' in bringup.LAUNCH_ARGUMENTS
    assert bringup.LAUNCH_ARGUMENTS['bench_fixture'] == 'false', \
        "the bench fixture fakes the frames the ZED owns; never on by default"
```

- [ ] Write `rover/src/navi_nav2/navi_nav2/bringup.py` — the pure half of the launch file, so the test above needs no installed workspace:

```python
"""The launch file's wiring as data, so it can be tested without launching.

generate_launch_description() needs package share directories; these
constants and helpers do not, and they are what the wiring actually is.
"""

import os

LIFECYCLE_NODES = [
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'velocity_smoother',
    'collision_monitor',
]

LAUNCH_ARGUMENTS = {
    'params_file': '',          # filled from the share directory at launch
    'autostart': 'true',
    'perception': 'true',       # include SP7's tile_aggregator + traversability_layer
    'bench_fixture': 'false',   # fake seed, frames and odometry; NEVER with a ZED running
    'log_level': 'info',
}

ODOM_TOPIC = '/localization/odom_local'

# Nav2's internal velocity names, moved off "cmd_vel" so nothing can
# subscribe to an unmonitored velocity by accident.  The only name that
# leaves this stack is /autonomy_twist, and it is set as a parameter on the
# collision monitor (cmd_vel_out_topic), not as a remap.
_REMAPPINGS = {
    'controller_server': [('cmd_vel', 'cmd_vel_nav')],
    'behavior_server': [('cmd_vel', 'cmd_vel_nav'), ('odom', ODOM_TOPIC)],
    'velocity_smoother': [('cmd_vel', 'cmd_vel_nav')],
    'planner_server': [],
    'bt_navigator': [],
    'collision_monitor': [],
}


def remappings(node_name: str) -> list:
    """The (from, to) pairs for one node.  behavior_server's odom remap is
    not cosmetic: that node has no odom_topic parameter - its OdomSmoother
    hard-codes the name - so this is the only way it reads SP6's odometry."""
    return list(_REMAPPINGS[node_name])


def bt_rewrites(share_dir: str) -> dict:
    """The two behaviour-tree paths, as RewrittenYaml param_rewrites."""
    trees = os.path.join(share_dir, 'behavior_trees')
    return {
        'default_nav_to_pose_bt_xml':
            os.path.join(trees, 'navigate_to_pose_no_reverse.xml'),
        'default_nav_through_poses_bt_xml':
            os.path.join(trees, 'navigate_through_poses_no_reverse.xml'),
    }
```

- [ ] Write `rover/src/navi_nav2/launch/nav2_bringup.launch.py`:

```python
"""Nav2 for the Asterope rover: six lifecycle nodes and one manager.

    ros2 launch navi_nav2 nav2_bringup.launch.py

Arguments:
    params_file:=<path>     override params/nav2_rover.yaml
    autostart:=false        bring the nodes up unconfigured (debugging)
    perception:=false       do not include SP7's tile_aggregator and
                            traversability_layer (the offline test and the
                            bench smoke supply the seed themselves)
    bench_fixture:=true     publish the generated fixture seed, fake
                            map->odom and odom->base_footprint, and fake
                            /localization/odom_local.  NEVER run this with
                            the ZED up: the wrapper owns map->odom and this
                            would give base_footprint a second parent.
    log_level:=debug

The velocity chain:

    controller_server ---.
    behavior_server -----+--> cmd_vel_nav --> velocity_smoother
                                                    |
                                            cmd_vel_smoothed
                                                    |
                                            collision_monitor
                                                    |
                                             /autonomy_twist
                                                    |
                                             mode_supervisor  (SP5)
                                                    |
                                              /rover_twist

Nothing in this launch publishes /rover_twist or /manual_twist.  The
collision monitor is last on purpose: it has the final word on every
velocity, including the behaviours' recovery motions.

MUST DO at the next camera session (none of it is testable without a ZED):
  1. Set global_costmap.obstacle_layer.enabled, local_costmap.voxel_layer.
     enabled and collision_monitor.points_filtered.enabled to true, once
     cloud_filter publishes /autonomy/points_filtered.
  2. Check the seed lines up with real terrain: drive to a known rock, and
     confirm the lethal cells in /global_costmap/costmap sit on it and not
     a metre away.  A misaligned seed looks entirely plausible.
  3. Re-measure CPU with perception live (spec section 11 risk 6) and
     compare against the camera-less numbers recorded below.
  4. Watch the RotationShim hand over to RPP on the real chassis and
     measure the steering slew before any speed stage is raised.
  5. Confirm the collision polygons against the real footprint with the
     rover on blocks before they are trusted in the yard.

Orin measurements (camera-less, fixture seed, Task 7):
    RECORD HERE: nav2 total CPU %, RSS, time to all-active.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from navi_nav2 import bringup


def generate_launch_description():
    share = get_package_share_directory('navi_nav2')
    default_params = os.path.join(share, 'params', 'nav2_rover.yaml')

    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    perception = LaunchConfiguration('perception')
    bench_fixture = LaunchConfiguration('bench_fixture')
    log_level = LaunchConfiguration('log_level')

    arguments = [
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('perception', default_value='true'),
        DeclareLaunchArgument('bench_fixture', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    # The behaviour-tree paths are the only thing the parameter file cannot
    # spell for itself: they are absolute paths into this package's share.
    configured = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=bringup.bt_rewrites(share),
        convert_types=True)

    def server(package, executable, name):
        return Node(
            package=package,
            executable=executable,
            name=name,
            output='screen',
            parameters=[configured],
            remappings=bringup.remappings(name),
            arguments=['--ros-args', '--log-level', log_level],
        )

    servers = GroupAction([
        server('nav2_controller', 'controller_server', 'controller_server'),
        server('nav2_planner', 'planner_server', 'planner_server'),
        server('nav2_behaviors', 'behavior_server', 'behavior_server'),
        server('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        server('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother'),
        server('nav2_collision_monitor', 'collision_monitor', 'collision_monitor'),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            # The node list comes from the parameter file like everything
            # else - one place, and test_bringup_launch.py checks it still
            # matches bringup.LIFECYCLE_NODES. autostart is the one thing
            # the command line may override.
            parameters=[configured, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
        ),
    ])

    sp7 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('navi_autonomy'),
            'launch', 'autonomy_perception.launch.py')),
        condition=IfCondition(perception))

    fixture = Node(
        package='navi_nav2',
        executable='fixture_seed_publisher',
        name='fixture_seed_publisher',
        output='screen',
        parameters=[{'bench_frames': True}],
        condition=IfCondition(bench_fixture),
    )

    return LaunchDescription(arguments + [sp7, fixture, servers])
```

- [ ] Run the pure suite. Build, then prove the launch file at least parses and the six nodes reach `active` with no perception and no fixture (they will complain about missing TF; that is expected and not fatal):

```
bash -c 'source /opt/ros/humble/setup.bash && cd rover && colcon build --packages-select navi_nav2 &&
  source install/local_setup.bash && cd .. && ROS_DOMAIN_ID=92 \
  timeout 40 ros2 launch navi_nav2 nav2_bringup.launch.py perception:=false 2>&1 | tail -40'
```

Expect `Managed nodes are active` from the lifecycle manager. Then, in a second shell on domain 92, `ros2 topic list` and confirm `/autonomy_twist` is present and `/rover_twist`, `/manual_twist` and `/cmd_vel` are **not**.

- [ ] Commit: `git add rover/src/navi_nav2/behavior_trees rover/src/navi_nav2/launch rover/src/navi_nav2/navi_nav2/bringup.py rover/src/navi_nav2/test/test_bringup_launch.py && git commit -m "navi_nav2: one launch file for the six lifecycle nodes, and the no-reversing behaviour trees"`.

---

## Task 4: `RosNav2Control` — the supervisor's stub becomes real

**Files:** `rover/src/navi_supervisor/navi_supervisor/ros_nav2_control.py`, `rover/src/navi_supervisor/navi_supervisor/mode_supervisor.py` (two small edits), `rover/src/navi_supervisor/package.xml`, `rover/src/navi_supervisor/test/test_ros_nav2_control.py`

The supervisor did not send the Nav2 goal — SP11's `goal_relay` will — so it holds no goal handle and cannot cancel through an action client. `action_msgs/srv/CancelGoal` is explicit about this: *"If the goal ID is zero and timestamp is zero, cancel all goals."* That is the right semantics anyway; on a takeover the supervisor wants every goal gone, not one it happens to know about.

Deactivation goes through the lifecycle manager's `manage_nodes` service with `ManageLifecycleNodes.PAUSE` (= 1), which deactivates all six nodes. Not `RESET` (which cleans up and would need a full reconfigure), not `SHUTDOWN` (which ends them).

**Both calls are `call_async` and neither ever waits.** By the time `_run_actions()` reaches them the supervisor has already published a zero twist; a Nav2 that has hung is precisely the case where waiting would be fatal.

- [ ] Write the test first, `rover/src/navi_supervisor/test/test_ros_nav2_control.py`:

```python
"""RosNav2Control against fake services, on a throwaway domain.

The contract this pins is the one nav2_control.py's docstring names: the
sequence the supervisor asks for must not change now that the stub has
stopped being a stub.  So the test drives the supervisor the way a takeover
does and watches what arrives on the wire.
"""

import os

# 93, not 91: test_mode_supervisor.py already owns 91, and two agents in
# this tree at once would otherwise cross-talk.  setdefault, so the suite
# command below - which exports one domain for the whole pytest process,
# because rclpy reads ROS_DOMAIN_ID once per process - still wins.
os.environ.setdefault("ROS_DOMAIN_ID", "93")   # throwaway; never the rover's

import json
import time

import pytest
import rclpy
from action_msgs.srv import CancelGoal
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from navi_supervisor.mode_supervisor import ModeSupervisor
from navi_supervisor.nav2_control import Nav2Control
from navi_supervisor.ros_nav2_control import (CANCEL_SERVICE, MANAGE_SERVICE,
                                              RosNav2Control)


class FakeNav2(Node):
    """The two services Nav2 exposes, and nothing else."""

    def __init__(self):
        super().__init__('fake_nav2')
        self.cancels = []
        self.commands = []
        self.create_service(CancelGoal, CANCEL_SERVICE, self._on_cancel)
        self.create_service(ManageLifecycleNodes, MANAGE_SERVICE, self._on_manage)

    def _on_cancel(self, request, response):
        self.cancels.append(request)
        response.return_code = CancelGoal.Response.ERROR_NONE
        return response

    def _on_manage(self, request, response):
        self.commands.append(request.command)
        response.success = True
        return response


@pytest.fixture
def graph():
    rclpy.init()
    fake = FakeNav2()
    supervisor = ModeSupervisor()
    supervisor.attach_nav2_control(RosNav2Control(supervisor))
    executor = SingleThreadedExecutor()
    executor.add_node(fake)
    executor.add_node(supervisor)
    yield fake, supervisor, executor
    executor.shutdown()
    fake.destroy_node()
    supervisor.destroy_node()
    rclpy.shutdown()


def spin(executor, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)


def test_it_is_the_interface_sp5_wrote_and_not_a_new_one():
    assert issubclass(RosNav2Control, Nav2Control)
    assert RosNav2Control.cancel_goal is not Nav2Control.cancel_goal
    assert RosNav2Control.deactivate is not Nav2Control.deactivate


def request_mode(supervisor, mode):
    message = String()
    message.data = json.dumps({"mode": mode})
    supervisor._on_mode_request(message)


def test_a_takeover_cancels_every_goal_and_pauses_the_stack(graph):
    """The supervisor starts in manual, and supervisor_state only queues
    the Nav2 actions when it was autonomous, so the takeover has to be a
    real takeover: into autonomous first, then back out."""
    fake, supervisor, executor = graph
    spin(executor, 2.0)          # service discovery

    request_mode(supervisor, "autonomous")
    spin(executor, 0.5)
    assert not fake.cancels, "entering autonomous must not cancel anything"

    request_mode(supervisor, "manual")
    spin(executor, 2.0)

    assert len(fake.cancels) == 1, "exactly one cancel per takeover"
    assert len(fake.commands) == 1
    assert fake.commands[0] == ManageLifecycleNodes.Request.PAUSE


def test_the_cancel_asks_for_every_goal_not_one(graph):
    """Zero uuid and zero stamp: action_msgs/srv/CancelGoal's 'cancel all
    goals'.  The supervisor never sent the goal, so it has no handle - and
    on a takeover it wants all of them gone regardless."""
    fake, supervisor, executor = graph
    spin(executor, 2.0)
    supervisor._nav2.cancel_goal("test")
    spin(executor, 1.5)

    goal_info = fake.cancels[0].goal_info
    assert list(goal_info.goal_id.uuid) == [0] * 16
    assert goal_info.stamp.sec == 0 and goal_info.stamp.nanosec == 0


def test_it_never_blocks_when_nav2_is_not_there(graph):
    """No Nav2 on the graph is the normal state for most of a session, and
    the one moment this is called is the moment the rover must stop."""
    _, supervisor, _ = graph
    orphan = RosNav2Control(supervisor,
                            cancel_service='/nowhere/_action/cancel_goal',
                            manage_service='/nowhere/manage_nodes')
    started = time.monotonic()
    orphan.cancel_goal("no nav2")
    orphan.deactivate("no nav2")
    assert time.monotonic() - started < 0.5


def test_repeated_calls_do_not_leak_futures(graph):
    """100 sends without spinning the executor: nothing is done(), so the
    only thing holding the list down is the truncation in _reap() - which
    runs *after* every append, so the bound is MAX_PENDING exactly, not
    MAX_PENDING + 1."""
    _, supervisor, executor = graph
    spin(executor, 2.0)
    for _ in range(50):
        supervisor._nav2.cancel_goal("spam")
        supervisor._nav2.deactivate("spam")
    spin(executor, 2.0)
    assert len(supervisor._nav2._pending) <= RosNav2Control.MAX_PENDING
```

- [ ] Write `rover/src/navi_supervisor/navi_supervisor/ros_nav2_control.py`:

```python
"""The real Nav2Control: cancel every goal, pause the stack, never wait.

SP5 wrote the decision (supervisor_state.py queues CANCEL_GOAL and
DEACTIVATE_NAV2); this is the half that reaches Nav2.

Two services, not an action client and not a lifecycle client per node:

  * The supervisor did not send the goal - SP11's goal_relay does - so it
    holds no goal handle.  action_msgs/srv/CancelGoal says a zero goal id
    with a zero stamp cancels *all* goals, which is what a takeover wants
    anyway.
  * nav2_msgs/srv/ManageLifecycleNodes with PAUSE deactivates all six nodes
    through the one manager that owns them, in the order it knows to be
    safe.  RESET would tear down the configuration and need a full
    reconfigure; SHUTDOWN would end the processes.  PAUSE is reversible
    with RESUME, which is what starting the next run needs.

Nothing here waits.  The supervisor has already published a zero twist by
the time it calls either method, and a hung Nav2 is exactly the case where
waiting would keep the rover moving.  If the service is not there, that is
a log line, not an exception - Nav2 is absent for most of a manual session.
"""

from action_msgs.srv import CancelGoal
from nav2_msgs.srv import ManageLifecycleNodes

from navi_supervisor.nav2_control import Nav2Control

CANCEL_SERVICE = '/navigate_to_pose/_action/cancel_goal'
MANAGE_SERVICE = '/lifecycle_manager_navigation/manage_nodes'


class RosNav2Control(Nav2Control):
    """The interface nav2_control.py declares, wired to a running Nav2."""

    MAX_PENDING = 16

    def __init__(self, node, cancel_service=CANCEL_SERVICE,
                 manage_service=MANAGE_SERVICE):
        self._node = node
        self._logger = node.get_logger()
        self._cancel_client = node.create_client(CancelGoal, cancel_service)
        self._manage_client = node.create_client(ManageLifecycleNodes,
                                                 manage_service)
        self._pending = []

    # -- the interface -----------------------------------------------------

    def cancel_goal(self, reason: str) -> None:
        # Default-constructed goal_info: a zero uuid and a zero stamp, which
        # action_msgs/srv/CancelGoal defines as "cancel all goals".
        self._send(self._cancel_client, CancelGoal.Request(),
                   'cancel every Nav2 goal', reason)

    def deactivate(self, reason: str) -> None:
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.PAUSE
        self._send(self._manage_client, request, 'pause the Nav2 stack', reason)

    # -- the one way either of them reaches the graph ----------------------

    def _send(self, client, request, what: str, reason: str) -> None:
        if not client.service_is_ready():
            # Not an error: Nav2 is not running for most of a session, and
            # the supervisor has already stopped the rover by now.
            self._logger.info(
                f"{what} ({reason}): {client.srv_name} is not there; nothing to do")
            return
        try:
            self._pending.append(client.call_async(request))
        except Exception as exc:
            self._logger.error(f"{what} ({reason}) failed to send: {exc!r}")
            return
        # Reap AFTER the append, never before: _reap() truncates only when
        # the list already exceeds MAX_PENDING, so reaping first would leave
        # a steady state of MAX_PENDING + 1 and the bound the docstring
        # promises would be off by one.
        self._reap()
        self._logger.info(f"{what} ({reason}): asked")

    def _reap(self) -> None:
        """Drop finished futures, and never grow without bound: this is
        called at the end of every _send(), and a Nav2 that never answers
        must not turn a takeover into a memory leak.  On return,
        len(self._pending) <= MAX_PENDING - that is the invariant
        test_repeated_calls_do_not_leak_futures asserts."""
        self._pending = [f for f in self._pending if not f.done()]
        if len(self._pending) > self.MAX_PENDING:
            for stale in self._pending[:-self.MAX_PENDING]:
                stale.cancel()
            self._pending = self._pending[-self.MAX_PENDING:]
```

- [ ] Edit `rover/src/navi_supervisor/navi_supervisor/mode_supervisor.py` — two additions, nothing removed (every existing SP5 test keeps passing because the constructor default is unchanged):

```python
    def attach_nav2_control(self, nav2_control):
        """Replace the Nav2 hook after construction.

        RosNav2Control needs this node to create its service clients, so it
        cannot be passed to __init__.  The constructor default stays
        NullNav2Control: a supervisor built by a test, or by anything that
        has no Nav2, must still record what it asked for.
        """
        self._nav2 = nav2_control
```

and in `main()`:

```python
def main():
    # Imported here, not at module scope: ros_nav2_control pulls in
    # nav2_msgs and action_msgs, and mode_supervisor must stay importable -
    # and runnable with NullNav2Control - on a box that has neither.  It
    # also keeps those two packages out of the import path of all 49
    # existing SP5 tests (11 in test_mode_supervisor.py, 38 in
    # test_supervisor_state.py).
    from navi_supervisor.ros_nav2_control import RosNav2Control

    rclpy.init()
    node = ModeSupervisor()
    # The stub is the constructor default so tests and Nav2-less bringups
    # keep working; a real run talks to a real Nav2.
    node.attach_nav2_control(RosNav2Control(node))
    try:
        rclpy.spin(node)
    ...
```

**No module-level import** of `ros_nav2_control` in `mode_supervisor.py`: the import lives inside `main()`, above.

- [ ] Add `<depend>nav2_msgs</depend>` and `<depend>action_msgs</depend>` to `rover/src/navi_supervisor/package.xml`.
- [ ] Run the whole supervisor suite on domain 93 — the SP5 tests and the new one both, 49 existing plus the new file's, all in one pytest process on one domain. Commit: `git add rover/src/navi_supervisor && git commit -m "navi_supervisor: the Nav2 hook stops being a stub - cancel every goal, pause the stack, never wait"`.

---

## Task 5: The fixture — a deterministic elevation window with a pit in the way

**Files:** `rover/src/navi_nav2/navi_nav2/fixture.py`, `rover/src/navi_nav2/navi_nav2/fixture_seed_publisher.py`, `rover/src/navi_nav2/test/test_fixture.py`

There is no recorded map (verified: nothing under `rover/`, `sim/`, `tests/`, `.worktrees/`). The fixture is therefore built out of SP7's own modules, seeded so it is byte-identical everywhere, and *it is code, not a 4 MB binary in a text repo*. When SP12 records a real yard bag, `elevation_from_npz()` takes its place and every assertion below runs unchanged.

The geometry is chosen so that the test can only pass for the right reason: the pit sits **on** the straight line from start to goal, so a planner that never saw the seed produces a path straight through it and fails; and the north corridor is wide enough (2.2 m of clear centre-line positions, against a rover that needs 0.85 m of clearance on each side) that a path definitely exists, so a failure means something is wrong rather than the problem being impossible.

- [ ] Write the test first, `rover/src/navi_nav2/test/test_fixture.py`:

```python
"""The fixture is generated, so its determinism is part of the contract."""

import hashlib

import numpy as np
import pytest

from navi_autonomy.traversability import LETHAL, UNKNOWN
from navi_autonomy.window import WINDOW_CELLS
from navi_localization.elevation_grid import RESOLUTION
from navi_nav2 import fixture


def test_the_window_is_sp7s_window():
    elevation = fixture.elevation()
    assert elevation.shape == (WINDOW_CELLS, WINDOW_CELLS)
    assert elevation.dtype == np.float32


def test_the_generator_is_deterministic_in_this_process():
    """Two calls, one terrain.  This half is a bit-exact pin and safe as
    one: it never crosses an architecture."""
    first = hashlib.sha256(fixture.elevation().tobytes()).hexdigest()
    second = hashlib.sha256(fixture.elevation().tobytes()).hexdigest()
    assert first == second


def test_it_is_the_same_fixture_on_every_machine():
    """The same terrain on the laptop, on the Orin and in six months -
    pinned by the structure of the int8 seed, which is what the planner
    actually consumes.

    NOT by a hash of the float32 elevation: numpy's sin/cos are
    SIMD-dispatched and are not bit-identical between amd64 (AVX2/AVX512)
    and arm64 (NEON/SVE), and a 1-ULP float64 difference survives the
    float32 cast whenever it straddles a rounding tie.  deploy_rover.sh
    --test runs this file on the Orin, so a float hash would fail there for
    a fixture that is, physically, the same terrain.
    """
    cost = fixture.seed()
    lethal = int((cost == LETHAL).sum())
    assert lethal == pytest.approx(fixture.LETHAL_CELLS,
                                   rel=fixture.LETHAL_CELLS_REL_TOLERANCE), \
        f"the fixture's lethal set moved: {lethal} cells"
    assert fixture.cell_of(*fixture.PIT_CENTRE) == fixture.PIT_CELL
    elevation = fixture.elevation()
    tolerance = fixture.ELEVATION_EXTREME_TOLERANCE_M
    assert float(elevation.min()) == pytest.approx(fixture.ELEVATION_MIN_M,
                                                   abs=tolerance)
    assert float(elevation.max()) == pytest.approx(fixture.ELEVATION_MAX_M,
                                                   abs=tolerance)


def test_the_pit_puts_lethal_cells_on_its_rim():
    """SP7's whole point: a hole is invisible to positive-only voxels, and
    this is the terrain that proves the seed carries it."""
    cost = fixture.seed()
    ix, iy = fixture.cell_of(*fixture.PIT_CENTRE)
    ring = cost[iy - 30:iy + 30, ix - 30:ix + 30]
    assert (ring == LETHAL).any(), "the pit rim must be lethal"


def test_the_pit_blocks_the_straight_line_from_start_to_goal():
    cost = fixture.seed()
    x0, y0 = fixture.START
    x1, y1 = fixture.GOAL
    blocked = False
    for t in np.linspace(0.0, 1.0, 400):
        ix, iy = fixture.cell_of(x0 + t * (x1 - x0), y0 + t * (y1 - y0))
        if cost[iy, ix] == LETHAL:
            blocked = True
    assert blocked, "a planner that ignored the seed must not be able to pass"


def test_the_start_and_the_goal_are_on_clear_ground():
    cost = fixture.seed()
    for x, y in (fixture.START, fixture.GOAL):
        assert fixture.clearance_cells(cost, x, y, radius_m=0.85) == 0, \
            "a start or goal inside the inflation makes the test meaningless"


def test_nothing_inside_the_window_is_unknown():
    """track_unknown_space is true on both costmaps, so an unknown cell in
    the middle of the fixture would make the problem unsolvable rather than
    hard, and the two failures look identical.

    The outermost ring is unknown by construction and stays that way:
    traversability._padded pads with NaN, so valid_layer is 0 there - the
    frontier of a mapped area really is unknown, and the fixture must not
    pretend otherwise.  Nothing plans within 20 m of the window edge."""
    interior = fixture.seed()[1:-1, 1:-1]
    assert (interior == UNKNOWN).sum() == 0


def test_the_occupancy_grid_says_where_it_starts():
    grid = fixture.occupancy_grid(stamp=None)
    assert grid.header.frame_id == 'map'
    assert grid.info.resolution == pytest.approx(RESOLUTION)
    assert grid.info.width == grid.info.height == WINDOW_CELLS
    # info.origin is the corner of cell (0, 0): a 48 m window centred on the
    # map origin starts at (-24, -24).
    assert grid.info.origin.position.x == pytest.approx(-24.0)
    assert grid.info.origin.position.y == pytest.approx(-24.0)


@pytest.mark.parametrize('x', [6.0, 9.0])
def test_the_north_corridor_is_wide_enough_for_the_rover(x):
    """A path has to exist, or a failing planner and an impossible problem
    look the same.  Checked where the corridor is narrowest: beside the pit
    (x = 6, squeezed between the rim and the northern boulder) and at the
    wall's north end (x = 9)."""
    cost = fixture.seed()
    clear = [y for y in np.arange(1.0, 9.0, 0.05)
             if fixture.clearance_cells(cost, x, float(y), radius_m=0.85) == 0]
    assert len(clear) * 0.05 >= 1.8, \
        f"corridor at x={x} is narrower than the rover plus margin"
```

- [ ] Write `rover/src/navi_nav2/navi_nav2/fixture.py`:

```python
"""The elevation window the offline planning test plans over.

Spec section 9 rung 3 says "a recorded elevation map".  There is none: no
bag, no npz, no camera on the rover the night this was written.  So the
fixture is *generated* - deterministically, out of SP7's own modules, so
that what is tested is the real cost curve and the real hole detection and
not a hand-written cost grid that agrees with itself.

It is code rather than a committed array on purpose: 960 x 960 float32 is
3.7 MB and its seed is 0.92 MB, and neither belongs in a repository whose
rover side is entirely text.  The constants below pin it instead - the
structure of the int8 seed, not a hash of the float32 elevation, because
numpy's transcendental kernels are not bit-identical across architectures
and this suite runs on the Orin as well as the laptop.

The terrain, in the map frame, 48 m centred on the origin:

    y
    ^         (6, 6.0) boulder r 0.9 h 0.60
    |                                    wall x = 9, y in [-8, 1.5]
    |   START (0,0) ---- pit (6,0) r 1.2 d 0.50 ---- GOAL (12,0)
    |         (6,-6.0) boulder r 0.9 h 0.60
    +---------------------------------------------> x

The pit sits on the straight line from start to goal, so a planner that
never received the seed cannot produce a passing path.  The wall, which
reaches from the southern edge to y = 1.5, closes the southern detour and
forces the way round to be the northern one.  The two boulders narrow that
corridor without closing it: beside the pit the clear band for the rover's
centre runs from y = 2.05 (pit rim plus the 0.85 m circle) to y = 4.25
(boulder minus the same), 2.2 m of it.  The problem is hard, not
impossible - a planner that cannot solve it is broken, and that is the
whole point of choosing the numbers this way.

When SP12 records a real yard bag, elevation_from_npz() replaces
elevation() and every assertion in test_offline_planning.py stands.
"""

import numpy as np
from nav_msgs.msg import OccupancyGrid

from navi_autonomy.grid_map_io import build_occupancy_grid
from navi_autonomy.traversability import seed_from_elevation
from navi_autonomy.window import WINDOW_CELLS
from navi_localization.elevation_grid import RESOLUTION

# The window is centred on the map origin: cell (0, 0) is the corner at
# (-24, -24), which is what info.origin carries.
ORIGIN_IX = -WINDOW_CELLS // 2
ORIGIN_IY = -WINDOW_CELLS // 2

START = (0.0, 0.0)
GOAL = (12.0, 0.0)
PIT_CENTRE = (6.0, 0.0)
PIT_RADIUS_M = 1.2
PIT_DEPTH_M = 0.50
BOULDERS = (((6.0, 6.0), 0.9, 0.60), ((6.0, -6.0), 0.9, 0.60))
WALL_X = 9.0
WALL_Y = (-8.0, 1.5)
WALL_HALF_THICKNESS_M = 0.15
WALL_HEIGHT_M = 0.50

# Gentle ground, well under the 25 degree slope and 0.14 m step thresholds:
# a 0.02 m amplitude over a 6 m wavelength is a 1.2 degree slope, plus
# 0.004 m of seeded noise.  Flat-as-a-table terrain would let a broken
# traversability layer pass.
GROUND_AMPLITUDE_M = 0.02
GROUND_WAVELENGTH_M = 6.0
NOISE_M = 0.004
NOISE_SEED = 20260831

# What the fixture is pinned by.  NOT a hash of the float32 elevation: numpy's
# sin/cos are SIMD-dispatched and are not bit-identical between amd64 and
# arm64, a 1-ULP float64 difference survives the float32 cast on a rounding
# tie, and deploy_rover.sh --test runs this suite on the Orin.  What matters
# is the int8 seed the planner actually consumes, so that is what is pinned -
# structurally.  (The default_rng(20260831).normal half *is* stream-stable by
# NumPy's compatibility policy; the trig half is not.)
LETHAL_CELLS = 0            # FILL_IN_FROM_THE_FIRST_RUN
LETHAL_CELLS_REL_TOLERANCE = 1e-3   # a threshold-straddling cell may flip
PIT_CELL = (600, 480)       # cell_of(*PIT_CENTRE), exact by construction
ELEVATION_MIN_M = -0.50     # the pit floor
ELEVATION_MAX_M = 0.60      # the boulder tops
# The ground wave (+-0.02 m) and the seeded noise (~4 sigma of 0.004 m) ride
# on top of both extremes, so the extremes are pinned to a band, not a value.
ELEVATION_EXTREME_TOLERANCE_M = 0.05


def _axes():
    """Cell-centre coordinates, storage convention: row 0 is the smallest y,
    column 0 the smallest x, both ascending (elevation_grid.py)."""
    xs = (np.arange(WINDOW_CELLS, dtype=np.float64) + ORIGIN_IX + 0.5) * RESOLUTION
    ys = (np.arange(WINDOW_CELLS, dtype=np.float64) + ORIGIN_IY + 0.5) * RESOLUTION
    return np.meshgrid(xs, ys)


def cell_of(x: float, y: float) -> tuple:
    """(column, row) of the cell containing the point - the same lattice
    build_occupancy_grid writes."""
    return (int(np.floor(x / RESOLUTION)) - ORIGIN_IX,
            int(np.floor(y / RESOLUTION)) - ORIGIN_IY)


def elevation() -> np.ndarray:
    x, y = _axes()
    z = GROUND_AMPLITUDE_M * np.sin(2 * np.pi * x / GROUND_WAVELENGTH_M) \
        * np.cos(2 * np.pi * y / GROUND_WAVELENGTH_M)

    rng = np.random.default_rng(NOISE_SEED)
    z = z + rng.normal(0.0, NOISE_M, size=z.shape)

    pit = np.hypot(x - PIT_CENTRE[0], y - PIT_CENTRE[1]) <= PIT_RADIUS_M
    z[pit] -= PIT_DEPTH_M

    for (cx, cy), radius, height in BOULDERS:
        z[np.hypot(x - cx, y - cy) <= radius] += height

    wall = ((np.abs(x - WALL_X) <= WALL_HALF_THICKNESS_M)
            & (y >= WALL_Y[0]) & (y <= WALL_Y[1]))
    z[wall] += WALL_HEIGHT_M

    return z.astype(np.float32)


def elevation_from_npz(path: str) -> np.ndarray:
    """A recorded window, when SP12 has one.  Same shape, same convention."""
    grid = np.load(path)['elevation'].astype(np.float32)
    if grid.shape != (WINDOW_CELLS, WINDOW_CELLS):
        raise ValueError(f"expected a {WINDOW_CELLS}^2 window, got {grid.shape}")
    return grid


def seed(grid=None) -> np.ndarray:
    """The int8 cost grid, straight through SP7's own cost curve."""
    _, cost = seed_from_elevation(elevation() if grid is None else grid, RESOLUTION)
    return cost


def occupancy_grid(stamp, grid=None) -> OccupancyGrid:
    from builtin_interfaces.msg import Time
    return build_occupancy_grid(seed(grid), ORIGIN_IX, ORIGIN_IY, RESOLUTION,
                                'map', stamp if stamp is not None else Time())


def clearance_cells(cost, x: float, y: float, radius_m: float) -> int:
    """How many lethal cells lie within radius_m of (x, y).

    Zero is the assertion the planning test makes at every point of the
    path: the rover's inscribed circle never touches a lethal cell.
    """
    from navi_autonomy.traversability import LETHAL
    ix, iy = cell_of(x, y)
    reach = int(np.ceil(radius_m / RESOLUTION))
    y0, y1 = max(iy - reach, 0), min(iy + reach + 1, cost.shape[0])
    x0, x1 = max(ix - reach, 0), min(ix + reach + 1, cost.shape[1])
    patch = cost[y0:y1, x0:x1]
    rows = (np.arange(y0, y1) - iy)[:, None]
    cols = (np.arange(x0, x1) - ix)[None, :]
    inside = (rows ** 2 + cols ** 2) <= reach ** 2
    return int(((patch == LETHAL) & inside).sum())
```

- [ ] Run the test once, read `LETHAL_CELLS` out of the failure message, and put that one number in `fixture.py` in place of the placeholder. `PIT_CELL` is already exact — `cell_of(6.0, 0.0)` is `(600, 480)` by construction (`6.0 / 0.05 = 120`, `ORIGIN_IX = -480`) — so it needs no run to fill in; if it comes out otherwise, the storage convention is wrong and that is the bug, not the constant. Once `LETHAL_CELLS` is committed it is never silently regenerated: a change to it is a change to the fixture terrain and must be argued for in the commit message.
- [ ] Replace the Task-1 placeholder with the real `rover/src/navi_nav2/navi_nav2/fixture_seed_publisher.py`:

```python
"""The fixture seed on the graph, plus - only when asked - the frames and
odometry the ZED would otherwise own.

Two jobs, one node, because they are the same job: standing in for a rover
that has no camera attached.

    ros2 run navi_nav2 fixture_seed_publisher
    ros2 run navi_nav2 fixture_seed_publisher --ros-args -p bench_frames:=true

bench_frames publishes a static map->odom and a 20 Hz odom->base_footprint,
and /localization/odom_local to match.  NEVER run it with the ZED wrapper
up: the wrapper owns map->odom, and base_footprint would get a second
parent - the exact tree split SP6's launch file goes out of its way to
avoid.  It is false by default and start_navi.sh never sets it.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from navi_localization.odom_local import BASE_FRAME, ODOM_FRAME
from navi_nav2 import fixture

SEED_TOPIC = '/autonomy/costmap_seed'
ODOM_TOPIC = '/localization/odom_local'


def latched_qos() -> QoSProfile:
    """Exactly the QoS traversability_layer publishes the seed with; a
    durability mismatch means the costmap gets no data at all."""
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST, depth=1)


class FixtureSeedPublisher(Node):

    def __init__(self):
        super().__init__('fixture_seed_publisher')
        self.declare_parameter('bench_frames', False)
        self.declare_parameter('robot_x', fixture.START[0])
        self.declare_parameter('robot_y', fixture.START[1])
        self.declare_parameter('republish_period_s', 2.0)

        self._publisher = self.create_publisher(
            OccupancyGrid, SEED_TOPIC, latched_qos())
        self._grid = fixture.occupancy_grid(self.get_clock().now().to_msg())
        self._publish_seed()
        # Latched already, but a costmap that starts late and misses the
        # transient_local sample on a busy domain is a five-minute mystery;
        # a slow republish costs nothing.
        self.create_timer(float(self.get_parameter('republish_period_s').value),
                          self._publish_seed)

        if bool(self.get_parameter('bench_frames').value):
            self.get_logger().warn(
                "bench_frames: faking map->odom, odom->base_footprint and "
                f"{ODOM_TOPIC}. Never run this with the ZED wrapper up.")
            self._static = StaticTransformBroadcaster(self)
            self._static.sendTransform(self._identity('map', ODOM_FRAME))
            self._tf = TransformBroadcaster(self)
            self._odom_publisher = self.create_publisher(Odometry, ODOM_TOPIC, 10)
            self.create_timer(0.05, self._publish_pose)

    def _publish_seed(self):
        self._grid.header.stamp = self.get_clock().now().to_msg()
        self._grid.info.map_load_time = self._grid.header.stamp
        self._publisher.publish(self._grid)

    def _identity(self, parent: str, child: str) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.rotation.w = 1.0
        return transform

    def _publish_pose(self):
        x = float(self.get_parameter('robot_x').value)
        y = float(self.get_parameter('robot_y').value)
        stamp = self.get_clock().now().to_msg()

        transform = self._identity(ODOM_FRAME, BASE_FRAME)
        transform.header.stamp = stamp
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        self._tf.sendTransform(transform)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = ODOM_FRAME
        odometry.child_frame_id = BASE_FRAME
        odometry.pose.pose.position.x = x
        odometry.pose.pose.position.y = y
        odometry.pose.pose.orientation.w = 1.0
        self._odom_publisher.publish(odometry)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FixtureSeedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] Run the pure suite. Then a five-second sanity run on domain 92: `ros2 run navi_nav2 fixture_seed_publisher` in one shell, `ros2 topic echo --once /autonomy/costmap_seed --field info` in another; `width`, `height` = 960, `resolution` = 0.05, origin `(-24, -24)`.
- [ ] Commit: `git add rover/src/navi_nav2/navi_nav2/fixture.py rover/src/navi_nav2/navi_nav2/fixture_seed_publisher.py rover/src/navi_nav2/test/test_fixture.py && git commit -m "navi_nav2: the generated elevation fixture, with the pit sitting on the straight line to the goal"`.

---

## Task 6: Rung 3 — Nav2 plans on the fixture, and the path avoids every lethal cell

**Files:** `rover/src/navi_nav2/test/test_offline_planning.py`, `rover/src/navi_nav2/pytest.ini`

This is the plan's centrepiece. It runs headless on the laptop on `ROS_DOMAIN_ID=92`: one `ros2 launch` of the bringup with `perception:=false bench_fixture:=true`, then assertions in five groups. Every one of them exists because it fails for a *different* reason.

**A — bringup.** All six nodes reach `active` within 45 s.
**B — parameter fidelity.** What the running nodes think their parameters are, read with `ros2 param dump --print`. This is the assertion that catches a mistyped key: Nav2 ignores what it does not recognise, so a typo shows up here as the plugin's default and nowhere else.
**C — a plan exists.** `ComputePathToPose` with `planner_id="GridBased"` from `(0, 0)` to `(12, 0)` returns a non-empty path in the `map` frame within 30 s.
**D — the plan is safe.** The path, densified to 0.05 m, never has a sample whose fixture cost is `100` or `-1`, and never has a sample with a lethal cell inside 0.75 m.
**E — the seed carries real cost, not a flattened one.** The path's maximum `|y|` is at least 1.5 m. A stack whose static layer received a *flattened* seed (all free — `trinary_costmap` flipped, a wrong `lethal_cost_threshold`, a later layer overwriting the static one) plans the straight line, passes C, passes D against that empty costmap — and fails this. A stack that received *nothing at all* never gets here: `track_unknown_space: true` on both costmaps plus `allow_unknown: false` on both planners makes an all-`NO_INFORMATION` costmap unplannable, so it fails at **C**, on the start pose.
**F — A/B.** C and D again with `planner_id="SmacBased"`.
**G — the action the operator actually sends.** One `NavigateToPose` goal: accepted, a `/plan` published within 20 s that passes D, at least one `/autonomy_twist` message (the remap chain end to end), then a cancel that is honoured and leaves a zero twist behind.
**H — the single-writer rule.** `/rover_twist` and `/manual_twist` are not on the graph; `/cmd_vel` is not either (everything is remapped); `/autonomy_twist` has exactly one publisher.
**I — the caps hold on the wire.** Every `/autonomy_twist` message: `linear.y == 0.0`, `-0.15 ≤ linear.x ≤ 0.05`, `|angular.z| ≤ 0.1`.

- [ ] Write `rover/src/navi_nav2/pytest.ini`, so the package has its own pytest configuration wherever it is collected from (`deploy_rover.sh --test` runs pytest with the package directory as its rootdir):

```
[pytest]
addopts = -p no:cacheprovider
markers =
    graph: needs a live ROS graph on a throwaway domain
```

- [ ] Write `rover/src/navi_nav2/test/test_offline_planning.py`:

```python
"""Rung 3 of the testing ladder: Nav2 plans on the fixture elevation map.

    ROS_DOMAIN_ID=92 python3 -m pytest \
        rover/src/navi_nav2/test/test_offline_planning.py -q -s

Needs the workspace built and sourced (the launch file, the params and the
fixture node come out of share/).  Brings the whole stack up once for the
module, asserts against it, and kills the process group on the way out.

Domain 92 is a throwaway.  Nothing here publishes /manual_twist or
/rover_twist, and the assertions at the bottom prove neither exists.
"""

import math
import os
import signal
import subprocess
import time

import numpy as np
import pytest
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from navi_autonomy.traversability import LETHAL, UNKNOWN
from navi_nav2 import bringup, fixture

DOMAIN = '92'
INSCRIBED_M = 0.75          # robot_radius 0.80 less the 0.05 m cell it stands in
BRINGUP_TIMEOUT_S = 45.0
PLAN_TIMEOUT_S = 30.0

# deploy_rover.sh --test runs `python3 -m pytest test` in every src/*/ on the
# Orin under `set -eo pipefail`, with no ROS_DOMAIN_ID set.  Without this
# guard every test in this file errors in the stack fixture and the whole
# deploy fails - and if the domain happened to be set, the deploy would
# silently bring a full Nav2 stack up on the rover.  SKIP is the right
# outcome there; the Task 6 and Task 7 commands set the domain explicitly
# and still run it.
pytestmark = pytest.mark.skipif(
    os.environ.get('ROS_DOMAIN_ID') != DOMAIN,
    reason=f"rung 3 needs an explicit ROS_DOMAIN_ID={DOMAIN}; "
           f"run it with the Task 6 command, not from deploy_rover.sh --test")


# --------------------------------------------------------------------------
# the stack, once for the module
# --------------------------------------------------------------------------

@pytest.fixture(scope='module')
def stack():
    assert os.environ.get('ROS_DOMAIN_ID') == DOMAIN, \
        f"run this with ROS_DOMAIN_ID={DOMAIN}; never on domain 0"
    process = subprocess.Popen(
        ['ros2', 'launch', 'navi_nav2', 'nav2_bringup.launch.py',
         'perception:=false', 'bench_fixture:=true'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        start_new_session=True)
    try:
        _wait_for_active(BRINGUP_TIMEOUT_S)
        yield process
    finally:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)


def _lifecycle_state(node_name):
    result = subprocess.run(['ros2', 'lifecycle', 'get', f'/{node_name}'],
                            capture_output=True, text=True)
    return result.stdout.strip()


def _wait_for_active(timeout_s):
    deadline = time.monotonic() + timeout_s
    pending = list(bringup.LIFECYCLE_NODES)
    while pending and time.monotonic() < deadline:
        pending = [n for n in pending if not _lifecycle_state(n).startswith('active')]
        if pending:
            time.sleep(1.0)
    assert not pending, f"not active after {timeout_s} s: {pending}"


@pytest.fixture(scope='module')
def client(stack):
    rclpy.init()
    node = Node('offline_planning_test')
    yield node
    node.destroy_node()
    rclpy.shutdown()


def spin_until(node, predicate, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


# --------------------------------------------------------------------------
# A - bringup
# --------------------------------------------------------------------------

def test_every_lifecycle_node_is_active(stack):
    for name in bringup.LIFECYCLE_NODES:
        assert _lifecycle_state(name).startswith('active'), name


# --------------------------------------------------------------------------
# B - parameter fidelity: what the running nodes actually believe
# --------------------------------------------------------------------------

def running_params(node_name):
    """Nav2 ignores a parameter key it does not know.  A typo in the file
    is therefore invisible everywhere except here, where the node reports
    the plugin default instead of the value we wrote."""
    dumped = subprocess.run(['ros2', 'param', 'dump', '--print', f'/{node_name}'],
                            capture_output=True, text=True, check=True).stdout
    return yaml.safe_load(dumped)[f'/{node_name}']['ros__parameters']


def test_the_planner_runs_theta_star_with_smac_beside_it(stack):
    planner = running_params('planner_server')
    assert planner['planner_plugins'] == ['GridBased', 'SmacBased']
    assert planner['GridBased']['plugin'] == 'nav2_theta_star_planner/ThetaStarPlanner'
    assert planner['SmacBased']['plugin'] == 'nav2_smac_planner/SmacPlanner2D'
    assert planner['GridBased']['allow_unknown'] is False
    assert planner['GridBased']['w_traversal_cost'] == 2.0


def test_the_controller_is_the_shim_and_will_not_reverse(stack):
    controller = running_params('controller_server')
    follow = controller['FollowPath']
    assert follow['plugin'] == 'nav2_rotation_shim_controller::RotationShimController'
    assert (follow['primary_controller']
            == 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController')
    assert follow['allow_reversing'] is False
    assert follow['desired_linear_vel'] == 0.05
    assert controller['odom_topic'] == '/localization/odom_local'


def test_the_speed_caps_reached_the_smoother(stack):
    smoother = running_params('velocity_smoother')
    assert list(smoother['max_velocity']) == [0.05, 0.0, 0.1]
    assert list(smoother['min_velocity']) == [-0.15, 0.0, -0.1]
    assert smoother['odom_topic'] == '/localization/odom_local'


def test_the_costmaps_read_the_seed_at_five_centimetres(stack):
    for node_name in ('global_costmap/global_costmap', 'local_costmap/local_costmap'):
        costmap = running_params(node_name)
        assert costmap['resolution'] == 0.05
        assert costmap['robot_radius'] == 0.80
        assert costmap['track_unknown_space'] is True
        assert costmap['static_layer']['map_topic'] == '/autonomy/costmap_seed'
        assert costmap['static_layer']['trinary_costmap'] is False


def test_the_cloud_layers_are_off(stack):
    assert running_params(
        'global_costmap/global_costmap')['obstacle_layer']['enabled'] is False
    assert running_params(
        'local_costmap/local_costmap')['voxel_layer']['enabled'] is False
    assert running_params('collision_monitor')['points_filtered']['enabled'] is False


def test_nav2s_velocity_leaves_on_autonomy_twist(stack):
    monitor = running_params('collision_monitor')
    assert monitor['cmd_vel_out_topic'] == '/autonomy_twist'
    assert monitor['base_frame_id'] == 'base_footprint'


def test_the_behaviour_tree_that_is_loaded_is_ours(stack):
    path = running_params('bt_navigator')['default_nav_to_pose_bt_xml']
    assert path.endswith('behavior_trees/navigate_to_pose_no_reverse.xml')
    assert os.path.exists(path)


# --------------------------------------------------------------------------
# C, D, E, F - the plan
# --------------------------------------------------------------------------

def pose_stamped(x, y, yaw=0.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def compute_path(node, planner_id):
    action = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
    assert action.wait_for_server(timeout_sec=20.0), "no compute_path_to_pose server"
    goal = ComputePathToPose.Goal()
    goal.start = pose_stamped(*fixture.START)
    goal.goal = pose_stamped(*fixture.GOAL)
    goal.planner_id = planner_id
    goal.use_start = True

    send = action.send_goal_async(goal)
    assert spin_until(node, lambda: send.done(), 15.0), "goal was never sent"
    handle = send.result()
    assert handle.accepted, f"{planner_id} refused the goal"
    result = handle.get_result_async()
    assert spin_until(node, lambda: result.done(), PLAN_TIMEOUT_S), \
        f"{planner_id} produced no result in {PLAN_TIMEOUT_S} s"
    outcome = result.result()
    assert outcome.status == GoalStatus.STATUS_SUCCEEDED, \
        f"{planner_id} failed: status {outcome.status}"
    return outcome.result.path


def densify(path, step_m=0.05):
    """Sample the path at the costmap resolution.  Theta* returns any-angle
    segments; checking only the vertices would let a straight run across a
    pit through untouched."""
    points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    samples = []
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        span = math.hypot(x1 - x0, y1 - y0)
        count = max(int(span / step_m), 1)
        for i in range(count):
            t = i / count
            samples.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
    samples.append(points[-1])
    return samples


def assert_path_is_safe(path, label):
    cost = fixture.seed()
    samples = densify(path)
    assert len(samples) > 50, f"{label}: a path this short is not a 12 m path"
    for x, y in samples:
        ix, iy = fixture.cell_of(x, y)
        assert 0 <= ix < cost.shape[1] and 0 <= iy < cost.shape[0], \
            f"{label}: ({x:.2f}, {y:.2f}) is outside the fixture window"
        value = cost[iy, ix]
        assert value != LETHAL, f"{label}: path crosses a lethal cell at ({x:.2f}, {y:.2f})"
        assert value != UNKNOWN, f"{label}: path crosses unknown ground at ({x:.2f}, {y:.2f})"
        touching = fixture.clearance_cells(cost, x, y, INSCRIBED_M)
        assert touching == 0, (
            f"{label}: {touching} lethal cells within {INSCRIBED_M} m of "
            f"({x:.2f}, {y:.2f}) - the rover's circle would be in the pit")


def test_theta_star_produces_a_path(client):
    path = compute_path(client, 'GridBased')
    assert isinstance(path, Path)
    assert path.header.frame_id == 'map'
    assert len(path.poses) >= 2
    start = path.poses[0].pose.position
    end = path.poses[-1].pose.position
    assert math.hypot(start.x - fixture.START[0], start.y - fixture.START[1]) < 0.5
    assert math.hypot(end.x - fixture.GOAL[0], end.y - fixture.GOAL[1]) < 0.5


def test_theta_stars_path_avoids_every_lethal_cell(client):
    assert_path_is_safe(compute_path(client, 'GridBased'), 'GridBased')


def test_the_path_went_round_the_pit_which_means_the_seed_arrived(client):
    """The assertion that proves the seed carries real cost.

    A stack whose static layer received a *flattened* seed - all free, e.g.
    trinary_costmap flipped to true, or a wrong lethal_cost_threshold, or a
    later layer overwriting the static one - plans the straight line from
    (0,0) to (12,0), which never leaves y = 0, and it passes C and D
    against that empty costmap.  This is the assertion that catches it.

    A stack that received *nothing at all* does not get this far: both
    costmaps set track_unknown_space: true and both planners set
    allow_unknown: false, so an unseeded costmap is entirely
    NO_INFORMATION and Theta*'s isUnsafeToPlan() rejects the start pose -
    the run fails at C.
    """
    path = compute_path(client, 'GridBased')
    excursion = max(abs(p.pose.position.y) for p in path.poses)
    assert excursion >= 1.5, (
        f"the path stayed within {excursion:.2f} m of the straight line: the "
        f"costmap almost certainly never received /autonomy/costmap_seed")


def test_smac_is_a_working_second_opinion(client):
    """Spec section 5: SmacPlanner2D loaded as a second named plugin for
    A/B.  A plugin that is loaded but cannot plan is not an A/B."""
    assert_path_is_safe(compute_path(client, 'SmacBased'), 'SmacBased')


# --------------------------------------------------------------------------
# G, H, I - the action the operator sends, and the single-writer rule
# --------------------------------------------------------------------------

def test_a_navigate_to_pose_goal_is_accepted_planned_and_cancellable(client):
    plans = []
    twists = []
    client.create_subscription(Path, '/plan', plans.append, 10)
    client.create_subscription(Twist, '/autonomy_twist', twists.append, 10)

    action = ActionClient(client, NavigateToPose, 'navigate_to_pose')
    assert action.wait_for_server(timeout_sec=20.0), "no navigate_to_pose server"
    goal = NavigateToPose.Goal()
    goal.pose = pose_stamped(*fixture.GOAL)

    send = action.send_goal_async(goal)
    assert spin_until(client, lambda: send.done(), 15.0)
    handle = send.result()
    assert handle.accepted, "bt_navigator refused the goal"

    assert spin_until(client, lambda: len(plans) > 0, 20.0), \
        "no /plan published for a NavigateToPose goal"
    assert_path_is_safe(plans[-1], '/plan')

    assert spin_until(client, lambda: len(twists) > 0, 20.0), \
        "nothing reached /autonomy_twist: the velocity chain is broken"

    cancel = handle.cancel_goal_async()
    assert spin_until(client, lambda: cancel.done(), 10.0), "cancel was never answered"

    twists.clear()
    spin_until(client, lambda: False, 3.0)
    if twists:
        assert twists[-1].linear.x == 0.0 and twists[-1].angular.z == 0.0, \
            "a cancelled goal must leave a zero twist behind"


def test_the_caps_hold_on_the_wire(client):
    """Whatever the controller asked for, this is what left the stack."""
    twists = []
    client.create_subscription(Twist, '/autonomy_twist', twists.append, 10)

    action = ActionClient(client, NavigateToPose, 'navigate_to_pose')
    assert action.wait_for_server(timeout_sec=20.0)
    goal = NavigateToPose.Goal()
    goal.pose = pose_stamped(*fixture.GOAL)
    send = action.send_goal_async(goal)
    assert spin_until(client, lambda: send.done(), 15.0)
    handle = send.result()
    spin_until(client, lambda: len(twists) >= 5, 20.0)
    handle.cancel_goal_async()
    spin_until(client, lambda: False, 2.0)

    assert twists, "no velocity was produced at all"
    for twist in twists:
        assert twist.linear.y == 0.0, "vy is pinned to zero at the smoother"
        assert -0.15 <= twist.linear.x <= 0.05, f"vx {twist.linear.x} outside the cap"
        assert abs(twist.angular.z) <= 0.1 + 1e-9, f"wz {twist.angular.z} outside the cap"


def test_nothing_in_this_stack_can_reach_the_chassis(stack):
    topics = subprocess.run(['ros2', 'topic', 'list'],
                            capture_output=True, text=True, check=True).stdout.split()
    assert '/rover_twist' not in topics, "only mode_supervisor may publish /rover_twist"
    assert '/manual_twist' not in topics, "nothing may publish /manual_twist, ever"
    assert '/cmd_vel' not in topics, \
        "Nav2's default velocity name is unremapped somewhere; it must be cmd_vel_nav"
    assert '/autonomy_twist' in topics


def test_autonomy_twist_has_exactly_one_publisher(client):
    publishers = client.get_publishers_info_by_topic('/autonomy_twist')
    assert len(publishers) == 1, \
        f"{[p.node_name for p in publishers]} - only the collision monitor may write it"
    assert publishers[0].node_name == 'collision_monitor'
```

- [ ] Run it. **Expected failure modes and what each one means** (fix the cause, never weaken the assertion):
  - *No lifecycle node becomes active* — read the launch output; a `pluginlib` class-loader error is a plugin-name typo, a YAML parse error is an indentation slip in the params file.
  - *`test_the_path_went_round_the_pit` fails with a small excursion* — the seed arrived but was **flattened to free**: `trinary_costmap` flipped to `true` (which collapses SP7's 0–99 band), a wrong `lethal_cost_threshold`, or a later layer overwriting the static one. A static layer that received *nothing at all* does not fail here — it fails at C first (next item). Check `trinary_costmap: false`, `lethal_cost_threshold: 100`, and the layer order in both costmaps.
  - *`ComputePathToPose` fails immediately with an unplannable start* — the seed never reached the static layer at all. With `track_unknown_space: true` on both costmaps and `allow_unknown: false` on both planners, an unseeded costmap is entirely `NO_INFORMATION` and Theta\*'s `isUnsafeToPlan()` rejects the start pose outright, so the run dies at C and never reaches D or E. Check `ros2 topic info -v /autonomy/costmap_seed` shows two endpoints, both `TRANSIENT_LOCAL`, and that the fixture node is up.
  - *`ComputePathToPose` times out or fails* — the start or goal is inside inflation (Task 5's fixture test guards that), or the global costmap has no TF (`bench_fixture:=true` must be set), or `allow_unknown: false` plus an unknown patch has walled the goal in.
  - *`test_smac_is_a_working_second_opinion` times out on the Orin (Task 7, `timeout 60`)* — `SmacBased.max_planning_time: 5.0` over a 960 × 960 grid with `downsample_costmap: false` is comfortable on the laptop and may be tight on arm64. Raise `max_planning_time`; never weaken the assertion.
  - *`assert_path_is_safe` fails on clearance by a cell or two* — `INSCRIBED_M = 0.75` is 15 cells against an inflation that marks ≈ 0.81 m (16.2 cells) as `253`, so the margin is real but about 1.2 cells, and `clearance_cells()` measures from the sample's *containing* cell (up to a 0.035 m half-diagonal of slop) while Theta\*'s Bresenham line-of-sight can corner-cut between two blocked cells. If it flakes, lower `INSCRIBED_M` to 0.70 — do not touch the planner or the inflation.
  - *No `/autonomy_twist`* — the chain is broken: `ros2 topic info -v cmd_vel_nav` and `cmd_vel_smoothed` should each show one publisher and one subscriber.
  - *`test_the_caps_hold_on_the_wire` sees `wz > 0.1`* — the smoother's `max_velocity` did not reach the node; test B would normally have caught it, so suspect a second params file.
- [ ] Also run the pure suite and the supervisor suite; nothing here may have broken them.
- [ ] Prove the deploy path is safe: with **no** `ROS_DOMAIN_ID` in the environment, `bash -c 'source /opt/ros/humble/setup.bash && cd rover/src/navi_nav2 && python3 -m pytest test -q'` must report the offline-planning tests as **skipped**, not errored — that is what `deploy_rover.sh --test` does on the Orin, under `set -eo pipefail`, and an error there fails the whole deploy.
- [ ] Commit: `git add rover/src/navi_nav2/test/test_offline_planning.py rover/src/navi_nav2/pytest.ini && git commit -m "Rung 3: Nav2 plans on the fixture map and the path never comes within the rover's radius of a lethal cell"`.

---

## Task 7: `start_navi.sh`, the Orin, and what the next camera session must do

**Files:** `rover/start_navi.sh`, `rover/src/navi_nav2/launch/nav2_bringup.launch.py` (the measurement block in the docstring)

- [ ] Edit `rover/start_navi.sh`. Header, after the item 6 block:

```
#   7. nav2_bringup.launch.py - Nav2: Theta*/RPP planning to /autonomy_twist,
#      which only mode_supervisor reads. Nav2 needs the frames and the
#      odometry localisation publishes, so it is skipped when --no-localization
#      is given.
```

and in the flag list:

```
#   ./start_navi.sh --no-nav2    no Nav2 (nothing plans; manual drive is unaffected)
```

Then `START_NAV2=1` beside the other flags, `--no-nav2) START_NAV2=0; shift ;;` in the `case`, and — **after** the `wait_for_localization` block and **before** `START_VIDEO`, so Nav2 starts on a graph that already has TF:

```bash
if [ "$START_NAV2" -eq 1 ]; then
    if [ "$START_LOCALIZATION" -eq 0 ]; then
        # Nav2's costmaps need map->odom->base_footprint, and the ZED
        # wrapper is the only thing that publishes it. Starting Nav2
        # without it produces a node that logs a TF timeout twice a second
        # forever and plans nothing.
        echo "skipping nav2: it needs localisation for TF (--no-localization was given)"
    else
        echo "starting nav2 (plans to /autonomy_twist; only mode_supervisor drives)"
        ros2 launch navi_nav2 nav2_bringup.launch.py &
        BACKGROUND_PIDS+=("$!")
    fi
fi
```

and extend the stale-process cleanup, next to the existing `kill_stale` calls:

```bash
    kill_stale "nav2 launches" "ros2 launch navi_nav2"
```

- [ ] Run the launcher's own gate test, which must still pass: `bash rover/test/test_start_navi_gate.sh`.
- [ ] Deploy and build on the Orin: `./deploy_rover.sh`.
- [ ] **Re-verify on the Orin that the Nav2 stack is actually there, before anything is launched.** The environment block's numbers were taken on 2026-08-31 and a re-flashed Orin will not have kept them; a missing plugin package otherwise surfaces as a pluginlib class-loader error tens of seconds into the configure transition, buried in `/tmp/nav2_bench.log`. Run Task 1's test on the Orin itself and record what it printed:

```
ssh star@a_navi 'source /opt/ros/humble/setup.bash && cd ~/navi/src/navi_nav2 &&
  python3 -m pytest test/test_nav2_available.py -q'
ssh star@a_navi 'dpkg -l | grep -c "ros-humble-nav2\|ros-humble-navigation2"'
```

  Expected: 15 passed (14 parametrised package checks plus the pluginlib-XML check) and `31`. If the count differs from the laptop's, `comm` the two `dpkg -l` lists before going further and use the deb-carry runbook below. Do **not** proceed to the smoke launch on a failure here.

- [ ] Then the camera-less smoke test, which is the whole of what tonight can prove:

```
ssh star@a_navi 'source /opt/ros/humble/setup.bash && cd ~/navi && source install/local_setup.bash &&
  ROS_DOMAIN_ID=92 nohup ros2 launch navi_nav2 nav2_bringup.launch.py \
      perception:=false bench_fixture:=true > /tmp/nav2_bench.log 2>&1 &'
sleep 45
ssh star@a_navi 'source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=92 &&
  for n in controller_server planner_server behavior_server bt_navigator velocity_smoother collision_monitor; do
      printf "%s " "$n"; ros2 lifecycle get /$n; done'
ssh star@a_navi 'source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=92 &&
  ros2 topic list | grep -E "autonomy_twist|rover_twist|manual_twist|cmd_vel"'
```

- [ ] Measure, on the Orin, with the fixture seed driving two 0.05 m costmaps (this is §11 risk 6, as far as a camera-less rover can answer it):

```
ssh star@a_navi 'top -b -n 3 -d 2 | grep -E "controller_serv|planner_server|collision_moni|bt_navigator" '
ssh star@a_navi 'ps -o rss=,comm= -C controller_server,planner_server,bt_navigator | sort -n'
```

Write the numbers into the measurement block in `nav2_bringup.launch.py`'s docstring (`RECORD HERE:`), in the style `localization.launch.py` already uses — machine, date, power mode, what was running. If the total is above roughly a core and a half, note it and leave the documented fallback (local 24 m at 0.05 m, global 48 m at 0.10 m) in place for SP10 to take up.

- [ ] Then a planning request against the canned costmap on the Orin itself, so "it plans there too" is a fact and not an inference:

```
ssh star@a_navi 'source /opt/ros/humble/setup.bash && cd ~/navi && source install/local_setup.bash &&
  ROS_DOMAIN_ID=92 timeout 60 python3 -m pytest src/navi_nav2/test/test_offline_planning.py -q -p no:cacheprovider 2>&1 | tail -20'
```

(The stack is already up from the smoke test; kill it first — `ssh star@a_navi 'pkill -x controller_server; pkill -x planner_server; pkill -x behavior_server; pkill -x bt_navigator; pkill -x velocity_smoother; pkill -x collision_monitor; pkill -x lifecycle_manager'` — `pkill -x` only, never `-f`.)

- [ ] Finally, tear down cleanly and confirm nothing is left on domain 92.

**The deb-carry runbook** — not needed today (both machines already have identical `navigation2` 1.1.20 sets; verified 2026-08-31), written down because the next missing package will not announce itself. Put it in the launch file docstring beneath the MUST-DO list:

```
# Carrying a package to the Orin (no internet there, arm64, jammy):
#
#   # 1. the Orin computes its own URIs - right architecture, right
#   #    versions, only what it is actually missing:
#   ssh star@a_navi 'apt-get install --print-uris -y --no-install-recommends \
#       ros-humble-<pkg> | grep -oP "(?<=^.)http[^\x27]+" > /tmp/uris.txt; wc -l < /tmp/uris.txt'
#   scp star@a_navi:/tmp/uris.txt /tmp/uris.txt
#   # 2. the laptop, which has internet, does the downloading:
#   wget -x -P ~/orin-debs -i /tmp/uris.txt
#   # 3. carry and install:
#   rsync -a ~/orin-debs/ star@a_navi:~/orin-debs/
#   ssh star@a_navi 'sudo dpkg -i $(find ~/orin-debs -name "*.deb")'
#
# Do NOT apt-get download on the laptop: it is amd64 and the Orin is arm64,
# and the deb would install and then fail to load.
```

**MUST-DO at the next camera session** (already written into the launch file docstring in Task 3; repeated here so the plan carries it too):

1. Turn on the three cloud booleans once `cloud_filter` publishes `/autonomy/points_filtered`, and re-run the offline planning test to prove nothing else changed.
2. Verify the seed aligns with real terrain — a misaligned seed puts lethal cells metres from real rocks and looks entirely plausible (§11 risk 7).
3. Re-measure Orin CPU with perception live and compare with the camera-less numbers recorded above.
4. Watch the RotationShim → RPP handover on the real chassis; measure the steering slew before any speed stage is raised (§10, §11 risk 3).
5. Check the collision polygons against the real footprint with the rover on blocks (§11 risk 8).

- [ ] Commit: `git add rover/start_navi.sh rover/src/navi_nav2/launch/nav2_bringup.launch.py && git commit -m "start_navi.sh brings Nav2 up with localisation, and the Orin bench numbers are written down"`.

---

## Done when

- Six lifecycle nodes come up from one launch file and all reach `active`, on the laptop and on the Orin.
- `test_offline_planning.py` is green on domain 92: Theta\* and SmacPlanner2D both produce a path from `(0, 0)` to `(12, 0)` that never brings the rover's 0.75 m circle into contact with a lethal cell, and that leaves the straight line by at least 1.5 m — which is the proof that the seed reached the costmap carrying real cost rather than flattened to free. (A seed that never arrived at all fails earlier, at `ComputePathToPose`.)
- The same file **skips**, rather than errors, when `ROS_DOMAIN_ID` is not 92 — so `deploy_rover.sh --test` on the Orin passes instead of failing the deploy under `set -eo pipefail`, and never brings a Nav2 stack up on the rover by accident.
- The fixture is pinned structurally — lethal-cell count, pit cell, elevation extremes — so `test_fixture.py` is green on the Orin's arm64 as well as the laptop's amd64.
- Task 1's `test_nav2_available.py` has been run **on the Orin** and its output recorded, before anything was launched there.
- `mode_supervisor` cancels every Nav2 goal and pauses the stack on takeover, through two `call_async` service calls that never wait, with the SP5 action sequence unchanged.
- `/autonomy_twist` has exactly one publisher; `/rover_twist`, `/manual_twist` and `/cmd_vel` do not exist on the test domain.
- `start_navi.sh --no-nav2` works, and Nav2 is skipped rather than started blind when localisation is not running.
- The Orin's camera-less CPU and RAM numbers are written into the launch file, and the five camera-session MUST-DOs are recorded there too.
