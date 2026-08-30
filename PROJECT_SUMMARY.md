# Asterope rover ground station — project goal and state

Updated 2026-08-30 from the live repository (`~/star/Navi`, `master`).

## The goal

**The Gazebo view in the ground station must reflect the real world as
faithfully as the rover's sensors allow.** An operator at the base-station
laptop, in semi-autonomous mode, sees the real rover placed by its own
localisation inside a live 3-D reconstruction of the yard — the ground it
has driven on and the objects around it — built from the ZED 2i as the rover
moves, with no camera video. What the operator sees in Gazebo *is* what the
rover has seen: nothing invented, nothing hidden that the sensors captured.
Everything else in this repository serves that: localisation places the
rover, the map pipeline builds the world, the ground station shows it and
lets the operator drive, save and reload.

Context: the Asterope rover of STAR Dresden for the European Rover
Challenge Mars yard (≈ 35 × 35 m, tours ≤ 250 m). Jetson Orin (25 W mode)
with a ZED 2i on the rover; a laptop as ground station.

## Operating modes and what "done" means

| mode | state | definition of done |
|---|---|---|
| **Manual** | done | gamepad drive over rosbridge, rover video (15 fps) in the panel |
| **Semi-autonomous** | working, being refined | Gazebo shows the rover at its localised pose on the live terrain with obstacles as 3-D blocks; no video; MAP row: save / load / clear maps stored on the rover |
| **Simulation** | done | the old dead-reckoning sim with the organisers' scan, for the laptop alone |
| **Autonomous** | designed, deliberately not built | Nav2 goal navigation (specs/plans of 2026-08-29-autonomy*) |

Quality bar for the reconstruction, in order: (1) geometry at the right
place and size (5 cm ground cells, 10 cm obstacle voxels, tiles of 2.5 m);
(2) completeness — everything the ZED has fused appears, holes close as the
rover keeps looking; (3) no artefacts — no spikes, no stale objects the
sensor no longer sees, no flash on update; (4) texture/colour last (an
after-ride textured export via the ZED's `save_3d_map` is the planned
next step).

## Constraints that shape every decision

- ZED 2i is the only localisation sensor: visual-inertial tracking + area
  memory; **the magnetometer is never used**; there is no wheel odometry.
- Rover code runs on the Orin (`rover/`, deployed with `deploy_rover.sh`);
  the ground station has **no ROS** (PySide6 + roslibpy over rosbridge);
  the simulation runs on its own ROS domain (42) behind a one-way bridge.
- Maps live on the rover (`~/navi_maps/<name>.npz`), saved only on request.
- Never publish to `/manual_twist` from tests (it drives the physical
  rover); `sim/src/navi_sim_ik/vendor/` is read-only.
- WiFi is shared with video: map traffic is tiles and deltas, tens of KB/s.

## Where things are

- Designs: `docs/superpowers/specs/` — localisation (08-29), tiled map
  (08-29), obstacle voxels (08-30), autonomy (08-29, not executed).
- Plans and execution ledgers: `docs/superpowers/plans/`, `.superpowers/sdd/`.
- Rover packages: `rover/src/navi_localization` (pose, status, elevation
  grid, tiles, voxels, mapper), `rover/src/navi_teleop` (video).
- Simulation: `sim/src/navi_sim_bringup` (bridge, terrain/obstacle writer,
  meshes), `sim/src/navi_sim_ik`, `sim/src/navi_sim_video`.
- Ground station: `ground_station/` with tests in `tests/`.
- Launchers: `start_navi.sh` (rover), `start_sim.sh --mode semi`,
  `start_ground_station.sh`; Claude Code helpers in `.claude/`
  (`/deploy-rover`, `/sim-e2e`, `ros-reviewer`).

## Open items (2026-08-30)

- Holes in obstacle surfaces while the rover is static (fill pass on the
  laptop or finer ZED mapping resolution).
- Stale objects (a person who walked through) stay until the ZED re-fuses
  that chunk — no free-space decay yet.
- Textured after-ride export (tier 2); overwrite of a saved map from the UI;
  review of the last obstacle fix (commit e4fbbd5) still to be done.

---

# History — planning notes of 2026-08-25 (superseded; kept for context)

# Mars Rover Navigation — Planning & Development Summary

Written 2026-08-25. **Environment note**: this file was written from a session that does
NOT have live filesystem access to the actual project repo (expected at `/home/ole/star`
on the machine where the work happened — see "Where the code actually lives" at the
bottom). Everything below reflects the project's state as of the end of the last working
session, reconstructed from conversation history, not a fresh verification against the
live repo. Treat it as a planning/handoff document, not a source of truth for exact file
contents — re-verify against the real repo before acting on specifics.

## 1. Project in one paragraph

Building a navigation stack for a real Mars-analog rover ("Asterope") for the European
Rover Challenge (ERC): Jetson Orin Nano (Ubuntu 22.04.5, ROS 2 Humble) running dual ZED2i
cameras, a holonomic 4-wheel independent-steer+drive chassis (the real hardware has its
own self-written inverse kinematics — not touched by this repo). A ground station with
Manual / Semi-Autonomy / Full-Autonomy modes and a strict fallback hierarchy sits on top.
Development happens in a Docker-based ROS 2 Humble container on a WSL2 host, deliberately
matching the Jetson's environment rather than the host's native (newer) ROS 2/Ubuntu.

Full roadmap, from the original brainstorming:

1. Mesh/simulation tooling — **done**
2. Holonomic drivetrain + real-asset simulation — **done** (this session closed out the
   remaining blockers)
3. Localization (AMCL) — **not started**
4. Mode supervisor / teleop / ground station — **not started**
5. Nav2 navigation — **not started**
6. Integration / fallback drills — **not started**

## 2. What's actually built

### Phase 1 — Mesh mapping tools & simulation environment (complete, merged)

Package `rover_mapping_tools`: converts a 3D mesh into a Nav2 occupancy grid
(`mesh_to_map` CLI) or a Gazebo Classic world (`mesh_to_world` CLI). Core modules:
`mesh_loader.py` (load + rasterize meshes into an occupancy grid), `occupancy_grid_writer.py`
(writes `.pgm`+`.yaml` in Nav2's convention), `gazebo_world_writer.py` (writes SDF world
files), `cli.py`, `sample_room.py` (synthetic test fixture, still used for fast/simple
tests independent of the real terrain).

Package `rover_description` / `rover_simulation`: originally a placeholder box-chassis
diff-drive rover in a synthetic room, later fully replaced (see Phase 1.5 below).

All 7 tasks individually implemented and code-reviewed via Subagent-Driven Development,
with a final whole-branch review that caught and fixed 3 critical + 3 important bugs
(rover falling through the floor, `/odom` never publishing, chassis pitching due to a
missing 3rd contact point, `use_sim_time` missing, plus two rasterizer bugs in
`mesh_loader.py`'s z-band/degenerate-triangle handling) — all fixed with real headless
Gazebo verification evidence, not just code inspection.

### Phase 1.5 — Holonomic drivetrain & real asset integration (complete, including
follow-up fixes done in the most recent session)

**Real rover URDF** (`rover_description/urdf/rover.urdf.xacro`): replaced the placeholder
with the real Asterope geometry (chassis, camera mounts, etc., sourced from the
hardware team's `asterope_iiI.urdf`), real dual-ZED2i camera frames matching
`zed-ros2-wrapper` conventions, and 4 independently-steered + independently-driven wheels
(`front_left`/`front_right`/`rear_left`/`rear_right`, each a `continuous` steer joint
(axis Z) + `continuous` drive joint (axis Y), positions `(±0.455, ±0.455)`, wheel radius
`0.155`). Wired to `gazebo_ros2_control/GazeboSystem` via a `<ros2_control>` block;
`rover_description/config/ros2_control.yaml` defines `joint_state_broadcaster` +
`steer_controller` (position) + `drive_controller` (velocity).

**Swerve-drive IK (simulation-only)**: `rover_simulation/swerve_kinematics.py` — pure
Python, no ROS imports, computes per-wheel steer angle + drive velocity from
`(vx, vy, wz)`, including a reverse-drive optimization (if a wheel's target angle is
>90° from its previous commanded angle, flip 180° and reverse drive direction instead of
steering the long way around). `rover_simulation/swerve_ik_node.py` is a thin `rclpy`
wrapper: subscribes `/cmd_vel`, publishes `/steer_controller/commands` +
`/drive_controller/commands`. **This IK has no bearing on the real rover**, which has its
own self-written IK subsystem — it exists purely to drive the simulated `ros2_control`
interfaces from a `Twist` for testing.

**Real Mars yard terrain**: the real scanned mesh (provided as `.ply`/`.obj`/`.fbx`,
~1.68M faces, ~37m×44m) is used via `rover_simulation/meshes/mars_yard.ply` (the single
checked-in source-of-truth asset) and a generated `rover_simulation/worlds/mars_yard.world`.
Spawn pose is a **hand-overridable launch argument**
(`spawn_x`/`spawn_y`/`spawn_z` on `sim_bringup.launch.py`), not hardcoded — defaults tuned
to the flattest nearby spot on the real mesh (`0.0, 5.0, 0.29`), found by raycasting
against the real mesh data.

**Real, hard-won bug fixes** (all found via actually running the stack in headless
Gazebo, not just code review — this was a deliberate lesson carried from Phase 1's final
review, where an unexecuted "manual verification" step had hidden 3 real bugs):

1. *Terrain collision*: the rover initially free-fell through the terrain indefinitely.
   The original hypothesis ("Gazebo Classic/ODE can't handle a 1.68M-face trimesh") was
   **wrong** — isolated via a trivial 2-triangle test mesh that Gazebo's ODE physics
   simply can't build usable collision geometry from a `.ply` file at all, regardless of
   size. Fix: a new `mesh_decimate` CLI tool (in `rover_mapping_tools`, using `pyfqmr` for
   lightweight quadric decimation) builds a ~20k-face collision-only mesh exported as
   `.obj`; the full-res mesh stays `.ply` for visuals. `gazebo_world_writer.write_gazebo_world`
   gained an optional `collision_mesh_path` parameter for this split.
2. `controller_manager` never starting: `gazebo_ros2_control` injects the full URDF into
   its embedded controller node as a `--param robot_description:=<xml>` CLI-style
   parameter override, and rcl's parser choked on it. Root cause (again, isolated by
   trial rather than assumed): the URDF's XML comments (long `--`-heavy banner lines)
   broke rcl's parser — not the URDF's raw size. Fix: a `_strip_comments` helper in
   `sim_bringup.launch.py` strips comments from *only* the in-memory string handed to
   that one parameter; the source `.xacro`/`.urdf` files on disk are untouched.
3. *Terrain not rendering in the GUI* (found and fixed in the most recent session): three
   separate, stacked bugs, each confirmed individually —
   - Gazebo's OGRE-based renderer needs a mesh's directory explicitly registered as a
     resource path (`GAZEBO_RESOURCE_PATH`); it does this automatically for `model://`
     URIs but not for the raw absolute `file://` paths this project's generated worlds
     use.
   - `gzclient` (the GUI window) and `gzserver` were being started simultaneously by the
     stock `gazebo_ros` launch file; `gzclient` gives up and exits silently within ~4s if
     `gzserver` isn't already accepting connections, and this project's `gzserver` —
     loading a large real terrain mesh plus the whole `ros2_control` stack — reliably
     takes longer than that. Fixed by starting `gzclient` ~15s after `gzserver`, not
     simultaneously.
   - Even with both of the above fixed, Gazebo's renderer turned out to be **completely
     unable to parse `.ply`** for rendering either (a separate bug from the collision
     one, same underlying pattern) — confirmed via a real OGRE exception
     ("Header chunk didn't match either endian"). Fixed with a new `mesh_convert` CLI
     (full-resolution re-export in a different format, preserving the mesh's real
     per-vertex color data) that builds a full-res `.obj` copy of the terrain for
     rendering only. This `.obj` (~168MB) is regenerated on demand, not committed to
     git — only the original `.ply` (~44.5MB) is checked in.

All three of the rendering fixes were verified for real: `gzclient` observed staying
alive under sustained heavy render load with zero exceptions in Gazebo's own logs, while
collision/controllers remained correct (rover still settles properly, all 3 controllers
active).

**Interactive control set up this session** (not part of the original phase plan, added
because it's needed to actually test-drive the sim):
- Keyboard teleop via `ros-humble-teleop-twist-keyboard` (works out of the box, no
  external hardware setup).
- Physical Xbox-controller teleop via `ros-humble-teleop-twist-joy` + `joy`, with a
  holonomic axis mapping (left stick = translate, right stick = rotate, matching the
  ground station design's convention) in
  `rover_simulation/config/teleop_twist_joy_holonomic.yaml`. Verified the ROS side (node
  loads the config, reports the intended mapping) but **not** end-to-end against real
  hardware — see "Known open item" below.

**Test coverage**: 47 automated tests across the three packages (`rover_mapping_tools`,
`rover_description`, `rover_simulation`), all passing as of the last verified run.

## 3. Known open item — Xbox controller WSL2 passthrough (unresolved)

The controller needs `usbipd-win` (on the Windows host) to attach to WSL2 before it's
usable at all. As of the end of the last session, repeated `lsusb`/`/proc/bus/input/devices`
checks from inside WSL2 showed the controller was **not actually attached**, despite the
user believing the Windows-side attach had been done. The kernel log showed one attach
attempt that got partway through USB enumeration and then disconnected — consistent with
the Windows-side `usbipd attach --wsl --busid <BUSID>` command's PowerShell window having
been closed (that command only stays connected as long as its own window keeps running,
unless `--auto-attach` is used). This was never confirmed working end-to-end. Next step:
re-run the attach step and keep the window open (or use `--auto-attach`), then re-verify
`/dev/input/js0` appears — a real, documented WSL2/`xpad`-driver rough edge was also
flagged as a possibility if the device attaches (`lsusb` sees it) but never gets a
working driver binding.

## 4. Not yet started (per the original roadmap)

- **Phase 3 — Localization (AMCL)** against the real Mars yard map. Note:
  `mesh_to_map`'s occupancy-grid rasterizer is known to be too slow for the full
  1.68M-face real mesh (a separate, explicitly deferred sub-project from Phase 1 — not
  addressed by any of the fixes above, which were all about the Gazebo *simulation* path,
  not the Nav2 *map generation* path). This will likely need to be resolved before or
  during the localization phase.
- **Phase 4 — Mode supervisor / teleop / ground station**: Manual / Semi-Autonomy /
  Full-Autonomy modes with a strict fallback hierarchy, per the original design spec —
  not designed in detail yet, only scoped at a high level in the original brainstorming.
- **Phase 5 — Nav2 navigation** (path planning/following) on top of the localization
  layer.
- **Phase 6 — Integration & fallback drills** — end-to-end testing of the mode-switching
  and fallback behavior.

## 5. Other open decisions (not urgent, but unresolved)

- **Branch integration**: all of Phase 1 + Phase 1.5's work has been living on a
  worktree branch (`worktree-mesh-mapping-and-sim-environment`, forked from `master`).
  Whether/when to merge, PR, or keep developing on it was never explicitly decided — it
  was raised once after Phase 1 and never revisited given the sustained Phase 1.5 work.
- A lightweight final review of the follow-up fixes (terrain collision, controller_manager,
  rendering) was suggested but not done — those fixes were implemented directly rather
  than through the project's usual task-brief-plus-independent-review cycle, given they
  were bug fixes to already-reviewed code rather than new scope.

## 6. Where the code actually lives

Per the last known session state (**not verified live from this session** — see the note
at the top of this document):

- Worktree: `/home/ole/star/.claude/worktrees/mesh-mapping-and-sim-environment`
- Branch: `worktree-mesh-mapping-and-sim-environment`, forked from `master` in the main
  repo at `/home/ole/star`
- Key docs inside that repo: `docs/superpowers/specs/2026-08-19-mars-rover-nav-design.md`
  (whole-project design), `docs/superpowers/specs/2026-08-20-holonomic-drivetrain-and-real-assets-design.md`,
  `docs/superpowers/plans/2026-08-19-mesh-mapping-and-sim-environment.md`,
  `docs/superpowers/plans/2026-08-20-holonomic-drivetrain-and-real-assets.md`,
  `.superpowers/sdd/2026-08-20-holonomic-drivetrain-and-real-assets/progress.md` (the
  detailed task-by-task SDD ledger — git-ignored, local to that machine only), and a
  `HANDOFF.md` at the repo root (written specifically for picking the project back up in
  a fresh session).
- Dev environment: a Docker image `rover-dev:humble` (ROS 2 Humble + Gazebo Classic 11 +
  `ros2_control` + the various pip packages this project needs — `trimesh`, `pyfqmr`,
  etc.), local to whatever machine that worktree lives on, **not committed to git**.
  Convenience scripts `run_simulation.sh`, `drive_teleop.sh`, `drive_teleop_joystick.sh`
  at the repo root.

**This machine/session doesn't have any of the above** — `/home/ole/star` doesn't exist
here. If you're planning to continue this work from here, you'll need to either get back
to the original machine/environment, or re-clone/re-establish the project (git history,
if pushed anywhere, would be the fastest path to recovering it) — this directory
(`/home/ole/navi/navigation`) only has the four original raw asset files
(`Model3D_mesh.fbx`, `Model3D_mesh1.ply`, `Model3D_mesh2.obj`, `asterope_iiI.urdf`) that
this whole project was originally built from.

## 7. The BEMA drive bridge (2026-08-30, branch `bema-bridge`)

The ground station's gamepad twist now has a path to the real wheels. A new
node `bema_bridge` (in `navi_teleop`, Python + the `msgpack` package) runs on
the Orin, subscribes `/manual_twist`, and speaks msgpack-RPC to the primary
Pi (`192.168.178.26`): the BEMA drive server on :21022 (`F1(vX, vY, w_degps)`
at 20 Hz under the exclusive-access lease, `__sam__ping` every 0.5 s) and the
mission coordinator on :21031 (`notifyConnected` heartbeat every 1 s —
without it the coordinator drops to Disconnected within 2 s and disables
movement; `startManual` arms driving after a 5 s PrepareManual).

Safety: a **1 s deadman** — when `/manual_twist` stops, the node sends
`F1(0,0,0)` then `F2 stopMovement` once and keeps streaming zeros; there is
NO deadman on the Pi side (the lease watchdog expiring does not stop the
wheels), so this node is the stop reflex. Start-up sends nothing that moves
wheels; `F0 init` and `F4 resetEncoders` (both physically move the wheels)
fire only from explicit GS buttons behind confirm dialogs. The node never
calls `__sam__force`.

GS side: a DRIVE row (semi-auto mode, beside the MAP row) with STOP always
live, Manual ("arming (5 s)"), Init/Reset-encoders behind confirms, and a
status line from `/drive_status` (1 Hz JSON: connected, lease, coordinator
state name, deadman, twist age, last error). `/drive_command` carries the
button actions as JSON. The old `DriveState` tracker class was renamed
`DriveCommandTracker`; the new `/drive_status` dataclass owns the name.

Config: `bema_host`/`bema_port`/`coordinator_port`/`deadman_s`/`twist_topic`
are node parameters (addresses have churned before — coordinator still dials
a NaVi at .18 that doesn't exist; `a_navi` is .33). `start_navi.sh` starts
the bridge (`--no-bridge` to skip). The Orin has no internet: `msgpack`
1.2.2 was installed from a wheel carried over scp.

Verified: 240 GS + 65 teleop tests locally; 171 + 65 on the Orin after
`deploy_rover.sh --test`; a headless bench on domain 93 (fake BEMA server
`test/fake_bema_server.py --forward-twist`, bridge on `/bench_twist`) showed
the full chain forward 0.2 m/s / 0.5 rad/s with the w double-negation
self-consistent, and zeros after the publisher stopped. **Still open:** the
absolute turn direction and the wheel-corner mapping are only checkable on
hardware — rover on blocks, hand on STOP, before any free driving. The
coordinator-driven autonomy path (NaVi RPC server, `setTargets`) is
deliberately not built.
