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
