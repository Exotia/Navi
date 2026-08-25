# Mars Rover Navigation — Rebuild Design

Written 2026-08-25. This supersedes `PROJECT_SUMMARY.md` as the live plan: the
old repo (`/home/ole/star`) is not available from here, so this is a full
rebuild in `/home/ole/navi/navigation`, using the old summary only as
historical context (approach, hard-won bug fixes), not as code to resume.

## Target environment

- Jetson Orin Nano, native Ubuntu 22.04.5, ROS 2 Humble. **No Docker** —
  development and simulation happen directly on the Jetson.
- Dual ZED2i cameras, 4-wheel independent-steer+drive holonomic chassis.
  Real inverse kinematics is owned by the hardware team and will be provided
  later; until then, drive nodes just consume `/cmd_vel` (Twist) directly.

## Build order

1. **Ground station** — built first, before other nodes exist, so every
   later node is verifiable the moment it's built.
2. Rover description + drive (`/cmd_vel` passthrough; real IK wired in once
   provided).
3. Localization.
4. Mode supervisor.
5. Nav2 navigation.
6. Integration / fallback drills.

Gazebo simulation is **not a scheduled phase** — only brought in ad hoc if a
specific test genuinely can't be done on real hardware.

## Ground station

Cross-platform desktop app (Python + Qt), talking to the rover over
`rosbridge_suite` (websocket + JSON) via `roslibpy` — the ground station
machine never needs ROS 2 installed, which is what makes it actually
OS-independent rather than "cross-platform if you also set up ROS2."

UI: a dashboard of per-subsystem cards (Drive, Localization, Mode, Nav), each
with a "view details" drill-down subpage, plus a generic **System Nodes**
panel that lists every ROS2 node's health (alive/dead, rate, last seen) so
new nodes are visible without custom per-node UI work. Built incrementally —
only Drive is functional at first; other cards/pages are added as their
phase lands, not all at once.

Wireframe: https://claude.ai/code/artifact/17b1a796-30ea-4841-bfab-bf3b9c49390d

## Localization

- Odometry: wheel/IK feedback (once real IK exists) fused with ZED2i
  visual-inertial odometry via `robot_localization` (EKF). Until real IK
  lands, VIO alone is the odometry source.
- Two swappable localization backends, chosen by a launch argument (never
  run simultaneously):
  - **Competition/known-map mode**: AMCL against a pre-built map generated
    from the real scanned terrain mesh already in this repo. The mesh is
    decimated before rasterizing to occupancy-grid (fixes the old
    slow-rasterizer problem — same decimation trick used for the Gazebo
    collision mesh in the old plan).
  - **Test-anywhere mode**: `slam_toolbox` live SLAM, no pre-built map
    needed — used whenever testing away from the real competition yard.
- Both feed the same downstream stack (costmaps, planner); only the pose
  source changes.

## Mode supervisor

Manual / Semi-Autonomy / Full-Autonomy. **Single-tier fallback**: any fault
during Semi- or Full-Autonomy drops straight to Manual — no intermediate
fallback tier. Fallback triggers:

- Ground station comms lost (heartbeat/connection drop).
- Localization fully lost (no valid pose) — **not** mere uncertainty
  (high covariance alone does not trigger fallback).

## Nav2

Basic waypoint navigation + obstacle avoidance using Nav2's default planner,
controller, and costmap plugins. No custom recovery behaviors beyond Nav2's
stock defaults.

## Explicitly out of scope for now

- Docker-based dev environment.
- Custom swerve-drive inverse kinematics (superseded by the real IK to be
  provided).
- Scheduled Gazebo simulation phase.
- Multi-tier autonomy fallback (Full → Semi → Manual ladder).
- Nav2 custom recovery behaviors.
- Perception / science-task detection (not part of navigation).
