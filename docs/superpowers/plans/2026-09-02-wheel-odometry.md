# Wheel odometry (Tier B), and the EKF it unlocks (Tier C)

Status: PLANNED, not started. Tier A (commanded-twist odometry with gyro
heading, `navi_localization/twist_odometry.py`) is being built separately
and is the interim; nothing in this plan blocks on it, and nothing here
throws it away - the EKF in Tier C simply swaps its odometry input from
the commanded topic to the measured one when this lands.

## Why

The rover's only position source is the ZED's visual-inertial tracking.
When the camera is blinded - sun, dusk, a lens full of dust - position is
gone and the run halts on grace periods. The reference stack this project
is benchmarked against (AGH's kalman_robot) survives exactly this because
losing one sensor does not silence the choir: a robot_localization EKF
fuses wheel odometry, IMU and vision as separate voices. Asterope has all
the hardware for the missing voice; what is missing is plumbing.

## The facts the plan stands on

- Every drive motor is run by a TMC4671 field-oriented controller, which
  REQUIRES a per-motor encoder to function at all and exposes measured
  velocity as a register read. The wheel speeds exist; nobody asks.
- The four steering angles are measured - `asteropeEncoderOffsets.json`
  is their zero calibration, consumed by `BemaServer.cpp` on the primary
  Pi (192.168.178.26) - and the BEMA's own IK takes `beta_hat[4]` and
  `beta_dot_hat[4]` as INPUTS, so measured steering reaches its software
  layer today.
- The BEMA's status serializer (`IkController::serialize`) already builds
  a JSON carrying its ICR states, so a telemetry path to the RPC layer
  exists in shape, if not necessarily in content.
- The forward kinematics needed - four wheel speeds and four steering
  angles into one body twist - is the same ICR geometry `navi_shaper`'s
  `ik_geometry.py` transcribes from the 2.42 model, run in reverse: a
  least-squares body twist from eight wheel-frame constraints. The
  chassis constants (`HPARAMS`, wheel radius 0.125 m) are already in the
  repo and already trusted.

## THE GATE - task 0, answer before anything else is scheduled

**Does the BEMA msgpack-RPC already expose wheel telemetry (per-wheel
omega and beta), and at what rate?**

How to answer, in preference order:

1. Re-clone the firmware read-only (`git clone
   ssh://git@gitlab.star-dresden.de:11022/star-projekte/merope/bema/bemacontroller.git`
   - the local copy was deliberately deleted) and grep the msgpack
   dispatch table for status/telemetry methods; follow what F-numbers
   map to and what `serialize()` output is reachable from outside.
2. Probe the live Pi: the bridge already speaks the protocol from
   `navi_teleop/bema_bridge.py`; a bench script calling each known
   method and dumping replies answers it empirically in an hour.

Two outcomes, two very different calendars:

- **RPC exists**: everything below is rover-side only. 1-2 days.
- **RPC missing**: a few added lines in `bemacontroller` (extend the
  status JSON or add one method), but it is another team's repo and
  their Pi - the code is hours, the coordination is the calendar. Budget
  a week of elapsed time and start the conversation early. The LED-fix
  precedent shows the workflow: branch, small commit, push to their
  GitLab, deploy to the Pi with them present.

## Tasks (after the gate)

1. **`wheel_kinematics.py` (pure, navi_shaper or navi_localization).**
   Forward kinematics: `body_twist(omega[4], beta[4]) -> (vx, vy, wz,
   residual)`. Least squares over the eight rolling constraints, built on
   `ik_geometry`'s constants. The residual is the slip indicator and gets
   published - it is free and it is the stuck-detection the gap analysis
   wanted. Tests against the vendored 2.42 model outputs: feed the
   model's own omega/beta for known twists, require recovery to float
   precision; degenerate cases (all-stop, pure spin, one wheel
   disagreeing) pinned.

2. **Telemetry into ROS.** Extend `bema_bridge` (it already owns the RPC
   session, the deadman, and the 20 Hz cadence) to poll telemetry and
   publish `sensor_msgs/JointState` (raw, for debugging) plus
   `nav_msgs/Odometry` on `/odom/wheels` via task 1. Covariance from the
   residual: clean rolling is trustworthy, high residual (slip) inflates
   it, which is exactly what the EKF needs to de-weight slipping wheels.
   No tf published - the ZED owns the tree until Tier C decides
   otherwise. Unit conversions pinned by test against the TMC4671
   datasheet scaling and the wheel radius; sign conventions against
   `bema_bridge`'s existing negations (REP-103 vs model frame - this is
   where the silent bugs live, so every sign gets a test naming the
   convention it implements).

3. **Bench validation, wheels off the ground.** Command known twists,
   compare `/odom/wheels` twist against `/chassis_twist` - they must
   agree to a few percent with the wheels free. Then floor tests: drive
   a taped 5 m line and a 360 degree spin, closure error recorded in
   docs. Acceptance: under 5 percent distance error on the line, under
   10 degrees on the spin - loose numbers on purpose; the EKF weighs it,
   it does not have to be perfect, only honest about its covariance.

4. **Tier C - the EKF (separate go/no-go after 1-3).**
   `robot_localization` ekf_node fusing `/odom/wheels` (vx, vy, wz),
   the ZED IMU (yaw rate, orientation), and the ZED pose as a pose
   source. The hard part is not configuration but AUTHORITY: today the
   ZED wrapper publishes map->odom->base and everything downstream
   trusts it. The EKF must take over odom->base with the wrapper's tf
   demoted (`publish_tf: false` in the camera config, map->odom kept) -
   a cutover that changes the meaning of every odom-frame consumer
   (Nav2's local costmap, the controller's odometry topic) and must be
   rehearsed in the sim and on a bench day, not discovered in a yard.
   Kalman's split (ekf local + ekf global) is the template; their config
   is in the reference clone under kalman_slam/config for the taking.

## Risks

- **R1 - the gate goes the slow way** (RPC missing): calendar risk only;
  mitigation is starting the cross-team ask the same day the gate is
  answered.
- **R2 - encoder units and signs**: TMC4671 velocities are electrical
  revs with pole-pair scaling; a factor-of-N error looks plausible on a
  bench. Mitigation: task 3's taped-line test is the ground truth, and
  task 2 refuses to merge without it.
- **R3 - steering encoder rate**: if beta arrives slower than omega, FK
  pairs stale angles with fresh speeds through turns. Measure the rates
  in task 0; if needed, interpolate beta to omega timestamps.
- **R4 - the tf cutover** (Tier C): highest-blast-radius change in the
  stack; gets its own plan, its own sim rehearsal, and a revert
  runbook before any yard attempt.

## What NOT to do

- Do not publish a second odom tf from any Tier B node.
- Do not let the FK guess through a missing telemetry field - a wheel
  with no reading makes the residual explode and the covariance say so.
- Do not begin Tier C before task 3's numbers are written down.
