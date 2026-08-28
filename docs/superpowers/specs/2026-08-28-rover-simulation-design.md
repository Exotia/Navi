# Rover Simulation — Design

Written 2026-08-28. A Gazebo simulation of the Asterope rover, driven by the
bema controller's inverse kinematics, shown in the ground station in place of
the camera when the operator enters semi-autonomous mode.

## Goal

Semi-autonomous driving will eventually use the rover's localisation to place
the rover on a map. Localisation does not exist yet, so this builds everything
around it first: the URDF articulated the way the IK commands it, a world to
drive in, and a view of it in the ground station. When localisation arrives it
replaces one input — the pose — and nothing else changes.

The simulation is a view *of* the rover, not a detached toy. The rover stays
connected and stays driven; only the picture changes.

Out of scope: localisation, physics, terrain following, autonomy of any kind,
and the arm.

## Constraints found in the environment

Verified on 2026-08-28 on this laptop:

- ROS 2 Humble is installed at `/opt/ros/humble` and `rclpy` imports under
  system Python. Nothing sources `setup.bash`, which is the only reason `ros2`
  appears absent. Gazebo Classic 11.10.2, `gazebo_ros`, `ros2_control`,
  `robot_state_publisher` and `rviz2` are all present.
- The laptop already sees the rover's ROS graph over the wired link with no
  configuration at all — both are on domain 0 with the default RMW, and
  `ros2 topic list` returns `/manual_twist`, `/video_request` and
  `/video_status`. This is what removes the need for a second rosbridge.
- **There is no H.264 encoder on this laptop.** `x264enc` is absent, which is
  why an earlier loopback probe of the video path sent zero frames.
  `gstreamer1.0-plugins-ugly` is required and needs sudo.
- The ground station's virtualenv is built with
  `include-system-site-packages = false`, so the GUI cannot import `rclpy`.
  That is unchanged and deliberate: the GS remains a rosbridge client and
  learns nothing about ROS.
- `Model3D_mesh2.obj` is 161 MB: 840,753 vertices, 1,677,535 faces, extent
  37.4 m x 43.8 m with z from -1.50 to +1.03. It carries vertex colours in the
  `v` lines, which is a MeshLab extension rather than part of OBJ, so Gazebo
  will very likely load the geometry and ignore the colour.
- The IK is a Simulink-generated C++ model. `kinematics.cpp/.h` and
  `IkController` include only `<array>`, `<cmath>`, `<cstring>`, `<utility>`
  and their generated siblings — none of `bemacontroller`'s seven submodules
  (rpclib, module-drivers, the interfaces) are reachable from them.

## The IK's interface

```
in:  VX_out, VY_out, U_p, beta_hat[4], beta_dot_hat[4], TS
out: Beta_dot[4], omega[4], beta_next[4], indirect_mode,
     input_ICR, controller_ICR, feasable_ICR, current_ICR, border_ICR,
     eta_dot_constrained, eta_dot_ref_init
```

Body twist in; four steering angles and four wheel speeds out, plus the ICR
diagnostics that say whether the requested motion was feasible. `TS` is fixed
at 0.06 s, matching the rover.

## Architecture

Everything runs on the laptop. The rover must be reachable, because it hosts
the ROS graph carrying `/manual_twist`.

```
gamepad -> GS --rosbridge--> rover graph --/manual_twist--> navi_sim_ik
                                                                 |
                                          steering + wheel commands, pose
                                                                 v
                                                             Gazebo
                                                                 |
                                                         chase camera
                                                                 v
                                       navi_sim_video --H.264/RTP--> udp 5601
                                                                 |
                                                                 v
                                            GS VideoReceiver -> VideoPanel
```

The ground station gains a mode switch and nothing else. It publishes the same
twist to the same place in both modes; the switch selects which video port the
panel receives on and asks the rover to stop streaming when the sim is shown.

### Kinematic, permanently

The simulation applies the IK's outputs to the joints and integrates the body
pose from the commanded twist. There is no contact, no friction and no
`ros2_control`.

This is not a first step towards physics. Once localisation supplies the pose,
physics would only ever serve to *test* traction and stability, which is not
what this simulation is for. The URDF has one `<inertial>` tag in the entire
file; authoring masses, inertia tensors and friction coefficients with no
measured rover data to check them against would produce a simulation whose
misbehaviour could not be attributed — a wrong `mu1` looks exactly like an IK
bug. Deliberately not built, and no seam left for it.

The consequence to keep visible: the pose is dead reckoning, so it drifts from
the real rover and the picture cannot tell. The view carries a permanent
"DEAD RECKONING — no localisation" marker until localisation feeds the pose.

### Two video ports

The rover streams to 5600 and the simulation to 5601. A single shared port
would require the mode switch to stop one sender before starting the other,
and a late packet from the old source would decode as garbage in the new
stream. Separate ports remove the ordering hazard entirely. The switch still
asks the rover to stop streaming, to spare the field link.

### No control plane for sim video

The rover needs `/video_request` because it is a different machine that has to
be told to start encoding. The simulation is on the same machine as the ground
station, so its sender simply streams whenever the simulation runs, and the
mode switch starts the receiver the ground station already owns. Carrying the
rover's control plane across would have been copying a mechanism without its
reason.

## Components

### `navi_sim_ik` (C++, ament_cmake)

Vendors `bemacontroller/src/ert_rtw/*.cpp` and `IkController` unchanged, so the
simulation runs byte-identical math to the rover and cannot silently diverge
from it. The generated code is treated as a black box to be regenerated, never
edited.

Subscribes `/manual_twist`, steps the model every 60 ms, publishes joint
commands and the integrated pose. There are no encoders in a kinematic
simulation, so `beta_hat` and `beta_dot_hat` are fed back from the model's own
previous `beta_next` and `Beta_dot`. It also publishes the ICR diagnostics,
which are the only way to see that a commanded motion was infeasible.

### `navi_sim_bringup`

The URDF's four wheels, currently on fixed joints, become a steering joint
(revolute about Z) carrying a wheel joint (continuous about Y) at each corner.
The world file loads `Model3D_mesh2.obj` as visual-only scenery. Also holds the
chase camera and the launch file.

### `navi_sim_video` (Python)

Subscribes to the chase camera and pipes raw frames into `gst-launch` as
H.264/RTP to `127.0.0.1:5601`. This mirrors the rover's `video_sender`,
including sending stderr to a temp file rather than a pipe nothing drains.

### Ground station

A mode switch, a receiver that can be pointed at either port, and the dead
reckoning marker. `VideoReceiver` and `VideoPanel` are otherwise unchanged —
the sim stream is H.264/RTP exactly like the rover's.

## The wheel index mapping

`beta_hat[i]` corresponds to Steer Module `i+1`, and `omega[i]` to Drive Module
`i+1`. **Which physical corner is module 1 is not recorded anywhere in this
repository** — it is wiring knowledge held in the hardware.

Getting it wrong produces a simulation that steers the wrong corners while
looking entirely plausible, so it is not left implicit: the mapping lives in
one named constant in `navi_sim_ik`, and until it is confirmed against the
rover the simulation logs it at startup. The default assumed order is
front-left, front-right, rear-right, rear-left — assumed, not verified.

## Error handling

- The panel's existing distinction between "the source says it is streaming
  but nothing arrives" and "the local receiver died" is source-agnostic and
  covers the simulation unchanged.
- The rover being unreachable takes `/manual_twist` with it, so the simulation
  freezes rather than drifting. The IK node reports the topic going stale.
- An infeasible command is not an error: the IK's `feasable_ICR` and
  `indirect_mode` outputs are surfaced rather than hidden, since that
  behaviour is a large part of why the real algorithm is being used.
- A dead simulation must never read as a healthy one. If frames stop, the
  panel says so in the same words it uses for the rover.

## Testing

- `navi_sim_ik`: unit tests over recorded twist sequences, asserting the
  vendored model's outputs. This is where silent divergence from the rover
  would be worst, so it gets the most attention.
- URDF: parses, and the eight new joints exist with the expected axes,
  limits and parents.
- `navi_sim_video`: the rover `video_sender`'s test approach, reused.
- Ground station: the mode switch selects the right port and requests the
  right rover state; the dead reckoning marker is present in the sim view.
- Gazebo itself is verified by running it, not by unit tests.

## Prerequisites

- `sudo apt install gstreamer1.0-plugins-ugly` on the laptop, for `x264enc`.
- The rover reachable on the wired link, since it hosts `/manual_twist`.

## Deferred

- Localisation replacing the dead-reckoned pose. The interface it will use is
  the pose the IK node already publishes.
- Terrain following. The rover drives on a flat plane at z = 0 with the scan
  as scenery; the relief in this particular scan is only about ±1.5 m over
  37 m, so the clipping may not be worth fixing. Decide after looking at it.
- Converting the map to COLLADA to keep its vertex colours, and decimating
  1.68 M faces if load time proves painful.
- Physics. Recorded here as a decision, not a backlog item.
