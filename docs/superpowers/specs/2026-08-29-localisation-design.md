# Localisation for Semi-Autonomous Mode — Design

Written 2026-08-29. Gives the rover a pose, shows that pose in the ground
station's Gazebo view, and builds a map of what the rover has seen. Replaces
the dead-reckoned pose the 2026-08-28 simulation design left as its one open
input.

## Goal

In semi-autonomous mode the operator drives the rover looking at the Gazebo
view only — no camera stream is allowed. The rover in that view must be where
the real rover is, and the ground under it must be either the organisers' scan
of the yard or what the rover has itself seen so far.

Today's IK-driven, dead-reckoned simulation stays as its own fourth mode,
"Simulation", for driving with no rover localisation. It is not changed.

Out of scope: autonomy of any kind, path planning, Nav2, the arm, terrain
physics, the second (rear) ZED, and matching against a prior scan (see
"Deferred" — the interfaces are laid out for it).

## Decisions taken with the user

- Everything on the ZED 2i may be used **except the magnetometer**. Its topic
  is never subscribed; nothing feeds it into a heading.
- There is no wheel odometry on the rover and none is invented from the
  commanded twist: commanded motion on sand is not measured motion, and the
  ZED's IMU coasts through short dropouts better than a lie would.
- The rover side (localisation, video, mapping) runs without the simulation
  and without the ground station. Gazebo is a consumer of the pose and the
  map, never a dependency of anything on the rover.
- Only the front ZED is connected today. The URDF already carries both camera
  bodies; the front one is the black 37 x 188 x 30 mm box at
  `(0.322, 0, 0.154)` in `base_link`, facing +X.

## Constraints found in the environment

Verified on 2026-08-29 on the Orin (`star@a_navi`) and this laptop:

- ZED SDK 4.2.5; `zed-ros2-wrapper humble-v4.2.5` built and working in
  `~/workspaces/isaac_ros-dev`. With positional tracking on,
  `/zed/zed_node/odom` publishes at **30.0 Hz** in `odom -> zed_camera_link`;
  load is ~25 % per core, GPU 68 %, 1.2 GB RAM. `robot_localization`,
  `slam_toolbox`, `nav2_bringup`, `grid_map_*`, `pcl_ros` are installed.
- **The wrapper takes 90 s to reach odometry** because it advertises ~30
  image topics and initialises a `libx264` ffmpeg image transport for each.
  This is a configuration problem, not a hardware one.
- **The ZED SDK opens the camera exclusively.** The existing `video_sender`
  opens the same camera as a UVC device through `v4l2src`. They cannot run at
  the same time — this is the one structural decision in this design.
- The Orin-side code lives in its own git repository at `~/navi` on the Orin
  (`navi_teleop`, `start_navi.sh`), clean, no remote. Nothing there is in this
  repository yet.
- One ZED 2i is connected (serial 34040416; both `/dev/video0` and `/dev/video1`
  are its nodes).
- Neither machine has `domain_bridge`. `rclpy` in Humble supports several
  contexts with different `domain_id`s in one process, which is enough.
- Gazebo Classic cannot update a heightmap in place; a terrain model has to be
  respawned to change.

## Architecture

```
                              ORIN                                     LAPTOP
 ZED 2i --stereo+IMU--> zed_wrapper --/zed/zed_node/odom-------> localization_status
                             |        --/zed/zed_node/pose/status-->        |
                             |                                    /localization/pose   (map -> base_footprint, 30 Hz)
                             |                                    /localization/status (OK | SEARCHING | OFF)
                             |                                              |
                             |--rgb/image_rect_color--> zed_video_sender --H.264/RTP--> GS video panel (manual mode only)
                             |
                             |--fused point cloud--> elevation_mapper --/localization/map (grid, ~0.5 Hz)-->
                                                                                            |
                                                                        sim_bridge (domain 0 <-> sim domain)
                                                                                            |
                                                             Gazebo: rover model at /localization/pose,
                                                                     terrain from /localization/map,
                                                                     chase camera --H.264/RTP 5601--> GS panel
```

Three sub-projects, in build order. Each is usable on its own.

1. **Rover localisation** — the Orin publishes a pose and its health.
2. **Ground station and view** — a fourth mode, Gazebo placed by the pose.
3. **Built map** — the rover maps what it sees; the view shows it.

## Repository layout

The Orin repository is brought into this one as `rover/` with `git subtree`
so its history survives, and it stops being a second place to look. `rover/`
holds `src/navi_teleop`, `src/navi_localization`, `start_navi.sh`. A
`deploy_rover.sh` on the laptop rsyncs `rover/` to `~/navi` on the Orin and
runs `colcon build` there over ssh; `~/navi` becomes a deploy target, not a
development tree.

## Sub-project 1: rover localisation

### `zed_wrapper` configuration

A `navi_localization/config/zed_front.yaml` overlaying the wrapper's
`common_stereo.yaml` + `zed2i.yaml`, launched from
`navi_localization/launch/localization.launch.py`:

- `pos_tracking_enabled: true`, `imu_fusion: true`, `area_memory: true` (the
  SDK's own loop closure), `publish_tf: true`, `publish_map_tf: true`,
  `two_d_mode: false`, `set_gravity_as_origin: true`.
- `camera_name: zed`, `base_frame: base_footprint` is **not** used: the wrapper
  is told its own frame is `zed_camera_link` and the offset to the rover is
  in the URDF (below). Re-expression happens in `localization_status`, so a
  wrong mount offset is one number in one place.
- Object detection, body tracking, streaming and SVO recording off. Mapping
  off here; sub-project 3 turns it on.
- Image transport: only `raw` published, ffmpeg/compressed/theora plugins
  disabled through the `.image_transport` parameter namespace. Image
  publishing at `pub_downscale_factor: 2.0` (640 x 360) and `pub_frame_rate`
  15; the video sender is the only consumer.
- Target: odometry within **20 s** of launch. Measured before and after; the
  number goes in the launch file's docstring.

### `localization_status` node (Python, `navi_localization`)

Subscribes `/zed/zed_node/odom` and `/zed/zed_node/pose/status`. Publishes:

- `/localization/pose` — `nav_msgs/Odometry`, `frame_id: map`,
  `child_frame_id: base_footprint`, at the wrapper's rate. The wrapper's
  `map -> odom` and `odom -> zed_camera_link` are composed with the static
  `zed_camera_link -> base_footprint` from the URDF. Covariance copied from
  the wrapper's pose-with-covariance.
- `/localization/status` — `navi_localization_msgs/LocalizationStatus`:
  `state` (`OK`, `SEARCHING`, `OFF`), `seconds_since_ok`, `source`
  (`"zed_vio"`), `distance_travelled`. Published at 2 Hz and on every state
  change. While `SEARCHING`, `/localization/pose` keeps publishing the **last
  good** pose with its stamp frozen, so consumers can see it is stale. Nothing
  is extrapolated.

A custom message rather than `diagnostic_msgs` because the ground station
reads it over rosbridge as JSON and a flat, typed message is the thing it can
render without parsing key/value strings.

### `zed_video_sender` (Python, `navi_teleop`)

`video_sender` keeps its `/video_request` / `/video_status` contract and its
refusal logic (`video_request.py` is untouched). Its source changes from
`v4l2src device=...` to a subscription on `/zed/zed_node/rgb/image_rect_color`
whose frames are written into `gst-launch-1.0` through stdin — the exact
`fdsrc blocksize=... ! rawvideoparse` front end `navi_sim_video` uses, which
is the one that was verified to frame a byte stream correctly. The request's
`device` field is ignored with a logged note; width/height come from the
image, and a request whose geometry differs from the published image is
refused with the reason in `/video_status`, not silently rescaled.

When the wrapper is not running (`--no-localization`), the old `v4l2src` path
is still available: `video_sender` gains a `source` parameter, `zed_topic`
(default) or `v4l2`, so a rover day without localisation still has video.

### URDF

`asterope_iiI.urdf` gains a `zed_camera_link` on a fixed joint from
`base_link` at `xyz="0.322 0 0.154" rpy="0 0 0"` — the front camera box's
centre — and a `zed_rear_camera_link` at the rear box, unused. `tests/test_urdf.py`
asserts both. The wrapper's own `zed_descr.urdf.xacro` is **not** loaded on
the rover; the composed tree is `map -> odom -> zed_camera_link` from the
wrapper and the static `base_link -> zed_camera_link` from `robot_state_publisher`
on the Orin, which therefore now runs there with the URDF.

Because the ZED body box was authored from photos, not from a measurement,
the offset is flagged in the URDF comment as **unverified** and
`localization_status` logs it at startup. Verifying it is a rover-day item.

### `start_navi.sh`

- `--no-localization` skips the wrapper, `robot_state_publisher` and
  `localization_status`; `video_sender` is then launched with `source:=v4l2`.
- Stale-process cleanup gains `component_container_isolated` and
  `robot_state_publisher`.
- Readiness waits for `/localization/status` to be **received** on a
  subscription, with a 60 s timeout that fails the launcher loudly.

### Testing

- `localization_status`: pure-Python tests of the transform composition
  against hand-computed poses, the `SEARCHING` freeze, and the state machine.
  No ROS runtime.
- `zed_video_sender`: the existing `test_video_sender.py` approach — the
  pipeline is a list of tokens, assert on it.
- URDF: `tests/test_urdf.py` extended.
- Rover day: drive a closed loop of ~20 m, record `/localization/pose` at
  start and end, report closure error. Startup time to first pose recorded.

## Sub-project 2: ground station and view

### Modes

`dashboard_page` gets four radio buttons: **Manual**, **Semi-autonomous**,
**Autonomous** (present, disabled, tooltip "not implemented"), **Simulation**.
`main_window._mode` takes the values `manual`, `semi_auto`, `autonomous`,
`simulation`.

- `manual`: unchanged.
- `simulation`: exactly what `semi_auto` does today — rover video stopped,
  panel on port 5601, DEAD RECKONING marker. The IK-driven sim.
- `semi_auto`: rover video stopped (and refused: `_on_stream_requested` in
  this mode does nothing but say "no camera stream in semi-autonomous mode"
  on the panel), panel on port 5601, marker shows the **localisation status**
  from `/localization/status` — `LOCALISED`, `SEARCHING … 4 s`, or
  `LOCALISATION OFF` in the same colours the panel uses today. The twist keeps
  reaching the rover in every mode, as before.

`ros_client` subscribes `/localization/status` and `/localization/pose` (the
pose only for the header readout `x y yaw`, at most 5 Hz — rosbridge
throttle). Nothing under `ground_station/` imports `rclpy`.

### Gazebo in semi-autonomous mode

`start_sim.sh --mode semi|simulation` (default `simulation`, so the existing
behaviour is the default).

In `semi` mode the simulation runs on its own ROS domain
(`ROS_DOMAIN_ID`, default 42, argument `--sim-domain`), and a new
`sim_bridge` node in `navi_sim_bringup` holds two `rclpy` contexts:

- domain 0 (the rover graph): subscribes `/localization/pose`,
  `/localization/status`, `/localization/map`, `/manual_twist`.
- the sim domain: republishes them under the same names.

Nothing goes the other way. `/clock`, `/tf` and the sim's
`/robot_description` never reach the rover graph — the collision the
2026-08-28 design flagged is closed.

`sim_ik_node` gains a `pose_topic` parameter (default empty). When set, it
subscribes `nav_msgs/Odometry` there and, on each message, **replaces** its
integrated pose with the received one before publishing the joint trajectory
and setting the model. The IK keeps driving the wheel and steering joints
from `/manual_twist` so the wheels still turn and steer in the picture; only
the body pose comes from outside. While `/localization/status` is not `OK`
the node stops applying poses and the model holds still; the marker says why.

The model pose is applied through `gazebo_ros_state`'s `/set_entity_state`
at the pose rate, capped at 30 Hz. The `planar_move` plugin is **not** loaded
in `semi` mode — two writers of one model pose would fight.

In `simulation` mode the launch is unchanged: domain 0, no bridge, `planar_move`.

### Testing

- Ground station: mode switch tests extended to four modes; the semi-auto
  stream refusal; the marker text for each status value; `ros_client`
  subscriptions. All with the existing fake rosbridge.
- `sim_bridge`: a test with two contexts in one process on two throwaway
  domain ids proves a message published on one appears on the other and not
  back.
- `sim_ik_node`: the pose override path in the existing gtest harness —
  given a pose message, the next published pose equals it, and while status
  is `SEARCHING` no pose is applied.
- End to end on the laptop: `mock/ros_bridge.py` gains a fake
  `/localization/pose` that walks a square; Gazebo's rover walks the same
  square, checked by reading `/gazebo/model_states`.

## Sub-project 3: built map

### On the Orin

The ZED SDK's spatial mapping is turned on (`mapping_enabled: true`,
`resolution: 0.10`, `max_mapping_range: 8.0`, `fused_pointcloud_freq: 0.5`).
It is the same GPU pipeline as the tracking, needs no new package, and is
consistent with the pose by construction.

`elevation_mapper` (Python, `navi_localization`) subscribes the fused point
cloud (`/zed/zed_node/mapping/fused_cloud`) and maintains a 2.5-D grid:
`grid_map_msgs/GridMap` with one `elevation` layer, 0.10 m cells, in the
`map` frame, growing as needed up to 60 x 60 m. Each cell holds the mean z
of the points in it; empty cells are NaN. Published on `/localization/map`
at 0.5 Hz **only when changed**. At 60 x 60 m and float32 that is 1.44 MB
per full publish; the rover's yard is 37 x 44 m and a run's map is far
smaller, and 0.5 Hz on a wired link is acceptable. A delta encoding is a
deferred item recorded below, not built now.

### On the laptop

`terrain_writer` (Python, `navi_sim_bringup`, sim domain) subscribes
`/localization/map`, writes it as a Gazebo heightmap (a `(2^n + 1)`-square
16-bit PNG plus the SDF model wrapping it), and respawns the `terrain` model
through `/delete_entity` + `/spawn_entity`, at most every 5 s and only on
change. The rover model is untouched by the respawn. The world file gets no
static terrain in `semi` mode; the ground plane at z = 0 remains so the rover
is never in the void.

Gazebo renders the heightmap with a single flat colour; the rover's own scan
shows as relief only. That is the truth of what has been seen, which is the
point.

### Testing

- `elevation_mapper`: pure-Python tests — points to cells, mean z, growth,
  the unchanged-map suppression.
- `terrain_writer`: the PNG geometry and SDF for a given grid; the 5 s cap.
- Laptop end to end: a recorded fused cloud (captured on a rover day, checked
  in under `rover/test_data/`, a few MB) replayed into the mapper, terrain
  appears in Gazebo.
- Rover day: drive the yard, watch the terrain fill in, compare visually to
  the real ground.

## Error handling

- Localisation lost: pose freezes, status says `SEARCHING` with the count,
  the GS marker turns the panel's failure colour, the Gazebo rover holds
  still. Recovery is automatic when the SDK re-acquires; the frozen stamp
  makes the gap visible in any log.
- Wrapper crashes: `localization_status` publishes `OFF` after 2 s without
  odometry; `start_navi.sh` does not restart it (a silent restart would
  reset the pose to zero and the map to nothing). The operator restarts.
- Rover unreachable: `sim_bridge` stops receiving; `sim_ik_node` zeroes the
  command as it does today; status goes stale and the panel says so.
- A video request in semi-auto is refused with the reason on the panel.
- The mount offset is logged at startup as unverified until measured.

## Deferred

- Matching against a prior scan (ICP/NDT of the ZED cloud against the
  organisers' mesh) to put the pose in the organisers' frame. Slots in as a
  publisher of `map -> odom` in place of the wrapper's own; nothing in
  sub-projects 2 and 3 changes. Decide when the ERC map format is known.
- `robot_localization` EKF. Only worth it once there is a second source.
- Delta encoding of `/localization/map`.
- The rear ZED. `zed_rear_camera_link` is in the URDF for it.
- Trimming `Model3D_mesh1.ply` / deciding its version-control fate.
- Bringing `~/workspaces/isaac_ros-dev` (the wrapper build) under this
  repository's control. It is treated as an installed dependency.
