# Obstacle Voxels Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Everything the ZED sees that is not ground is published per tile as 5 cm occupied voxels and drawn in Gazebo as grey blocky 3-D geometry on top of the orange terrain, live, with save/load/clear.

**Architecture:** Rover: `voxels.py` (pure numpy) voxelises the same fused-cloud update the grid bins, per 2.5 m tile, replace-per-tile; a `PayloadScheduler` generalises `TileScheduler`; `elevation_mapper` publishes `/localization/obstacle_tile` (PointCloud2 of voxel centres, tile id in `frame_id`), persists voxels in the npz, clears/loads them. Laptop: `sim_bridge` carries the topic; `obstacle_mesh.py` (pure numpy) makes a hidden-face-removed cube mesh; `terrain_writer` gains a second model kind through its existing spawn-before-delete/bounded/verified path.

**Tech Stack:** Python 3.10, numpy 1.21, ROS 2 Humble (`sensor_msgs`, `grid_map_msgs`, `gazebo_msgs`), pytest.

**Spec:** `docs/superpowers/specs/2026-08-30-obstacle-voxels-design.md` (binding), on top of `docs/superpowers/specs/2026-08-29-tiled-map-design.md`.

## Global Constraints

- Never publish to `/manual_twist`; laptop ROS tests on throwaway domains (91/92/93), never domain 0; `pkill -x` only; never `pkill -f` with a pattern matching your own shell.
- Commits: `git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit`, explicit `git add <paths>`, never push, never `git add -A`; on `index.lock` wait 2 s and retry (other agents work in the same tree on other files).
- Rover tests: `bash -c 'source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_localization:$PYTHONPATH python3 -m pytest rover/src/navi_localization/test -q -p no:cacheprovider --ignore=rover/src/navi_localization/test/test_localization_status.py'`. Sim tests: `PYTHONPATH=$PWD/sim/src/navi_sim_bringup:$PWD/rover/src/navi_localization:$PYTHONPATH python3 -m pytest sim/src/navi_sim_bringup/test -q -p no:cacheprovider`, then `cd sim && colcon build --packages-select navi_sim_bringup`.
- Constants (spec): `VOXEL = 0.05`, `OBSTACLE_MIN_ABOVE_GROUND = 0.10`, `OBSTACLE_MAX_ABOVE_ROVER = 2.5`, `MIN_POINTS_PER_VOXEL = 2`, tile key = `(ix // 50, iy // 50)` of the voxel's x/y index, voxel centre `(index + 0.5) · 0.05`, `frame_id = "map|<ix>|<iy>"`.
- The sim package must not import the rover package at runtime.

---

### Task 1: `voxels.py` — voxelisation and `ObstacleMap`; `PayloadScheduler` (rover, pure)

**Files:** Create `rover/src/navi_localization/navi_localization/voxels.py`, `rover/src/navi_localization/test/test_voxels.py`; Modify `rover/src/navi_localization/navi_localization/tiles.py` (+ `test_tiles.py`).

**Interfaces produced:**
- `voxels.py`: constants above; `occupied_voxels(points: (N,3) float64, ground_height: callable((ix_cell, iy_cell) int arrays) -> float array with NaN, rover_z: float | None) -> dict[tuple[int,int], np.ndarray]` — per tile key, sorted `(M,3) int32` absolute voxel indices; `class ObstacleMap` with `update(points, ground_height, rover_z) -> None` (replace touched tiles; a tile touched by points but yielding no voxels becomes empty), `tiles() -> dict[key, (M,3) int32]` (only non-empty tiles), `state() -> (K,3) int32` (all voxels concatenated, sorted; empty → shape (0,3)), `replace(voxels: (K,3) int32)`, `clear()`, `voxel_count -> int`.
- `tiles.py`: `class PayloadScheduler(changed=lambda new, old: not np.array_equal(new, old))` with exactly `TileScheduler`'s `offer/due/published/mark_all_dirty/forget_all/is_dirty` semantics and timing (8 dirty per tick oldest first, ≥ 1 s per tile, one round-robin keepalive with the `<=` rule, never a tile never seen); `TileScheduler(PayloadScheduler)` keeps its 1 cm/newly-finite comparator. All existing `test_tiles.py` tests must still pass unchanged.

Tests (TDD): a point 5 cm above ground is not a voxel, 15 cm is; a single point in a voxel is dropped, two are kept; a point 3 m above the rover is dropped, with `rover_z=None` it is kept; a cell without ground uses `rover_z - 1.0` as reference; voxel → tile key for negative indices; two updates: the second replaces the first tile's voxels and leaves an untouched tile alone; a touched tile that lost all voxels becomes empty (absent from `tiles()`); `state()/replace()/clear()` round trip; sorted determinism; `PayloadScheduler` with array payloads: dirty on change, quiet on same bytes, 8-per-tick, keepalive round-robin, `forget_all`. Performance: 200k points must voxelise in < 60 ms on this laptop — report the number.

Commit: `"Voxelise the fused cloud into per-tile obstacle voxels, and generalise the tile scheduler to any payload"`.

---

### Task 2: `obstacle_mesh.py` (laptop, pure)

**Files:** Create `sim/src/navi_sim_bringup/navi_sim_bringup/obstacle_mesh.py`, `sim/src/navi_sim_bringup/test/test_obstacle_mesh.py`.

**Interfaces produced:** `obstacle_mesh_from_voxels(centres: (N,3) float, size: float = 0.05) -> ObstacleMesh | None` (`vertices (V,3)`, `normals (V,3)`, `faces (F,3) int64`, `voxel_count`); `obj_bytes(mesh) -> bytes` (no mtllib, fixed-format numbers, deterministic for the same sorted input); `obstacle_sdf(mesh_uri, model_name) -> str` (static, visual only, grey ambient `0.55 0.55 0.58`, diffuse `0.62 0.62 0.65`). Hidden-face removal: a cube face is emitted only if the neighbour voxel in that direction is absent (use integer voxel indices `round(centre/size - 0.5)` and set membership via `np.unique` on packed keys or a Python set of tuples for ≤ 5k voxels — must handle 5,000 voxels in < 50 ms). Winding outward (normal points away from the cube). Empty input → `None`.

Tests: one cube → 12 faces, 8 vertices (shared per cube is fine; per-face vertices also acceptable if documented); 2×2×2 block → 24 faces; two separate cubes → 24; a 10×1×1 wall segment → 42; normals outward (each face's centroid + normal is further from the block centre than the centroid); determinism; SDF grey, static, no collision, no heightmap; None on empty.

Commit: `"Cube meshes with hidden faces removed for the obstacle voxels"`.

---

### Task 3: bridge topic + synthetic scene (laptop, small)

**Files:** Modify `sim/src/navi_sim_bringup/scripts/sim_bridge.py` (`DEFAULT_TOPICS` + comment), `sim/src/navi_sim_bringup/launch/sim.launch.py` (`bridged`), `sim/src/navi_sim_bringup/test/test_sim_bridge.py` (pin `/localization/obstacle_tile:sensor_msgs/msg/PointCloud2`), `sim/src/navi_sim_bringup/test/publish_synthetic_cloud.py` (`--wall`: 1.2 m high wall along the +y edge of the area, points every 3 cm; `--box`: a 0.5 m cube at (2.0, 1.0); both add to the ground points; docstring updated).

Commit: `"Bridge the obstacle tiles; the synthetic cloud can carry a wall and a box"`.

---

### Task 4: `elevation_mapper` publishes, saves, loads and clears obstacle tiles (rover)

Depends on Task 1. **Files:** Modify `elevation_mapper.py`, `map_store.py`, their tests; `docs/superpowers/specs/2026-08-29-tiled-map-design.md` status JSON (add `voxels`).

**Behaviour:** In `_on_cloud` after `self._grid.update(points)`: `self._obstacles.update(points, ground_height, self._rover_z)` where `ground_height(ix, iy)` reads this update's grid heights for those cells (add a small accessor on `ElevationGrid`, e.g. `height_at(ix_cells, iy_cells) -> float array with NaN` — grid indices are `floor(x/0.05)`, same as voxel x/y indices). A second scheduler `self._obstacle_scheduler = PayloadScheduler()` offered `self._obstacles.tiles()` in `_offer`; `_tick` publishes its due tiles as `build_obstacle_message(key, voxels, stamp)` on `/localization/obstacle_tile` (`OBSTACLE_TILE_TOPIC`, publisher depth 64) after the terrain tiles. `build_obstacle_message`: PointCloud2, fields x/y/z FLOAT32 offsets 0/4/8, `point_step 12`, `height 1`, `width N`, `is_dense True`, `header.frame_id = f"map|{ix}|{iy}"`, centres `(index + 0.5) * VOXEL`; `parse_obstacle_frame(frame_id) -> (ix, iy)`. `save`: npz gains `voxels` (`ObstacleMap.state()`); `MapStore.save/load` carry it (old files → empty `(0,3)`). `load`: `self._obstacles.replace(...)`, obstacle tiles present before but absent after → queued as empty obstacle messages; then `mark_all_dirty` on both schedulers. `clear`: `forget_all()` on the obstacle scheduler → empty obstacle messages queued into the same paced deque as the terrain blanks (deque entries become `(kind, key)`); `map_status` gains `"voxels": self._obstacles.voxel_count`. Ledger the queue-depth reasoning in a comment.

Tests (node level, Recorder publishers): a cloud with a wall publishes obstacle tiles with the right frame_id and centres; save/load round-trips voxels (status `voxels` count); clear sends empty obstacle tiles for every published obstacle tile; an obstacle tile whose voxels vanish in a later update is republished empty; terrain behaviour unchanged (existing tests green).

Commit: `"elevation_mapper publishes obstacle voxel tiles and saves, loads and clears them with the map"`.

---

### Task 5: `terrain_writer` draws obstacle tiles as a second model kind (laptop)

Depends on Task 2. **Files:** Modify `terrain_writer.py`, `test_terrain_writer.py`.

**Behaviour:** Keys become `(kind, ix, iy)`; `model_name(key, generation, run_id)` → `f"{kind}_{ix}_{iy}_{run_id}_g{gen}"` with `kind ∈ {"terrain", "obst"}` (existing tests updated: terrain names unchanged in form); `LEFTOVER_MODEL_RE` matches both prefixes (and the older `terrain_` forms). Subscribe `/localization/obstacle_tile` (PointCloud2, depth 64): parse `frame_id` → key `("obst", ix, iy)`; read x/y/z (float32, use `np.frombuffer` with the point_step, refuse other layouts with a logged error); `obstacle_mesh_from_voxels` → `obj_bytes` payload, or `b''` when empty → `_remove`. Mesh files `obst_<ix>_<iy>_v<N>.obj`; SDF via `obstacle_sdf`. Everything else (policy, budget, doomed, watchdog, model-list verification, sweep) shared and unchanged. The existing `_on_tile` for terrain builds keys `("terrain", ix, iy)`.

Tests: an obstacle message spawns `obst_…` with the grey SDF and the mesh file; an empty obstacle message removes it; terrain and obstacle tiles for the same (ix, iy) are independent models; the sweep dooms leftover `obst_…` from another run; a malformed cloud (wrong fields) is logged not raised; the shared budget counts both kinds (a burst of 3 terrain + 3 obstacle tiles → 4 in flight).

Commit: `"terrain_writer draws obstacle voxel tiles as grey block models beside the terrain"`.

---

### Task 6: End to end (laptop, then Orin)

Depends on 3, 4, 5. Laptop: mocks on domain 91 (`fake_localization`, mapper with `-p map_directory:=/tmp/navi_maps_test`, `publish_synthetic_cloud --wall --box`), `./start_sim.sh --mode semi --rover-domain 91 --twist-topic /sim_test_twist`; after 60 s grab a chase-camera frame on domain 42 and look at it (grey wall slab + grey box on orange ground, no spikes); `ros2 topic bw /localization/obstacle_tile` on domain 42; `get_model_list` shows `obst_*` and `terrain_*`; `clear` removes both kinds; `save`/`load` brings both back. Tear down (PIDs / `pkill -x`), domains 42/91 empty. Orin: `./deploy_rover.sh --test`, restart `start_navi.sh`, measure `hz`/`bw` of `/localization/obstacle_tile` and mapper CPU on the bench for 30 s; record in the launch docstring; then start the laptop sim in semi mode against the real rover (domain 0) and grab a frame: the bench scene should show the floor and grey blocks for the desk/walls. Report with numbers and the frame path.

Commit: `"Record the obstacle tile numbers from the bench"`.
