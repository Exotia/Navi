# Obstacle voxels — design

Date: 2026-08-30. Status: approved in conversation. Extends the tiled map
(`2026-08-29-tiled-map-design.md`); nothing there changes.

## Why

The terrain tiles now show the *ground* (20th-percentile height, cells above
the rover's band hidden). Everything else the ZED sees — walls, the desk,
chairs, boulders, people — is a hole. The operator wants those objects
displayed in 3-D at their real place and size, live, including close
objects on the bench.

## What

A sparse 3-D voxel map of everything that is not ground, published per
2.5 m tile next to the terrain tiles, and drawn in Gazebo as light-sand blocky
geometry (5 cm cubes with hidden faces removed) on top of the orange
terrain.

## Rover (`navi_localization`)

### Voxelisation — `voxels.py` (pure numpy, no ROS)

From the same fused-cloud update the grid bins:

- Voxel size `VOXEL = 0.05` m (the grid's cell size); voxel index
  `(ix, iy, iz) = floor(p / 0.05)` in the map frame.
- A point is an **obstacle candidate** when its z is more than
  `OBSTACLE_MIN_ABOVE_GROUND = 0.10` m above the ground height of its
  cell (the grid's 20th-percentile height, from this update) and at most
  `OBSTACLE_MAX_ABOVE_ROVER = 2.5` m above the rover's footprint (no pose
  yet → no upper limit). A cell with no ground height uses the rover's
  footprint z − 1.0 m as its reference.
- A voxel is **occupied** when ≥ `MIN_POINTS_PER_VOXEL = 1` candidate
  point falls in it. (Started at 2 to drop flying pixels; on the bench the
  SDK's fused cloud has ~1 point per 5 cm voxel, so 2 threw away most real
  surfaces. The fused cloud is already the SDK's filtered map.)
- Per tile `(ix // 50, iy // 50)` the update yields the occupied voxels
  as an `(N, 3) int32` array of absolute voxel indices, sorted (so equal
  content gives equal bytes). Tiles touched by this update **replace**
  their voxel set, exactly like grid cells (the SDK re-sends its whole
  fused map, and drops what it re-fuses away — that is the only "decay"
  there is; a person who walked through stays until the SDK's chunk is
  refined, which the design accepts and documents).
- `ObstacleMap`: `update(points, ground_height_of_cell, rover_z)`, per-tile
  storage, `state()`/`replace()`/`clear()` mirroring the grid;
  `tiles()` → `{key: voxels}`.

### Scheduling

`PayloadScheduler` in `tiles.py`: the `TileScheduler` rules (≤ 8 dirty per
tick oldest first, ≥ 1 s per tile, one clean tile per tick round-robin,
never a tile that was never seen) for payloads compared by `np.array_equal`
— `TileScheduler` becomes a thin subclass with its 1 cm comparator.

### Message: `/localization/obstacle_tile`

`sensor_msgs/PointCloud2`, one message per tile: fields `x, y, z` float32
(voxel **centres** in the map frame, i.e. `(index + 0.5) · 0.05`),
`height = 1`, `is_dense = true`, `header.frame_id = "map"`. Tile identity
travels in `header.frame_id` as `map|<ix>|<iy>` — the only place a
PointCloud2 can carry it for an **empty** tile (all obstacles gone), which
must be publishable. Consumers split on `|`. Bandwidth: a 2.5 m wall face
≈ 2,000 voxels ≈ 24 KB; an empty tile ≈ 100 B.

### Commands, status, persistence

`save` writes the voxels into the same `.npz` (`voxels`: `(N, 3) int32`,
absolute indices, all tiles concatenated); `load` restores them and marks
every obstacle tile dirty; `clear` publishes an empty obstacle tile for
every previously published one (paced with the terrain blanks, same
deque). `map_status` gains `"voxels": <count>`.

## Laptop

- `sim_bridge`: carries `/localization/obstacle_tile:sensor_msgs/msg/PointCloud2`.
- `obstacle_mesh.py` (pure numpy): `obstacle_mesh_from_voxels(centres,
  size=0.05) -> ObstacleMesh | None`, cube faces with **hidden-face
  removal** (a face is emitted only if the neighbouring voxel in that
  direction is absent), outward-facing winding, per-face normals, sorted
  input → deterministic `obj_bytes`; `obstacle_sdf(uri, model_name)` grey
  (`0.82 0.80 0.70`) static visual-only model.
- `terrain_writer`: a second model **kind** per tile. Keys become
  `(kind, ix, iy)` with `kind ∈ {"terrain", "obst"}`; model names
  `terrain_<ix>_<iy>_<run>_g<N>` / `obst_<ix>_<iy>_<run>_g<N>`; mesh
  files `tile_…` / `obst_…`; the leftover sweep matches both prefixes.
  One `TileRespawnPolicy`, one factory budget, one doomed list — shared.
  An empty obstacle tile removes the model (payload `b''`), as an all-NaN
  terrain tile does.
- `publish_synthetic_cloud.py` gains `--wall` (a 1.2 m wall along one
  edge) and `--box` (a 0.5 m cube) so the laptop end-to-end shows blocks.

## Ground station

Unchanged.

## Testing

Pure: voxelisation (threshold, min points, upper limit, replace-per-tile,
sorted determinism, state round trip); `PayloadScheduler` rules;
`obstacle_mesh` hidden-face removal (a 2×2×2 block has 24 faces, not 48;
two separate cubes 12 each), winding, determinism, empty → None. Node:
obstacle tiles published, empty tile after clear, save/load round trip,
frame_id identity parses. Writer: obstacle model spawned/replaced/removed
through the shared path; leftover sweep. Bridge default topic. End to end
on the laptop with `--wall --box`: grey blocks on orange ground in the
chase-camera frame; Orin: rates and bytes with the bench scene.

## Out of scope

Ray-cast free-space decay; textured meshes (tier 2: `save_3d_map`);
obstacle use for planning.
