# Tiled map, map saving and NEURAL depth — design

Date: 2026-08-29. Status: approved in conversation; supersedes the whole-map
message of the 2026-08-29 localisation design (section "Built map").

## Why

Three things the operator asked for after the first semi-autonomous runs:

1. **The whole tour stays visible.** Everything the rover has driven past is
   drawn in the Gazebo view, at obstacle-grade detail near the rover and
   as an overview further out. Today's whole-map message grows with the
   tour (a 35 × 35 m yard at 5 cm is 2 MB per message), which is why the
   map cannot both be complete and be refreshed often.
2. **Faster refresh, no flash.** Today: fused cloud 0.5 Hz → map every 2 s →
   terrain respawn every 5 s, and each respawn deletes the whole terrain
   before spawning the new one, so the ground blinks.
3. **A more accurate map.** The ZED ran in `PERFORMANCE` depth mode.

Plus, from the operator: after a ride, the option to **save the map under
a name**, and in the ground station a way to **load a saved map** — while
the live map is always built and shown, whether or not anything is saved.

## Measured facts this design rests on

Mars yard ≈ 35 × 35 m, a tour ≤ 250 m driven. WiFi is stable but shared
with the video stream; the operator wants the map traffic kept small.

Depth calibration on the Orin (nvpmodel 25 W, HD720, static scene; noise =
per-pixel std over 30 consecutive depth frames; fill = share of the image
with a stable depth):

| candidate | GPU | pose / depth Hz | fill | near noise med / p90 | far noise med / p90 |
|---|---|---|---|---|---|
| PERFORMANCE @30 (was) | 22 % | 30 / 30 | 67 % | 0.26 / 5.3 cm | 3.6 / 9.0 cm |
| ULTRA @30 | 34 % | 30 / 30 | 59 % | 0.14 / 2.2 cm | 1.8 / 13.8 cm |
| NEURAL_LIGHT @30 | 20 % | 30 / 30 | 68 % | 0.25 / 8.0 cm | 3.3 / 9.5 cm |
| NEURAL @30 | 46 % | 25 / 23 | 82 % | 0.05 / 0.36 cm | 0.75 / 3.8 cm |
| NEURAL_PLUS @30 | 50 % | 6.7 / 6.6 | 67 % | 0.04 / 0.46 cm | 0.97 / 4.4 cm |
| **NEURAL @15** | 29 % | 15 / 15 | 82 % | 0.05 / 0.42 cm | 0.70 / 15 cm |

`NEURAL` at 30 fps drops frames on this Orin (the neural model's latency,
not GPU capacity — the board drew 17 W of 25). `NEURAL` at 15 fps holds
its rate with headroom and has the NEURAL noise figures. **Decision:
`NEURAL`, 15 fps.** Consequence, accepted by the operator: the wrapper
grabs at 15 fps, so the manual-mode video is 15 fps too. Localisation at
15 Hz is ample for a rover at 0.5 m/s.

Gazebo Classic (found 2026-08-29): respawning a *heightmap* kills gzserver;
the terrain is a mesh. A mesh model is spawned/deleted through the ordinary
model path, which every world does all day long.

## Architecture

```
ZED 2i ─fused cloud 1 Hz─► elevation_mapper (Orin, 5 cm grid, 2.5 m tiles)
                              │ /localization/map_tile   (GridMap, one tile)
                              │ /localization/map_status (String JSON, 1 Hz)
                              ◄ /localization/map_command (String JSON)
        rover domain 0        │                         ▲
   ───────────────────────────┼─────────────────────────┼───────────────
        laptop                │ sim_bridge (domain 0 → 42)   rosbridge
                              ▼                              │
                       terrain_writer (sim domain)     ground station
                       one Gazebo model per tile       Map row: dropdown,
                       spawn new, then delete old      Load / Save as… / Clear
```

Everything below the line is the laptop; nothing in `ground_station/`
imports `rclpy` (rosbridge only). The ZED magnetometer stays unused.

## Rover: `elevation_mapper` (package `navi_localization`)

### Grid

`ElevationGrid` at **0.05 m** resolution, cap **60 m** a side (1200 cells,
≈ 12 MB for the height, top and count arrays — trivial). A cell's drawn
height is the **20th percentile** of that cell's z values in an update, not
the mean: a wall, a person or a flying pixel in a cell would otherwise pull
the mean into a spike, and the map has to show the ground the rover can
drive on. Each cell also keeps **`top`**, the max z of the update's points,
computed alongside the percentile and not published yet (a later change
draws obstacles from it). `elevation_mapper` clamps the elevation it
publishes — never `top`, and never the grid itself — to the rover's own
height from `/localization/pose`: `[z_rover - clamp_below, z_rover +
clamp_above]`, defaults 1.0 m / 0.5 m, no clamp until a pose has arrived.

### Tiles

The grid is partitioned into **2.5 m tiles** aligned to the map frame:
tile `(ix, iy)` covers `x ∈ [2.5·ix, 2.5·(ix+1))`, likewise y — 50 × 50
cells. Tile indices are derived from cell indices (`cell_index // 50`),
never from floats, so a cell belongs to exactly one tile.

A tile message carries **51 × 51 samples**: its own 50 × 50 cells plus one
halo row and column on the +x and +y sides, taken from the neighbouring
tiles. Adjacent tile meshes then share their boundary vertices and the
terrain has no seams. The halo is read-only data copied at publish time.

### Dirty tracking and scheduling — `TileScheduler` (pure Python, no ROS)

Per tile the scheduler keeps the elevation that was last published (a 51 ×
51 float32 array, NaN for unseen). After each grid update it marks a tile
**dirty** if any of its samples is newly finite or moved by more than
**1 cm** against what was last published.

Every scheduler tick (1 Hz):

- Publish every dirty tile whose last publication is ≥ 1 s old, oldest
  first, at most **8 per tick**. The rest wait for the next tick.
- Publish **one clean tile** in round-robin order over all tiles that have
  at least one seen cell (keepalive: a restarted sim or bridge has the
  whole yard back within `tiles` seconds and sees the rover's surroundings
  first, because those are the dirty ones).
- A tile with no seen cell is never published.

Expected traffic: driving touches ~2–6 tiles per second; a tile message is
51 × 51 × 4 B ≈ 10.4 KB → **20–70 KB/s**, independent of tour length.

### Message: `/localization/map_tile`

`grid_map_msgs/GridMap`, the type the bridge already carries:

- `header.frame_id = "map"`, stamp = now.
- `info.resolution = 0.05`, `info.length_x = info.length_y = 2.55`,
  `info.pose.position` = centre of the 51 × 51 sample lattice, z = 0.
- `layers = basic_layers = ["elevation"]`, one `Float32MultiArray` laid out
  the way `build_grid_map_message` lays out today's map (column-major,
  index (0, 0) at the largest x and y), NaN for unseen. `outer_start_index
  = inner_start_index = 0`.
- Sample `k` (0..50) of tile `ix` sits at the cell centre
  `x = 2.5·ix + (k + 0.5)·0.05`, so the lattice centre is
  `2.5·ix + 1.275`. Tile identity is not in the message; consumers derive
  it as `ix = round((pose_x − 1.275) / 2.5)` — one helper,
  `tile_index_of(pose_x, pose_y)`, in `navi_localization.tiles`, used by
  the mapper's tests and copied verbatim into `terrain_writer` (the sim
  package must not depend on the rover package).

`/localization/map` (the whole-map message) is **removed**. Its only
consumer was `terrain_writer`.

### Commands: `/localization/map_command` (`std_msgs/String`, JSON)

| command | effect |
|---|---|
| `{"action":"save","name":N}` | writes the current grid to `~/navi_maps/N.npz`. Refused if a file of that name exists unless `"overwrite":true`. |
| `{"action":"load","name":N}` | replaces the live grid with the file, marks every tile dirty (the sim redraws the yard within a few seconds), live mapping continues on top. |
| `{"action":"clear"}` | empties the grid; every previously published tile is published once more as all-NaN so the sim removes it. |

Names: `[A-Za-z0-9_-]{1,64}`, anything else refused. Every command's
outcome (ok / error text) is reported in the next status message. No
command ever touches the ZED.

File format `N.npz`: `elevation` (float32, NaN unseen), `count` (int32),
`origin_ix`, `origin_iy` (int), `resolution` (float), `saved_at` (ISO
8601 string). The grid is written whole; 60 × 60 m at 5 cm is 5.8 MB
uncompressed, `np.savez_compressed` makes it far smaller.

### Status: `/localization/map_status` (`std_msgs/String`, JSON, 1 Hz)

```json
{"resolution": 0.05, "cells_seen": 48210, "extent_m": [21.4, 17.9],
 "tiles": 71, "loaded": "yard-day1" | null,
 "maps": ["yard-day1", "test"],
 "last_command": {"action": "save", "name": "yard-day1", "ok": true,
                  "error": null, "at": "2026-08-29T15:02:11"} | null}
```

`maps` is the directory listing of `~/navi_maps/*.npz`, refreshed every
status message (cheap). The ground station's dropdown is this list.

### ZED configuration (`config/zed_front.yaml`)

`depth.depth_mode: 'NEURAL'`, `general.grab_frame_rate: 15`,
`general.pub_frame_rate: 15.0`, `mapping.resolution: 0.05`,
`mapping.fused_pointcloud_freq: 1.0`, `max_mapping_range` stays 8.0. The
launch docstring records the calibration numbers. The video sender's
`rawvideoparse framerate` is whatever the ground station asks for, so the
ground station asks for 15 (`ROVER_VIDEO_FPS` in
`ground_station/ui/main_window.py`) to match the grab rate - it used to
ask for 30, which told GStreamer that frames arrived twice as fast as
they did.

## Laptop: `sim_bridge`

Carries `/localization/map_tile:grid_map_msgs/msg/GridMap` instead of
`/localization/map`. Nothing else changes; `map_status` and `map_command`
are rosbridge traffic between the ground station and the rover and never
cross into the sim domain.

## Laptop: `terrain_writer` (package `navi_sim_bringup`)

One Gazebo model per tile. A tile's model is named `terrain_<ix>_<iy>_a`
or `…_b`, alternating with every replacement. Each received tile:

1. `terrain_mesh_from_grid` on the 51 × 51 samples (today's mesh code,
   unchanged in kind; `draw_resolution` parameter, default 0.05, subsamples
   by stride if raised) → `~/.gazebo/models/navi_terrain/meshes/tile_<ix>_<iy>_v<N>.obj`.
2. **Spawn the new model first**, and only when Gazebo confirms the spawn
   **delete the previous one** and remove its mesh file. If the spawn
   fails, the old tile stays and the error is logged. The ground never
   blinks and other tiles are never touched.
3. An all-NaN tile (from `clear`) deletes the tile's model and files.

Rate policy (`TileRespawnPolicy`, pure Python): per tile at most one
replacement per second; at most **4 spawns in flight** globally so a
keepalive burst cannot stall Gazebo; the newest payload per tile wins, an
older pending one is dropped. Unchanged payload → nothing.

`terrain_mesh.py` keeps its API; `MAX_SIDE` becomes irrelevant for 51-sample
tiles but stays as the guard it is.

## Ground station

In semi-autonomous mode, a **Map row** under the localisation readout
(pure PySide6 + roslibpy, state in `models.py`):

- **Dropdown** of saved maps from `map_status.maps`, **Load** button
  (disabled when the list is empty or nothing is selected).
- **Save as…**: `QInputDialog` for the name; refused locally when empty,
  invalid or already in the list (the rover refuses too, belt and braces).
  Sends `save`.
- **Clear**: confirmation dialog, then `clear`.
- A one-line status: `cells_seen`, extent, loaded map name, and the
  `last_command` outcome (error text in red) for 10 s after it changes.
- The row is disabled while `/localization/map_status` is stale (> 3 s,
  same rule as the localisation status).

`RosBridgeClient` gains `subscribe_map_status`, `send_map_command`;
`models.py` gains `MapState` (parsed status + staleness); the dashboard's
mode switch shows the row in semi-autonomous mode only.

## Testing

- **Rover, pure Python:** tile partition and halo; dirty detection (1 cm
  threshold, newly-finite); scheduler (8 per tick, ≥ 1 s per tile,
  round-robin keepalive, empty tiles never); message round trip through
  `tile_index_of`; save / load / clear on a temp dir including refusals
  (bad name, duplicate without overwrite) and the npz contents.
- **Rover, node level:** `map_command` → `map_status.last_command`; `load`
  republishes every tile; `clear` republishes all-NaN tiles.
- **Bridge:** the default topic list carries `map_tile` and not `map`.
- **Laptop, pure Python:** `TileRespawnPolicy`; spawn-before-delete
  ordering with fake `/spawn_entity` and `/delete_entity` (the delete must
  not be sent before the spawn future resolves; a failed spawn keeps the
  old model); all-NaN removes.
- **Laptop, end to end:** `publish_synthetic_cloud --grow` through the
  mapper (laptop copy), the real bridge and Gazebo: tiles appear, change
  every second, no blink (frame grabbed from the chase camera between two
  replacements shows terrain), gzserver alive after ≥ 60 replacements.
- **Ground station:** `MapState` parsing/staleness; Map row enabled/
  disabled; Save refuses empty/duplicate; commands sent as the JSON above
  (offscreen pytest-qt, as the existing dashboard tests).
- **Orin:** deploy, then GPU/CPU and pose/depth/cloud/tile rates with
  `NEURAL` @ 15 recorded in the launch docstring; drive a short loop and
  count tiles/s and bytes/s on `map_tile`.

## Out of scope

Traversability or any use of the map for planning; map merging between
sessions beyond "load replaces"; changing the ZED mapping range; the
organisers' scan (simulation mode is untouched).
