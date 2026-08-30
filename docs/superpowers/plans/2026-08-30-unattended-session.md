# Unattended session plan — 2026-08-30 (≈6–7 h)

Goal (PROJECT_SUMMARY.md): the Gazebo view reflects the real world as
faithfully as the ZED allows. Every item below can be built, tested on the
bench rover + laptop sim, and committed without the operator; anything that
needs a decision is parked, not guessed.

Rules: never `/manual_twist`; Orin stopped/started with `pkill -x` names;
each item ends with tests green, a commit, and a bench check where the rover
is involved. Settings stay as rolled back in `8d2612f`.

## 1. Tracking sanity guard (≈1 h) — bug seen today
On a restart with the desk <1 m away the ZED's VIO blew up: pose z −1764 m,
"distance travelled" 4.8 km, status still **OK**, mapper clamped every tile
away (0 tiles) and fused garbage into the map.
- `localization_status`: a pose jump > 2 m between consecutive messages, or
  |z| > 20 m, or speed > 5 m/s → state `SEARCHING` with reason
  `"pose jump"`; distance is not accumulated from such a jump.
- `elevation_mapper`: ignore clouds while status is not OK (already
  subscribed to status? if not, add), so a blow-up never lands in the map.
- Tests for both; bench check by calling `reset_pos_tracking` and watching.

## 2. After-ride textured map (tier 2 from the design) (≈3 h)
- Rover: find the wrapper's map-save service (`ros2 service list | grep
  zed`), wire `map_command save` to also request the SDK's fused point
  cloud (PLY with colour) into `~/navi_maps/<name>.ply` when available.
- Laptop `tools/mesh_from_cloud.py`: PLY → Poisson/ball-pivot mesh with
  vertex colours → OBJ (+ MTL) via Open3D in `.venv`; unit test on a
  synthetic cloud.
- `start_sim.sh --scan <obj>`: load that OBJ as a static, visual-only model
  under the live layer in semi mode (world composition already has the
  scan slot); e2e frame.
- GS: nothing (opt-in via CLI for now; a button is a UI decision).

## 3. Incremental cut without the full snapshot (≈1 h)
`ElevationGrid.tile_windows(keys)` slices touched tiles from raw storage;
`_offer(full=False)` uses it. Perf test: touched-only offer on a 1200²
grid < 5 ms.

## 4. Obstacle hole fill on the laptop (≈1 h)
`obstacle_mesh`: a missing voxel with ≥ 4 of 6 face-neighbours occupied is
drawn (one pass, per tile, before hidden-face removal). Test + e2e frame.

## 5. Review + docs (≈45 min)
`/code-review` on the day's diff, fix what it finds, update
PROJECT_SUMMARY.md open items, leave the Orin stack running.

Parked (needs a decision): free-space decay for stale obstacles; GS button
for the textured export; iPhone-scan alignment inputs.
