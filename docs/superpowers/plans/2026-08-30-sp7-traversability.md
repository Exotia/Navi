# SP7: Tile Aggregator and Traversability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The 2.5 m map tiles the rover already publishes are stitched into one 48 m rolling window and turned into a traversability map in which **holes are lethal**. Today's obstacle voxels are positive-only — a pit is invisible to them. After this sub-project a 0.2 m pit puts cost-100 cells on its rim, in a latched `nav_msgs/OccupancyGrid` that Nav2 (SP9) consumes as its costmap seed.

**Architecture:** One new ament_python package `rover/src/navi_autonomy` with two nodes and three pure-numpy modules. `window.py` owns the rolling-window geometry (paste a tile, shift the window, drop what leaves); `traversability.py` owns the slope/step/roughness/valid kernels and the cost curve; `grid_map_io.py` owns the `grid_map_msgs/GridMap` ↔ numpy conversion in both directions. `tile_aggregator` subscribes `/localization/map_tile`, pastes into the window, publishes `/autonomy/map`; `traversability_layer` subscribes `/autonomy/map`, derives the four layers and publishes `/autonomy/traversability` (GridMap) and `/autonomy/costmap_seed` (latched OccupancyGrid).

**Tech Stack:** Python 3.10, numpy 1.21, ROS 2 Humble (`grid_map_msgs`, `nav_msgs`, `std_msgs`), colcon (ament_python), pytest.

**Spec:** `docs/superpowers/specs/autonomy-plan.md` — §5 (first two blocks), §8 SP7 row, §2 (topic graph), §9 rung 1, §12, §13.

---

## Global Constraints

Verbatim from the spec, and non-negotiable:

- **Step lethal above 0.14 m**, "computed as max neighbour difference, so it catches **negative** steps (holes) as well as positive ones." A synthetic-grid test with a **0.2 m deep pit must produce lethal cells at its rim** — on the *flat ground beside* the pit, where a positive-only kernel yields exactly zero. That single assertion is the acceptance criterion of this sub-project.
- **Slope lethal above 25°**, roughness scaled below that.
- **Resolution 0.05 m everywhere.** "Resampling smears the step edges that matter most." A tile arriving at any other resolution is dropped with a warning, never resampled.
- **Window 48 m** → 960 × 960 cells.
- Pure-math parts (stitching geometry, slope/step/roughness kernels, the cost curve) live in plain Python modules importable **without a running ROS graph** — spec §9 rung 1, "pure functions (laptop): traversability maths on synthetic grids".
- **Never publish to `/manual_twist`.** ROS-graph tests use throwaway `ROS_DOMAIN_ID` 91/92/93, never domain 0. `pkill -x` only; never `pkill -f` with a pattern that matches your own shell.
- Commits: `git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit`, explicit `git add <paths>`, never `git add -A`, never push. On `index.lock`, wait 2 s and retry — other planning and implementation agents work in this tree on other files.
- **Do not touch** `rover/start_navi.sh`, `docs/superpowers/plans/2026-08-30-sp4-*`, `-sp5-*`, `-sp6-*`, or anything under `sim/`. SP5 and SP9 both edit `start_navi.sh`; SP7 ships a launch file and lets SP9's bringup include it.

### Commands

Pure tests (no ROS graph, but `grid_map_msgs` must be importable):

```
bash -c 'source /opt/ros/humble/setup.bash &&
  PYTHONPATH=$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization:$PYTHONPATH \
  python3 -m pytest rover/src/navi_autonomy/test -q -p no:cacheprovider \
  --ignore=rover/src/navi_autonomy/test/test_autonomy_graph.py'
```

Graph test (Task 7 only, throwaway domain):

```
bash -c 'source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=91 \
  PYTHONPATH=$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization:$PYTHONPATH \
  python3 -m pytest rover/src/navi_autonomy/test/test_autonomy_graph.py -q -p no:cacheprovider'
```

Build: `bash -c 'source /opt/ros/humble/setup.bash && cd rover && colcon build --packages-up-to navi_autonomy'`.

### The `/localization/map_tile` contract (read out of the code, not assumed)

From `rover/src/navi_localization/navi_localization/elevation_mapper.py` and `.../tiles.py`:

| | |
|---|---|
| Topic | `/localization/map_tile` |
| Type | `grid_map_msgs/msg/GridMap` |
| QoS | reliable, **volatile**, `KEEP_LAST` depth **64** (`TILE_QUEUE_DEPTH = 64`) |
| Layers | exactly one, `elevation` (`LAYER = 'elevation'`); `basic_layers` the same |
| `header.frame_id` | the mapper's `frame_id` parameter, **plain `"map"`** — unlike `/localization/obstacle_tile`, a map tile carries **no** identity in its frame |
| Tile identity | recovered from `info.pose.position.{x,y}` (the tile centre) via `tiles.tile_index_of` |
| Geometry | tile `(ix, iy)` owns the 5 cm cells `[50·ix, 50·ix+50)` in x and y (`TILE_CELLS = 50`, `TILE_M = 2.5`), and is published as **51 × 51 samples**: its own 50 × 50 plus one halo row and column taken from the +x / +y neighbours' first cells, so adjacent tile meshes share boundary vertices |
| `info.resolution` | `0.05` (`elevation_grid.RESOLUTION`) |
| `info.length_x/length_y` | `51 · 0.05 = 2.55` |
| `info.pose.position` | `tile_center(ix, iy) = (2.5·ix + 1.275, 2.5·iy + 1.275)`, orientation identity |
| Index order | grid_map's own: index (0, 0) at the **largest** x and y, rows run in −x, columns in −y, `Float32MultiArray` data **column-major** (`build_tile_message` does `tile[::-1, ::-1].T` then `flatten(order='F')`) |
| `outer_start_index` / `inner_start_index` | always 0 — this mapper never uses grid_map's circular buffer |
| Cell value | the **20th percentile** of that cell's z values in the last update, clamped to `[z_rover − 1.0, z_rover + 0.5]`, in the `map` frame |
| Unseen cells | **NaN**. An all-NaN tile is a deliberate message: `_queue_nan` publishes one when a tile vanishes, to tell consumers to erase it |
| Rate | one `_tick` per second; up to `MAX_DIRTY_PER_TICK = 8` dirty tiles + `NAN_TILES_PER_TICK = 16` blanks + 1 round-robin keepalive per tick. Measured on the Orin 2026-08-29, rover static: **5.0–6.2 Hz aggregate, 65–95 KB/s, with a 714 KB/s burst for the first ~10 s** |

The storage convention on the numpy side of this repo is the opposite of grid_map's and is the one every module here uses: **row 0 is the smallest y, column 0 the smallest x, both ascending** (`elevation_grid.py`'s docstring). The flip happens once, in `grid_map_io.py`.

### Design decisions

1. **Python + numpy, not C++.** Measured on this laptop, 2026-08-30: the full 960 × 960 derive is **≈150 ms** (step 35 ms, slope 49 ms, roughness 21 ms, valid 3 ms, cost 41 ms); the stitch of 400 tiles is **4 ms**. At the 1 Hz the elevation mapper runs at, that is ~15 % of one laptop core. The Orin's ARM cores are 2–3× slower single-thread, so budget 0.3–0.5 s per tick — comfortable at 1 Hz, and SP9/SP10 measure it on the Orin (spec §5, §11 risk 6); SP12 re-measures it in the yard. C++ would buy a factor the node does not need and would cost the pure-function testing ladder rung §9 gives us for free in Python. The repo's whole rover side is `rclpy` + numpy; matching it is worth more than the milliseconds.
2. **One package, `navi_autonomy`, holding both nodes.** They share all three pure modules, ship and launch together, and SP5/SP8/SP10 add `mode_supervisor`, `navi_rpc_server` and `twist_shaper` to the same place. Two packages would duplicate `package.xml`, `setup.py` and the grid_map dependency for no separation anyone needs. It **depends on `navi_localization`** (same colcon workspace, same `deploy_rover.sh` rsync) so the tile geometry has exactly one definition — unlike the sim, which had to copy `tile_index_of` verbatim because it lives in a different workspace.
3. **Rolling window.** 960 × 960 float32 (3.7 MB), origin tracked as integer 5 cm lattice indices. Recentre when the rover is more than **8.0 m (160 cells)** from the window centre in x or y, snapping the rover back to the centre; below that the origin never moves, so the common case is a pure paste. Cells that leave the window are **discarded** — they return only via the mapper's round-robin keepalive (~1 tile/s), which is acceptable because a 48 m window against the mapper's 60 m cap means a shift only ever drops the far frontier, and because Nav2 keeps its own costmap history. **NaN policy:** a never-seen cell stays NaN, and `valid` is 0 there, and the seed publishes `-1` (unknown) — never 0 (free). Unseen ground is not driveable ground.
4. **Cost curve** (exact, in this order):
   ```
   s = clip(slope     / radians(25.0), 0, 1)
   t = clip(step      / 0.14,          0, 1)
   r = clip(roughness / 0.05,          0, 1)      # ROUGHNESS_REF_M, one cell of vertical deviation from the local plane
   cost = round(99 * max(s, t, r))                # 0..99 — 100 is reserved for lethal
   cost = -1     where valid == 0                 # unknown
   cost = 100    where step >= 0.14 or slope >= radians(25.0)     # applied LAST
   ```
   `max`, not a mean: one bad indicator must not be averaged away by two good ones. **Lethal is applied after unknown on purpose** — a cell whose measured step is already lethal stays lethal even if its neighbourhood is incomplete. That is the safe direction, and it is exactly the case at the frontier of a hole.
5. **QoS.**
   - `/localization/map_tile` subscription: reliable, volatile, `KEEP_LAST` **depth 64**, matching the publisher's `TILE_QUEUE_DEPTH` exactly. The mapper can emit 25 tiles in a single tick (8 dirty + 16 blanks + 1 keepalive), and a map load marks all ~576 tiles dirty; the start-of-run burst was measured at 714 KB/s. A shallow queue drops tiles silently and leaves holes in the window that are indistinguishable from unseen ground. Durability must be volatile or the match fails and **no data arrives at all** — the lesson already written into `elevation_mapper`'s publisher comment.
   - `/localization/pose` subscription: default, depth 1 (the publisher is reliable/volatile depth 10).
   - `/autonomy/map`: reliable, **transient_local**, depth 1 — `traversability_layer` may start after the aggregator and must not wait a tick for a map.
   - `/autonomy/traversability`: reliable, volatile, depth 1. 4 layers × 3.7 MB = 14.7 MB per message, so it is published **only when `count_subscribers` > 0** — the same guard the ZED wrapper uses for the fused cloud. On the rover nothing subscribes to it; the view and the sim do.
   - `/autonomy/costmap_seed`: reliable, **transient_local**, depth 1 — latched, as the spec requires. 0.92 MB, always published.

### Spec ambiguities, and the rulings

- **"roughness scaled below that"** — the spec gives no roughness threshold. Ruled: roughness is never lethal on its own; it contributes to the scaled 0–99 band through `r = clip(roughness / 0.05, 0, 1)`. `ROUGHNESS_REF_M = 0.05` (one cell of deviation from the local plane) is a starting number, tuned in SP12 against real rocks, and lives in one named constant.
- **Roughness must not be a second slope measurement.** Ruled: `roughness = |e − mean(finite 4-neighbours)|`, which is **exactly zero on a plane of any inclination** and non-zero only for curvature and noise. A local standard deviation would have made every 20° slope "rough" and double-counted it in the `max`.
- **The map topic between the two nodes is unnamed in the spec.** Ruled: `/autonomy/map`, `grid_map_msgs/GridMap`, one `elevation` layer, same encoding as a tile.
- **The tile's halo row and column are ignored.** A tile is published as 51 × 51 so the simulation's adjacent meshes share boundary vertices; the 51st row and column are copies of the +x / +y neighbours' first cells. Ruled: the aggregator pastes only the tile's own 50 × 50. Merging the halo would let a **stale tile overwrite a fresher neighbour** whenever the two arrive out of order — and they do, because the mapper's scheduler sends dirty tiles oldest-first plus one round-robin keepalive. The cost is one cell at the very frontier of the mapped area, which `valid` already marks unknown.
- **An unseen cell next to seen ground is not lethal.** A hole the ZED cannot see into arrives as NaN, not as a depth. Ruled: NaN neighbours are treated as *missing measurements* — the step kernel ignores them, `valid` is 0, and the cell publishes as `-1` (unknown), not 100. What stops the rover at an unseen frontier is Nav2 refusing unknown space plus §5's forward `VoxelLayer` and collision monitor, not a guess made here. A pit the ZED *can* see into — the realistic case for a 0.2 m pit at a few metres — is lethal, and that is what the acceptance test asserts.
- **`start_navi.sh` is not modified.** SP5 and SP9 both edit it; a concurrent edit is a merge conflict for no gain. SP7 ships `autonomy_perception.launch.py`; SP9's Nav2 bringup includes it.

### Dependency reality check

`ros-humble-grid-map-msgs`, `-core`, `-ros`, `-cv` and `-costmap-2d` are **already installed on this laptop** (checked with `dpkg -l | grep grid-map`, all 2.0.1-1jammy). Nothing needs installing to build or test this plan; `navi_localization` already declares `<depend>grid_map_msgs</depend>`. The Orin has no internet, so `ros-humble-grid-map-costmap-2d` and its dependencies must be **carried over as debs** — that is spec §13's operator action and a deploy step recorded in Task 8, not a design constraint. `navi_autonomy` itself needs only `grid_map_msgs`, which the Orin already has because `navi_localization` runs there.

---

### Task 1: `navi_autonomy` package scaffold and `window.py` (pure numpy)

**Files:**
- Create `rover/src/navi_autonomy/package.xml`
- Create `rover/src/navi_autonomy/setup.py`
- Create `rover/src/navi_autonomy/setup.cfg`
- Create `rover/src/navi_autonomy/resource/navi_autonomy` (empty file)
- Create `rover/src/navi_autonomy/navi_autonomy/__init__.py` (empty file)
- Create `rover/src/navi_autonomy/navi_autonomy/window.py`
- Test: create `rover/src/navi_autonomy/test/test_window.py`

**Interfaces:**
- Consumes: `navi_localization.elevation_grid.RESOLUTION` (0.05), `navi_localization.tiles.TILE_CELLS` (50), `TILE_SAMPLES` (51), `tile_index_of`.
- Produces: `window.WINDOW_M = 48.0`, `WINDOW_CELLS = 960`, `RECENTRE_MARGIN_M = 8.0`, `RECENTRE_MARGIN_CELLS = 160`, `cell_index_of(x, resolution=RESOLUTION) -> int`, `class RollingWindow(cells=WINDOW_CELLS, resolution=RESOLUTION)` with `.elevation` `(cells, cells)` float32 NaN-filled, `.origin_ix`, `.origin_iy`, `.center -> (x, y)`, `.paste_tile(ix, iy, tile) -> None`, `.recentre(pose_x, pose_y) -> bool`, `.snapshot() -> np.ndarray`.

**Steps:**

- [ ] Write the package scaffold. `rover/src/navi_autonomy/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://relaxng.org/ns/structure/1.0"?>
<package format="3">
  <name>navi_autonomy</name>
  <version>0.1.0</version>
  <description>Orin-side autonomy perception: the 2.5 m map tiles stitched
  into a 48 m rolling window, and the traversability layers derived from it -
  slope, step (negative steps included, so holes are lethal), roughness and
  valid - published as a GridMap and as a latched OccupancyGrid costmap seed
  for Nav2.</description>
  <maintainer email="oxe.pxs@gmail.com">star</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>nav_msgs</depend>
  <depend>std_msgs</depend>
  <depend>grid_map_msgs</depend>
  <depend>navi_localization</depend>
  <exec_depend>python3-numpy</exec_depend>

  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```
  `setup.py`:
```python
import os
from glob import glob

from setuptools import setup

package_name = 'navi_autonomy'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='star',
    maintainer_email='oxe.pxs@gmail.com',
    description='Tile aggregation and traversability for the Orin.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tile_aggregator = navi_autonomy.tile_aggregator:main',
            'traversability_layer = navi_autonomy.traversability_layer:main',
        ],
    },
)
```
  `setup.cfg`:
```
[develop]
script_dir=$base/lib/navi_autonomy
[install]
install_scripts=$base/lib/navi_autonomy
```
  `resource/navi_autonomy` and `navi_autonomy/__init__.py` are both empty files.

- [ ] Write the failing test `rover/src/navi_autonomy/test/test_window.py`:
```python
"""The rolling window's geometry - pure numpy, no ROS.

  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization \
    python3 -m pytest rover/src/navi_autonomy/test/test_window.py -q'
"""
import numpy as np
import pytest

from navi_autonomy.window import (
    RECENTRE_MARGIN_CELLS, WINDOW_CELLS, RollingWindow, cell_index_of)


def tile(value=1.0):
    """A 51 x 51 tile whose own 50 x 50 is `value` and whose halo is NaN."""
    out = np.full((51, 51), np.nan, dtype=np.float32)
    out[:50, :50] = value
    return out


def test_cell_index_floors_towards_minus_infinity():
    assert cell_index_of(0.0) == 0
    assert cell_index_of(0.049) == 0
    assert cell_index_of(0.05) == 1
    assert cell_index_of(-0.01) == -1
    assert cell_index_of(-0.05) == -1


def test_a_fresh_window_is_all_unseen_and_centred_on_the_origin():
    w = RollingWindow()
    assert w.elevation.shape == (WINDOW_CELLS, WINDOW_CELLS)
    assert not np.isfinite(w.elevation).any()
    assert w.origin_ix == -WINDOW_CELLS // 2
    assert w.origin_iy == -WINDOW_CELLS // 2
    assert w.center == pytest.approx((0.0, 0.0))


def test_a_tile_lands_on_the_cells_it_owns():
    w = RollingWindow(cells=200)          # origin at lattice cell -100
    w.paste_tile(0, 0, tile(2.0))         # tile (0, 0) owns cells [0, 50)
    assert w.elevation[100:150, 100:150] == pytest.approx(2.0)
    assert not np.isfinite(w.elevation[99, 100])
    assert not np.isfinite(w.elevation[150, 100])
    assert not np.isfinite(w.elevation[100, 150])


def test_a_negative_tile_index_lands_on_negative_cells():
    w = RollingWindow(cells=200)
    w.paste_tile(-1, -2, tile(3.0))       # cells [-50, 0) in x, [-100, -50) in y
    assert w.elevation[0:50, 50:100] == pytest.approx(3.0)


def test_the_halo_row_and_column_are_ignored():
    """The 51st row and column are copies of the +x / +y neighbours' first
    cells, and those neighbours publish their own tiles. Merging a halo here
    would let a stale tile overwrite a fresher neighbour whenever the two
    arrive in the wrong order."""
    w = RollingWindow(cells=200)
    halo = tile(4.0)
    halo[:, 50] = 9.0                     # the +x neighbour's first column
    halo[50, :] = 9.0                     # the +y neighbour's first row
    w.paste_tile(0, 0, halo)
    assert np.allclose(w.elevation[100:150, 100:150], 4.0)
    assert not np.isfinite(w.elevation[100, 150])        # the +x neighbour's cell
    assert not np.isfinite(w.elevation[150, 100])        # the +y neighbour's cell
    w.paste_tile(1, 0, tile(5.0))                        # the neighbour's own message
    assert w.elevation[100, 150] == pytest.approx(5.0)
    w.paste_tile(0, 0, halo)                             # a stale (0, 0) again
    assert w.elevation[100, 150] == pytest.approx(5.0)   # and it stays the neighbour's


def test_an_all_nan_tile_blanks_the_tile_it_names():
    w = RollingWindow(cells=200)
    w.paste_tile(0, 0, tile(2.0))
    w.paste_tile(0, 0, np.full((51, 51), np.nan, dtype=np.float32))
    assert not np.isfinite(w.elevation[100:150, 100:150]).any()


def test_a_tile_outside_the_window_is_dropped_without_raising():
    w = RollingWindow(cells=200)
    w.paste_tile(40, 40, tile(1.0))       # cells [2000, 2050): nowhere near
    assert not np.isfinite(w.elevation).any()


def test_a_tile_straddling_the_edge_is_clipped():
    w = RollingWindow(cells=200)          # cells [-100, 100)
    w.paste_tile(1, 0, tile(7.0))         # cells [50, 100) in x - the last 50
    assert w.elevation[100:150, 150:200] == pytest.approx(7.0)
    w.paste_tile(2, 0, tile(8.0))         # cells [100, 150): entirely outside
    assert np.nanmax(w.elevation) == pytest.approx(7.0)
    assert np.nanmin(w.elevation) == pytest.approx(7.0)


def test_a_wrong_shaped_tile_raises():
    w = RollingWindow(cells=200)
    with pytest.raises(ValueError):
        w.paste_tile(0, 0, np.zeros((50, 50), dtype=np.float32))


def test_the_window_does_not_move_until_the_rover_is_far_from_its_centre():
    w = RollingWindow()
    before = (w.origin_ix, w.origin_iy)
    assert w.recentre(7.9, -7.9) is False
    assert (w.origin_ix, w.origin_iy) == before


def test_the_window_recentres_on_the_rover_and_carries_its_cells_along():
    w = RollingWindow()                        # 960 cells, origin -480
    w.paste_tile(0, 0, tile(6.0))              # lattice cells [0, 50)
    moved = w.recentre(10.0, 0.0)              # 200 cells: past the 160-cell margin
    assert moved is True
    assert w.origin_ix == cell_index_of(10.0) - WINDOW_CELLS // 2 == -280
    assert w.origin_iy == -WINDOW_CELLS // 2   # y never moved
    # The same ground, at its new place in the window.
    row, column = 0 - w.origin_iy, 0 - w.origin_ix
    assert w.elevation[row:row + 50, column:column + 50] == pytest.approx(6.0)
    assert RECENTRE_MARGIN_CELLS == 160


def test_cells_that_leave_the_window_are_gone():
    w = RollingWindow(cells=200)
    w.paste_tile(0, 0, tile(6.0))
    w.recentre(60.0, 0.0)                      # 1200 cells away: nothing survives
    assert not np.isfinite(w.elevation).any()


def test_snapshot_is_a_copy():
    w = RollingWindow(cells=200)
    w.paste_tile(0, 0, tile(1.0))
    snap = w.snapshot()
    w.paste_tile(0, 0, tile(2.0))
    assert snap[100, 100] == pytest.approx(1.0)
```

- [ ] Run it and watch it fail. `bash -c 'source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization:$PYTHONPATH python3 -m pytest rover/src/navi_autonomy/test/test_window.py -q -p no:cacheprovider'` → expect `ModuleNotFoundError: No module named 'navi_autonomy.window'`, 0 passed, collection error.

- [ ] Write `rover/src/navi_autonomy/navi_autonomy/window.py`:
```python
"""The 48 m rolling window the map tiles are stitched into.

Pure numpy: no rclpy, no message types. The geometry is the part that can be
wrong in a way a picture would not show - a tile pasted one cell off looks
like terrain either way - so it is the part that gets tested on a laptop in
milliseconds.

Storage convention, which is *not* grid_map's and is the same one
navi_localization.elevation_grid uses: row 0 is the smallest y, column 0 the
smallest x, both ascending. The conversion into grid_map's own convention
happens once, in navi_autonomy.grid_map_io.

Origin: `origin_ix` / `origin_iy` are the 5 cm lattice indices of column 0
and row 0, on the same lattice the mapper uses (cell k covers
[k * 0.05, (k + 1) * 0.05)). Integers, so a window shift is exact and no
float error ever accumulates into a half-cell offset.
"""

import math

import numpy as np

from navi_localization.elevation_grid import RESOLUTION
from navi_localization.tiles import TILE_CELLS, TILE_SAMPLES

# Spec section 5: "stitches tiles into a rolling window around the rover
# (48 m, so the 60 m map cap is never the binding constraint)".
WINDOW_M = 48.0
WINDOW_CELLS = int(round(WINDOW_M / RESOLUTION))     # 960

# How far the rover may drift from the window centre before the window is
# shifted back under it. 8 m of a 24 m half-width: a shift only ever drops
# ground at the far frontier, and at rover speeds it happens after tens of
# metres of travel rather than every tick.
RECENTRE_MARGIN_M = 8.0
RECENTRE_MARGIN_CELLS = int(round(RECENTRE_MARGIN_M / RESOLUTION))   # 160


def cell_index_of(x: float, resolution: float = RESOLUTION) -> int:
    """The lattice index of the cell containing `x`. Floor, not round: cell k
    covers [k * resolution, (k + 1) * resolution), including for negative x,
    which `int()` would truncate towards zero and get wrong."""
    return int(math.floor(x / resolution))


class RollingWindow:
    """A fixed-size elevation window that slides over the map lattice."""

    def __init__(self, cells: int = WINDOW_CELLS, resolution: float = RESOLUTION):
        self.cells = int(cells)
        self.resolution = float(resolution)
        self.elevation = np.full((self.cells, self.cells), np.nan, dtype=np.float32)
        self.origin_ix = -(self.cells // 2)
        self.origin_iy = -(self.cells // 2)

    @property
    def center(self) -> tuple:
        return ((self.origin_ix + self.cells / 2.0) * self.resolution,
                (self.origin_iy + self.cells / 2.0) * self.resolution)

    def snapshot(self) -> np.ndarray:
        return self.elevation.copy()

    # -- pasting ----------------------------------------------------------

    def _clip(self, y0: int, x0: int, block: np.ndarray):
        """Where `block`, with its (0, 0) at window cell (y0, x0), overlaps
        the window: (dst_y, dst_x, src_y, src_x) slices, or None."""
        height, width = block.shape
        dy0, sy0 = (y0, 0) if y0 >= 0 else (0, -y0)
        dx0, sx0 = (x0, 0) if x0 >= 0 else (0, -x0)
        take_y = min(self.cells - dy0, height - sy0)
        take_x = min(self.cells - dx0, width - sx0)
        if take_y <= 0 or take_x <= 0:
            return None
        return (slice(dy0, dy0 + take_y), slice(dx0, dx0 + take_x),
                slice(sy0, sy0 + take_y), slice(sx0, sx0 + take_x))

    def _replace(self, y0: int, x0: int, block: np.ndarray) -> None:
        got = self._clip(y0, x0, block)
        if got is not None:
            dst_y, dst_x, src_y, src_x = got
            self.elevation[dst_y, dst_x] = block[src_y, src_x]

    def paste_tile(self, ix: int, iy: int, tile) -> None:
        """One `/localization/map_tile` in the storage convention.

        Tile (ix, iy) owns lattice cells [50 ix, 50 ix + 50) in both axes and
        arrives as 51 x 51 samples: its own 50 x 50 plus a halo row and column
        copied from the +x / +y neighbours' first cells.

        The own 50 x 50 is replaced **wholesale, NaN included**, which is what
        makes the mapper's all-NaN "this tile is gone" message
        (elevation_mapper._queue_nan) erase what it named, and what makes a
        cell that stopped being seen stop being seen here too.

        **The halo is ignored.** It exists so the simulation's adjacent tile
        meshes share boundary vertices; every cell in it belongs to a
        neighbouring tile that the mapper publishes in its own right. Merging
        it here would let a stale tile overwrite a fresher neighbour whenever
        the two arrive in the wrong order - and tiles do arrive out of order,
        since the scheduler sends dirty tiles oldest-first and one round-robin
        keepalive. The cost is one cell at the very frontier of the mapped
        area, which `valid` marks unknown anyway.
        """
        tile = np.asarray(tile, dtype=np.float32)
        if tile.shape != (TILE_SAMPLES, TILE_SAMPLES):
            raise ValueError(
                f"a map tile is {TILE_SAMPLES}x{TILE_SAMPLES} samples, got {tile.shape}")
        x0 = TILE_CELLS * int(ix) - self.origin_ix
        y0 = TILE_CELLS * int(iy) - self.origin_iy
        self._replace(y0, x0, tile[:TILE_CELLS, :TILE_CELLS])

    # -- sliding ----------------------------------------------------------

    def recentre(self, pose_x: float, pose_y: float) -> bool:
        """Slide the window so the rover is back at its centre, but only once
        the rover is more than RECENTRE_MARGIN_M from that centre. True if it
        moved. Cells that leave the window are discarded: they come back only
        when the mapper's round-robin keepalive republishes their tile."""
        want_ix = cell_index_of(pose_x, self.resolution) - self.cells // 2
        want_iy = cell_index_of(pose_y, self.resolution) - self.cells // 2
        if (abs(want_ix - self.origin_ix) < RECENTRE_MARGIN_CELLS and
                abs(want_iy - self.origin_iy) < RECENTRE_MARGIN_CELLS):
            return False
        self._shift(want_ix - self.origin_ix, want_iy - self.origin_iy)
        return True

    def _shift(self, dx: int, dy: int) -> None:
        moved = np.full_like(self.elevation, np.nan)
        n = self.cells
        src_x0, dst_x0 = (dx, 0) if dx >= 0 else (0, -dx)
        src_y0, dst_y0 = (dy, 0) if dy >= 0 else (0, -dy)
        width, height = n - abs(dx), n - abs(dy)
        if width > 0 and height > 0:
            moved[dst_y0:dst_y0 + height, dst_x0:dst_x0 + width] = \
                self.elevation[src_y0:src_y0 + height, src_x0:src_x0 + width]
        self.elevation = moved
        self.origin_ix += dx
        self.origin_iy += dy
```

- [ ] Run again → expect `13 passed`.

- [ ] Commit: `git add rover/src/navi_autonomy && git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "A navi_autonomy package, and the 48 m rolling window the map tiles are stitched into"`.

---

### Task 2: the traversability kernels — holes become lethal (pure numpy)

**Files:**
- Create `rover/src/navi_autonomy/navi_autonomy/traversability.py`
- Test: create `rover/src/navi_autonomy/test/test_traversability.py`

**Interfaces:**
- Consumes: nothing but numpy (`RESOLUTION` from `navi_localization.elevation_grid`).
- Produces: `STEP_LETHAL_M = 0.14`, `SLOPE_LETHAL_DEG = 25.0`, `SLOPE_LETHAL_RAD`, `ROUGHNESS_REF_M = 0.05`, `step_layer(elevation) -> (rows, cols) float32`, `slope_layer(elevation, resolution=RESOLUTION) -> float32 radians`, `roughness_layer(elevation) -> float32 metres`, `valid_layer(elevation) -> float32 0.0/1.0`, `derive(elevation, resolution=RESOLUTION) -> dict[str, np.ndarray]` with keys `slope`, `step`, `roughness`, `valid`.

**Steps:**

- [ ] Write the failing test `rover/src/navi_autonomy/test/test_traversability.py`:
```python
"""The traversability maths on synthetic grids, holes included - spec section 9
rung 1. Pure numpy, no ROS.

  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization \
    python3 -m pytest rover/src/navi_autonomy/test/test_traversability.py -q'
"""
import math

import numpy as np
import pytest

from navi_autonomy.traversability import (
    ROUGHNESS_REF_M, SLOPE_LETHAL_DEG, SLOPE_LETHAL_RAD, STEP_LETHAL_M, derive,
    roughness_layer, slope_layer, step_layer, valid_layer)


def pit(depth=0.2, size=6, extent=24):
    """Flat ground at z = 0 with a `size` x `size` pit `depth` deep in the
    middle. The ZED sees into it, so its floor is measured, not NaN - which is
    the realistic case for a 0.2 m pit at a few metres."""
    grid = np.zeros((extent, extent), dtype=np.float32)
    lo = (extent - size) // 2
    grid[lo:lo + size, lo:lo + size] = -depth
    return grid, lo, size


def plane(degrees, extent=20, resolution=0.05):
    xs = np.arange(extent, dtype=np.float32) * resolution
    return np.tile((xs * math.tan(math.radians(degrees))).astype(np.float32), (extent, 1))


def test_the_thresholds_are_the_spec_numbers():
    assert STEP_LETHAL_M == 0.14
    assert SLOPE_LETHAL_DEG == 25.0
    assert SLOPE_LETHAL_RAD == pytest.approx(math.radians(25.0))


# -- the acceptance criterion of this sub-project ------------------------

def test_a_pit_makes_lethal_step_on_the_flat_ground_around_its_rim():
    """The point of SP7. The obstacle voxels are positive-only, so a hole is
    invisible to them; a max *absolute* neighbour difference makes the flat
    cells beside a 0.2 m pit lethal, because the ground beside them drops
    away. A positive-only kernel scores those same cells exactly zero."""
    grid, lo, size = pit()
    step = step_layer(grid)

    # A flat cell diagonally off the pit's corner, and one along its edge.
    assert step[lo - 1, lo - 1] == pytest.approx(0.2)
    assert step[lo - 1, lo + 2] == pytest.approx(0.2)
    assert step[lo - 1, lo - 1] > STEP_LETHAL_M
    # A positive-only kernel would see nothing there:
    rise = np.zeros_like(grid)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy == 0 and dx == 0:
                continue
            shifted = np.roll(np.roll(grid, dy, axis=0), dx, axis=1)
            rise = np.maximum(rise, shifted - grid)
    assert rise[lo - 1, lo - 1] == pytest.approx(0.0)


def test_the_pit_floor_is_lethal_at_its_edge_and_free_in_its_middle():
    grid, lo, size = pit(size=6)
    step = step_layer(grid)
    assert step[lo, lo] == pytest.approx(0.2)                 # floor, against the wall
    assert step[lo + 2, lo + 2] == pytest.approx(0.0)         # floor, interior
    assert step[2, 2] == pytest.approx(0.0)                   # far flat ground


def test_a_positive_step_is_lethal_too():
    """A rock, for symmetry with the pit: the ground *around* it and its own
    edge are lethal, the flat top of it is not."""
    grid = np.zeros((20, 20), dtype=np.float32)
    grid[9:12, 9:12] = 0.3
    step = step_layer(grid)
    assert step[8, 8] == pytest.approx(0.3)      # ground beside the rock
    assert step[9, 9] == pytest.approx(0.3)      # the rock's own edge
    assert step[10, 10] == pytest.approx(0.0)    # the middle of its flat top
    assert step[8, 8] > STEP_LETHAL_M


def test_a_step_just_under_the_threshold_is_not_lethal():
    grid = np.zeros((20, 20), dtype=np.float32)
    grid[10:, :] = -0.13
    step = step_layer(grid)
    assert step[9, 5] == pytest.approx(0.13)
    assert step[9, 5] < STEP_LETHAL_M


def test_step_ignores_unseen_neighbours_and_is_nan_where_unseen():
    grid = np.zeros((10, 10), dtype=np.float32)
    grid[5, 5] = np.nan
    step = step_layer(grid)
    assert not np.isfinite(step[5, 5])
    assert step[4, 5] == pytest.approx(0.0)      # the NaN neighbour is ignored


def test_step_does_not_wrap_around_the_grid_edge():
    grid = np.zeros((10, 10), dtype=np.float32)
    grid[0, :] = 1.0
    step = step_layer(grid)
    assert step[9, 5] == pytest.approx(0.0)      # row 9 must not see row 0


# -- slope ---------------------------------------------------------------

def test_slope_of_a_plane_is_its_inclination_everywhere_including_the_edge():
    grid = plane(20.0)
    slope = np.degrees(slope_layer(grid))
    assert slope[10, 10] == pytest.approx(20.0, abs=1e-3)
    assert slope[10, 0] == pytest.approx(20.0, abs=1e-3)     # one-sided difference
    assert slope[10, 19] == pytest.approx(20.0, abs=1e-3)


def test_slope_of_flat_ground_is_zero():
    assert np.degrees(slope_layer(np.zeros((10, 10), dtype=np.float32)))[5, 5] \
        == pytest.approx(0.0)


def test_a_30_degree_plane_is_over_the_slope_threshold_and_25_is_not_under_it():
    assert slope_layer(plane(30.0))[10, 10] > SLOPE_LETHAL_RAD
    assert slope_layer(plane(20.0))[10, 10] < SLOPE_LETHAL_RAD


# -- roughness -----------------------------------------------------------

def test_roughness_of_any_plane_is_zero_so_it_is_not_a_second_slope():
    for degrees in (0.0, 10.0, 20.0, 35.0):
        assert roughness_layer(plane(degrees))[10, 10] == pytest.approx(0.0, abs=1e-6)


def test_roughness_measures_a_bump():
    grid = np.zeros((10, 10), dtype=np.float32)
    grid[5, 5] = 0.04
    assert roughness_layer(grid)[5, 5] == pytest.approx(0.04)
    assert ROUGHNESS_REF_M == 0.05


# -- valid ---------------------------------------------------------------

def test_valid_needs_the_cell_and_its_four_axial_neighbours():
    grid = np.zeros((10, 10), dtype=np.float32)
    grid[5, 5] = np.nan
    valid = valid_layer(grid)
    assert valid[5, 5] == 0.0
    assert valid[4, 5] == 0.0        # a NaN neighbour
    assert valid[3, 3] == 1.0
    assert valid[0, 3] == 0.0        # the grid edge has no south neighbour


def test_derive_returns_the_four_layers_at_the_input_shape():
    grid, _, _ = pit()
    layers = derive(grid)
    assert set(layers) == {'slope', 'step', 'roughness', 'valid'}
    for name, array in layers.items():
        assert array.shape == grid.shape, name
        assert array.dtype == np.float32, name
```

- [ ] Run and watch it fail: `bash -c 'source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization:$PYTHONPATH python3 -m pytest rover/src/navi_autonomy/test/test_traversability.py -q -p no:cacheprovider'` → `ModuleNotFoundError: No module named 'navi_autonomy.traversability'`.

- [ ] Write `rover/src/navi_autonomy/navi_autonomy/traversability.py`:
```python
"""Slope, step, roughness and valid from an elevation grid - pure numpy.

The point of this module, and of the whole sub-project, is the sign of a
step. The obstacle voxels (navi_localization.voxels) are positive-only: they
are things sticking up out of the ground, so a hole is invisible to them and
the rover would plan straight into one. `step_layer` is the **maximum
absolute** difference to any of the eight neighbours, so the flat ground
around the rim of a pit scores the pit's depth and goes lethal, exactly as
the ground around a rock does.

No np.roll anywhere: rolling wraps the far edge of the grid onto the near
one, and a 48 m window has a real edge where the map runs out. Every
neighbour is a slice of a NaN-padded copy instead, so the edge behaves like
unseen ground, which is what it is.

Unseen cells are NaN throughout. A NaN neighbour is a *missing measurement*,
not a hole: the kernels ignore it and `valid` reports 0 there, so the seed
publishes unknown rather than guessing. What stops the rover at an unseen
frontier is Nav2 refusing unknown space and the forward-looking collision
monitor, not a guess made here.
"""

import math

import numpy as np

from navi_localization.elevation_grid import RESOLUTION

# Spec section 5. "step lethal above 0.14 m - just over the 0.125 m wheel
# radius. Computed as max neighbour difference, so it catches negative steps
# (holes) as well as positive ones." "slope lethal above 25 degrees,
# roughness scaled below that."
STEP_LETHAL_M = 0.14
SLOPE_LETHAL_DEG = 25.0
SLOPE_LETHAL_RAD = math.radians(SLOPE_LETHAL_DEG)

# Roughness is never lethal on its own; the spec only asks for it to be
# "scaled below" the slope threshold. This is the deviation from the local
# plane at which a cell reaches the top of the scaled cost band: one cell of
# vertical deviation. A starting number, tuned in SP12 against real rocks.
ROUGHNESS_REF_M = 0.05

_EIGHT = ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1))
_FOUR = ((-1, 0), (1, 0), (0, -1), (0, 1))


def _as_grid(elevation) -> np.ndarray:
    grid = np.asarray(elevation, dtype=np.float32)
    if grid.ndim != 2:
        raise ValueError(f"an elevation grid is 2-D, got shape {grid.shape}")
    return grid


def _padded(grid: np.ndarray) -> np.ndarray:
    rows, cols = grid.shape
    out = np.full((rows + 2, cols + 2), np.nan, dtype=np.float32)
    out[1:-1, 1:-1] = grid
    return out


def _shift(padded: np.ndarray, dy: int, dx: int, rows: int, cols: int) -> np.ndarray:
    """The neighbour at (dy, dx) of every cell, NaN outside the grid."""
    return padded[1 + dy:1 + dy + rows, 1 + dx:1 + dx + cols]


def step_layer(elevation) -> np.ndarray:
    """Max |height difference| to any of the eight neighbours, in metres.

    Absolute, which is the whole point: a cell whose neighbour is 0.2 m
    *below* it scores 0.2 and goes lethal. NaN where the cell itself is
    unseen; NaN neighbours contribute nothing (np.fmax ignores them)."""
    grid = _as_grid(elevation)
    rows, cols = grid.shape
    padded = _padded(grid)
    step = np.zeros((rows, cols), dtype=np.float32)
    for dy, dx in _EIGHT:
        np.fmax(step, np.abs(grid - _shift(padded, dy, dx, rows, cols)), out=step)
    return np.where(np.isfinite(grid), step, np.nan).astype(np.float32)


def _gradient(plus: np.ndarray, minus: np.ndarray, centre: np.ndarray,
              resolution: float) -> np.ndarray:
    """Central difference where both sides are seen, one-sided where only one
    is, zero where neither is. Exact on a plane in all three cases."""
    plus_ok = np.isfinite(plus)
    minus_ok = np.isfinite(minus)
    ahead = np.where(plus_ok, plus, centre)
    behind = np.where(minus_ok, minus, centre)
    span = np.where(plus_ok, resolution, 0.0) + np.where(minus_ok, resolution, 0.0)
    return ((ahead - behind) / np.where(span > 0.0, span, 1.0)).astype(np.float32)


def slope_layer(elevation, resolution: float = RESOLUTION) -> np.ndarray:
    """Ground inclination in radians: atan of the gradient magnitude."""
    grid = _as_grid(elevation)
    rows, cols = grid.shape
    padded = _padded(grid)
    gx = _gradient(_shift(padded, 0, 1, rows, cols), _shift(padded, 0, -1, rows, cols),
                   grid, resolution)
    gy = _gradient(_shift(padded, 1, 0, rows, cols), _shift(padded, -1, 0, rows, cols),
                   grid, resolution)
    slope = np.arctan(np.hypot(gx, gy)).astype(np.float32)
    return np.where(np.isfinite(grid), slope, np.nan).astype(np.float32)


def roughness_layer(elevation) -> np.ndarray:
    """|height - mean of the seen 4-neighbours|, in metres.

    Deliberately not a local standard deviation: on a 20 degree slope the
    standard deviation of a 3 x 3 window is 0.019 m, so every slope would
    also read as rough and be counted twice in the cost. This is a discrete
    Laplacian, which is **exactly zero on a plane of any inclination** and
    non-zero only for curvature and noise - which is what "roughness" is
    supposed to mean."""
    grid = _as_grid(elevation)
    rows, cols = grid.shape
    padded = _padded(grid)
    total = np.zeros((rows, cols), dtype=np.float32)
    count = np.zeros((rows, cols), dtype=np.float32)
    for dy, dx in _FOUR:
        neighbour = _shift(padded, dy, dx, rows, cols)
        seen = np.isfinite(neighbour)
        total += np.where(seen, neighbour, 0.0)
        count += seen
    rough = np.abs(grid - total / np.maximum(count, 1.0)).astype(np.float32)
    rough[count == 0] = 0.0
    return np.where(np.isfinite(grid), rough, np.nan).astype(np.float32)


def valid_layer(elevation) -> np.ndarray:
    """1.0 where the cell and all four axial neighbours are seen, else 0.0.

    That is the support the slope and roughness kernels need to mean what
    they say; without it the numbers are extrapolations from one side. The
    frontier of the mapped area is therefore *unknown*, not free."""
    grid = _as_grid(elevation)
    rows, cols = grid.shape
    padded = _padded(grid)
    ok = np.isfinite(grid)
    for dy, dx in _FOUR:
        ok &= np.isfinite(_shift(padded, dy, dx, rows, cols))
    return ok.astype(np.float32)


def derive(elevation, resolution: float = RESOLUTION) -> dict:
    """The four layers, in one pass over the window.

    Measured on the laptop, 2026-08-30, at the full 960 x 960: step 35 ms,
    slope 49 ms, roughness 21 ms, valid 3 ms - about 110 ms, against a 1 Hz
    tick. The Orin's cores are 2-3x slower single-thread; SP9/SP10 measure it
    there (spec section 5, section 11 risk 6), SP12 re-measures it in the
    yard, and the documented fallback is spec section 5's 24 m window."""
    grid = _as_grid(elevation)
    return {
        'slope': slope_layer(grid, resolution),
        'step': step_layer(grid),
        'roughness': roughness_layer(grid),
        'valid': valid_layer(grid),
    }
```

- [ ] Run again → expect `14 passed`.

- [ ] Commit: `git add rover/src/navi_autonomy/navi_autonomy/traversability.py rover/src/navi_autonomy/test/test_traversability.py && git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "Slope, step, roughness and valid from an elevation grid - a hole's rim is lethal because the step is the max absolute neighbour difference"`.

---

### Task 3: the cost curve — `/autonomy/costmap_seed` values (pure numpy)

**Files:**
- Modify `rover/src/navi_autonomy/navi_autonomy/traversability.py`
- Test: modify `rover/src/navi_autonomy/test/test_traversability.py`

**Interfaces:**
- Produces: `LETHAL = 100`, `UNKNOWN = -1`, `MAX_SCALED_COST = 99`, `costmap_seed(slope, step, roughness, valid) -> (rows, cols) int8`, `seed_from_elevation(elevation, resolution=RESOLUTION) -> (dict, int8 array)`.

**Steps:**

- [ ] Append the failing tests to `rover/src/navi_autonomy/test/test_traversability.py`, extending the existing import line to:
```python
from navi_autonomy.traversability import (
    LETHAL, MAX_SCALED_COST, ROUGHNESS_REF_M, SLOPE_LETHAL_DEG, SLOPE_LETHAL_RAD,
    STEP_LETHAL_M, UNKNOWN, costmap_seed, derive, roughness_layer, seed_from_elevation,
    slope_layer, step_layer, valid_layer)
```
```python
def test_the_seed_values_are_the_occupancy_grid_conventions():
    assert (LETHAL, UNKNOWN, MAX_SCALED_COST) == (100, -1, 99)


def test_a_pit_rim_is_lethal_in_the_seed():
    """End of the chain the whole sub-project exists for."""
    grid, lo, size = pit()
    layers, cost = seed_from_elevation(grid)
    assert cost.dtype == np.int8
    assert cost[lo - 1, lo - 1] == LETHAL
    assert cost[lo - 1, lo + 2] == LETHAL
    assert cost[lo, lo] == LETHAL                 # the floor against the wall
    assert cost[2, 2] == 0                        # far flat ground is free
    assert (cost == LETHAL).sum() == 48            # the 6x6 pit's two rings of rim


def test_never_seen_ground_is_unknown_not_free():
    grid = np.zeros((10, 10), dtype=np.float32)
    grid[5, 5] = np.nan
    _, cost = seed_from_elevation(grid)
    assert cost[5, 5] == UNKNOWN
    assert cost[4, 5] == UNKNOWN                  # incomplete neighbourhood
    assert cost[0, 3] == UNKNOWN                  # the grid edge
    assert cost[3, 3] == 0


def test_a_lethal_step_beats_an_incomplete_neighbourhood():
    """A cell can be short of support and still have a measured, lethal drop
    beside it. Lethal wins: that is the safe direction, and it is exactly the
    case at the frontier of a hole."""
    grid = np.zeros((10, 10), dtype=np.float32)
    grid[5, 5] = -0.3
    grid[5, 7] = np.nan            # takes valid away from (5, 6) and (5, 7)
    _, cost = seed_from_elevation(grid)
    assert valid_layer(grid)[5, 6] == 0.0
    assert cost[5, 6] == LETHAL


def test_slope_scales_below_the_threshold_and_is_lethal_above_it():
    _, cost_20 = seed_from_elevation(plane(20.0))
    _, cost_30 = seed_from_elevation(plane(30.0))
    assert cost_20[10, 10] == 79           # round(99 * 20/25)
    assert cost_30[10, 10] == LETHAL


def test_the_scaled_band_takes_the_worst_indicator_not_their_average():
    zeros = np.zeros((4, 4), dtype=np.float32)
    ones = np.ones((4, 4), dtype=np.float32)
    cost = costmap_seed(slope=zeros, step=np.full((4, 4), 0.07, dtype=np.float32),
                        roughness=zeros, valid=ones)
    assert cost[1, 1] == 50                # round(99 * 0.5), not a third of it


def test_flat_seen_ground_costs_nothing():
    cost = costmap_seed(slope=np.zeros((4, 4), dtype=np.float32),
                        step=np.zeros((4, 4), dtype=np.float32),
                        roughness=np.zeros((4, 4), dtype=np.float32),
                        valid=np.ones((4, 4), dtype=np.float32))
    assert (cost == 0).all()


def test_the_seed_never_emits_a_value_outside_the_occupancy_grid_range():
    rng = np.random.default_rng(7)
    grid = rng.normal(0.0, 0.3, (60, 60)).astype(np.float32)
    grid[rng.random((60, 60)) < 0.2] = np.nan
    _, cost = seed_from_elevation(grid)
    assert cost.min() >= -1 and cost.max() <= 100
    assert set(np.unique(cost)) <= set(range(-1, 101))
```

- [ ] Run → expect `ImportError: cannot import name 'costmap_seed'`.

- [ ] Append to `rover/src/navi_autonomy/navi_autonomy/traversability.py`:
```python
# nav_msgs/OccupancyGrid conventions: 0..100 cost, -1 unknown. 100 is
# reserved for lethal, so the scaled band tops out at 99 and a planner can
# tell "as bad as it gets while still driveable" from "do not".
LETHAL = 100
UNKNOWN = -1
MAX_SCALED_COST = 99


def costmap_seed(slope, step, roughness, valid) -> np.ndarray:
    """The four layers as one int8 cost grid for /autonomy/costmap_seed.

        s = clip(slope     / SLOPE_LETHAL_RAD, 0, 1)
        t = clip(step      / STEP_LETHAL_M,    0, 1)
        r = clip(roughness / ROUGHNESS_REF_M,  0, 1)
        cost = round(99 * max(s, t, r))
        cost = -1   where valid == 0
        cost = 100  where step >= STEP_LETHAL_M or slope >= SLOPE_LETHAL_RAD

    `max`, not a mean: one bad indicator must not be averaged away by two
    good ones. The order of the last two lines is deliberate - a cell with a
    measured lethal step is lethal even where its neighbourhood is too
    incomplete for `valid`, because that is the safe direction and because it
    is exactly what a hole's frontier looks like.
    """
    slope = np.nan_to_num(np.asarray(slope, dtype=np.float32), nan=0.0)
    step = np.nan_to_num(np.asarray(step, dtype=np.float32), nan=0.0)
    roughness = np.nan_to_num(np.asarray(roughness, dtype=np.float32), nan=0.0)
    valid = np.asarray(valid, dtype=np.float32)

    worst = np.maximum(
        np.maximum(np.clip(slope / SLOPE_LETHAL_RAD, 0.0, 1.0),
                   np.clip(step / STEP_LETHAL_M, 0.0, 1.0)),
        np.clip(roughness / ROUGHNESS_REF_M, 0.0, 1.0))
    cost = np.rint(MAX_SCALED_COST * worst).astype(np.int8)
    cost[valid < 0.5] = UNKNOWN
    cost[(step >= STEP_LETHAL_M) | (slope >= SLOPE_LETHAL_RAD)] = LETHAL
    return cost


def seed_from_elevation(elevation, resolution: float = RESOLUTION) -> tuple:
    """(the four layers, the int8 cost grid) - the whole derive in one call."""
    layers = derive(elevation, resolution)
    cost = costmap_seed(layers['slope'], layers['step'],
                        layers['roughness'], layers['valid'])
    return layers, cost
```

- [ ] Run → expect `22 passed` (14 from Task 2 plus these 8).

- [ ] Commit: `git add rover/src/navi_autonomy/navi_autonomy/traversability.py rover/src/navi_autonomy/test/test_traversability.py && git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "The costmap seed curve: lethal 100, unknown -1, worst-indicator scaling to 99 below the thresholds"`.

---

### Task 4: `grid_map_io.py` — the message contract in both directions

**Files:**
- Create `rover/src/navi_autonomy/navi_autonomy/grid_map_io.py`
- Test: create `rover/src/navi_autonomy/test/test_grid_map_io.py`

**Interfaces:**
- Consumes: `grid_map_msgs/msg/GridMap`, `std_msgs/msg/Float32MultiArray`, `MultiArrayDimension`, `nav_msgs/msg/OccupancyGrid`; `navi_localization.elevation_mapper.build_tile_message` (in the test only, as the round-trip's other half).
- Produces:
  - `ELEVATION_LAYER = 'elevation'`
  - `layer_from_message(message: GridMap, name: str) -> np.ndarray` — storage convention (row 0 smallest y, column 0 smallest x)
  - `tile_from_message(message: GridMap) -> (elevation (51,51) float32, ix: int, iy: int)` — raises `ValueError` on a wrong resolution, a missing `elevation` layer, or a non-zero circular-buffer index
  - `build_grid_map(layers: dict[str, np.ndarray], origin_ix, origin_iy, resolution, frame_id, stamp) -> GridMap`
  - `build_occupancy_grid(cost: np.ndarray, origin_ix, origin_iy, resolution, frame_id, stamp) -> OccupancyGrid`

**Steps:**

- [ ] Write the failing test `rover/src/navi_autonomy/test/test_grid_map_io.py`:
```python
"""The GridMap contract, both ways. The one test that stops the aggregator and
the rover's mapper drifting apart - the same job test_grid_map_round_trip.py
does for the simulation.

  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization \
    python3 -m pytest rover/src/navi_autonomy/test/test_grid_map_io.py -q'
"""
import numpy as np
import pytest
from builtin_interfaces.msg import Time

from navi_autonomy.grid_map_io import (
    ELEVATION_LAYER, build_grid_map, build_occupancy_grid, layer_from_message,
    tile_from_message)
from navi_localization.elevation_mapper import build_tile_message


def a_tile():
    tile = np.full((51, 51), np.nan, dtype=np.float32)
    tile[0, 0] = 1.0
    tile[0, 1] = 2.0
    tile[1, 0] = 4.0
    tile[25, 25] = 3.0
    tile[50, 50] = 6.0
    return tile


def test_what_the_rover_publishes_is_what_the_aggregator_reads():
    tile = a_tile()
    message = build_tile_message((3, -2), tile, 'map', Time())
    got, ix, iy = tile_from_message(message)
    assert np.array_equal(got, tile, equal_nan=True)
    assert (ix, iy) == (3, -2)


def test_a_tile_at_a_foreign_resolution_is_refused_not_resampled():
    message = build_tile_message((0, 0), a_tile(), 'map', Time())
    message.info.resolution = 0.10
    with pytest.raises(ValueError, match='resolution'):
        tile_from_message(message)


def test_a_message_without_an_elevation_layer_is_refused():
    message = build_tile_message((0, 0), a_tile(), 'map', Time())
    message.layers = ['colour']
    with pytest.raises(ValueError, match='elevation'):
        tile_from_message(message)


def test_a_circular_buffer_message_is_refused():
    message = build_tile_message((0, 0), a_tile(), 'map', Time())
    message.outer_start_index = 3
    with pytest.raises(ValueError, match='circular'):
        tile_from_message(message)


def test_a_built_grid_map_reads_back_as_what_went_in():
    rows, cols = 6, 4                       # deliberately not square
    elevation = np.arange(rows * cols, dtype=np.float32).reshape(rows, cols)
    valid = np.ones((rows, cols), dtype=np.float32)
    message = build_grid_map({'elevation': elevation, 'valid': valid},
                             origin_ix=-2, origin_iy=5, resolution=0.05,
                             frame_id='map', stamp=Time())
    assert list(message.layers) == ['elevation', 'valid']
    assert message.header.frame_id == 'map'
    assert message.info.resolution == pytest.approx(0.05)
    assert message.info.length_x == pytest.approx(cols * 0.05)
    assert message.info.length_y == pytest.approx(rows * 0.05)
    assert message.info.pose.position.x == pytest.approx((-2 + cols / 2.0) * 0.05)
    assert message.info.pose.position.y == pytest.approx((5 + rows / 2.0) * 0.05)
    assert message.info.pose.orientation.w == pytest.approx(1.0)
    assert message.outer_start_index == 0 and message.inner_start_index == 0
    assert np.array_equal(layer_from_message(message, 'elevation'), elevation)
    assert np.array_equal(layer_from_message(message, 'valid'), valid)


def test_a_built_grid_map_carries_nan_through():
    elevation = np.full((4, 4), np.nan, dtype=np.float32)
    elevation[1, 2] = 7.5
    message = build_grid_map({'elevation': elevation}, 0, 0, 0.05, 'map', Time())
    assert np.array_equal(layer_from_message(message, 'elevation'), elevation,
                          equal_nan=True)


def test_the_occupancy_grid_origin_is_the_corner_and_x_runs_fastest():
    cost = np.array([[0, 1, 2], [3, 4, 5]], dtype=np.int8)   # 2 rows (y), 3 cols (x)
    message = build_occupancy_grid(cost, origin_ix=-10, origin_iy=4,
                                   resolution=0.05, frame_id='map', stamp=Time())
    assert (message.info.width, message.info.height) == (3, 2)
    assert message.info.resolution == pytest.approx(0.05)
    assert message.info.origin.position.x == pytest.approx(-0.5)
    assert message.info.origin.position.y == pytest.approx(0.2)
    assert message.info.origin.orientation.w == pytest.approx(1.0)
    assert list(message.data) == [0, 1, 2, 3, 4, 5]


def test_the_occupancy_grid_keeps_lethal_and_unknown_intact():
    cost = np.array([[-1, 100], [0, 99]], dtype=np.int8)
    message = build_occupancy_grid(cost, 0, 0, 0.05, 'map', Time())
    assert list(message.data) == [-1, 100, 0, 99]


def test_the_layer_name_asked_for_is_the_one_returned():
    message = build_grid_map({'a': np.zeros((2, 2), dtype=np.float32),
                              'b': np.ones((2, 2), dtype=np.float32)},
                             0, 0, 0.05, 'map', Time())
    assert layer_from_message(message, 'b')[0, 0] == pytest.approx(1.0)
    with pytest.raises(ValueError, match='c'):
        layer_from_message(message, 'c')


def test_the_elevation_layer_name_matches_the_mapper():
    assert ELEVATION_LAYER == 'elevation'
```

- [ ] Run → expect `ModuleNotFoundError: No module named 'navi_autonomy.grid_map_io'`.

- [ ] Write `rover/src/navi_autonomy/navi_autonomy/grid_map_io.py`:
```python
"""grid_map_msgs/GridMap and nav_msgs/OccupancyGrid, to and from numpy.

The one place in this package that knows grid_map's index convention: index
(0, 0) at the **largest** x and y, rows running in -x, columns in -y, data
column-major. Everything else here uses the repo's storage convention -
row 0 the smallest y, column 0 the smallest x - so the flip lives here and
nowhere else.

`layer_from_message` is adapted from
sim/src/navi_sim_bringup/navi_sim_bringup/terrain_writer.py's
`elevation_from_message`, which already reads exactly these messages for the
simulation; it is generalised to any layer name and paired with a writer,
rather than reinvented. That module could not simply be imported: it lives
in the laptop's colcon workspace, not the rover's.

Why array.array and not a list or a numpy array: the generated
Float32MultiArray setter has exactly one fast path, `array.array` with type
code 'f'; anything else goes through a per-element assert. Measured on this
laptop at 960 x 960 = 921,600 floats, 2026-08-30: array.array 3.5 ms per
layer, `ndarray.tolist()` 21 ms. Serialising the whole four-layer GridMap
then costs 65 ms, and the OccupancyGrid 1.9 ms.
"""

import array

import numpy as np
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from navi_localization.elevation_grid import RESOLUTION
from navi_localization.tiles import TILE_SAMPLES, tile_index_of

ELEVATION_LAYER = 'elevation'


def layer_from_message(message: GridMap, name: str) -> np.ndarray:
    """One layer in the storage convention: row 0 the smallest y, column 0
    the smallest x."""
    if name not in message.layers:
        raise ValueError(f"no {name!r} layer in {list(message.layers)}")
    if message.outer_start_index or message.inner_start_index:
        raise ValueError(
            "the grid_map circular-buffer start indices are not zero. This "
            "reader does not unroll them, and neither elevation_mapper nor "
            "grid_map_io ever sets them.")
    layer = message.data[message.layers.index(name)]
    n_cols = layer.layout.dim[0].size
    n_rows = layer.layout.dim[1].size
    grid = np.asarray(layer.data, dtype=np.float32).reshape(n_cols, n_rows).T
    return grid.T[::-1, ::-1]


def tile_from_message(message: GridMap) -> tuple:
    """(elevation (51, 51) float32, ix, iy) from one /localization/map_tile.

    The tile's identity is not in the message anywhere except its centre:
    unlike an obstacle tile, a map tile's header.frame_id is the plain map
    frame, so the index comes back through navi_localization.tiles'
    `tile_index_of`, which is the exact inverse of the `tile_center` the
    mapper wrote.

    Resolution is checked, never resampled: spec section 5 puts the costmap
    at 0.05 m precisely because "resampling smears the step edges that matter
    most", and a tile at another resolution means the mapper changed under us.
    """
    resolution = float(message.info.resolution)
    if abs(resolution - RESOLUTION) > 1e-9:
        raise ValueError(
            f"map tile resolution {resolution} is not {RESOLUTION}; this node "
            "does not resample - resampling smears the step edges that matter most")
    elevation = layer_from_message(message, ELEVATION_LAYER)
    if elevation.shape != (TILE_SAMPLES, TILE_SAMPLES):
        raise ValueError(
            f"a map tile is {TILE_SAMPLES}x{TILE_SAMPLES} samples, got {elevation.shape}")
    ix, iy = tile_index_of(float(message.info.pose.position.x),
                           float(message.info.pose.position.y))
    return elevation, ix, iy


def _layer_message(grid: np.ndarray) -> Float32MultiArray:
    """`grid` already flipped into grid_map's index order."""
    n_rows, n_cols = grid.shape
    layer = Float32MultiArray()
    layer.layout.dim = [
        MultiArrayDimension(label='column_index', size=n_cols, stride=n_rows * n_cols),
        MultiArrayDimension(label='row_index', size=n_rows, stride=n_rows),
    ]
    layer.layout.data_offset = 0
    buffer = array.array('f')
    buffer.frombytes(np.ascontiguousarray(grid, dtype=np.float32)
                     .flatten(order='F').tobytes())
    layer.data = buffer
    return layer


def build_grid_map(layers: dict, origin_ix: int, origin_iy: int, resolution: float,
                   frame_id: str, stamp) -> GridMap:
    """Several storage-convention layers as one GridMap.

    `origin_ix` / `origin_iy` are the lattice indices of column 0 and row 0;
    grid_map wants the map's *centre*, which is half a window further on.
    """
    names = list(layers)
    if not names:
        raise ValueError("a GridMap needs at least one layer")
    first = np.asarray(layers[names[0]], dtype=np.float32)
    n_y, n_x = first.shape
    message = GridMap()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    # GridMapInfo carries no header of its own in grid_map_msgs 2.0.1 -
    # checked with `ros2 interface show grid_map_msgs/msg/GridMapInfo`, which
    # is resolution, length_x, length_y, pose and nothing else.
    message.info.resolution = float(resolution)
    # grid_map's rows run in -x and its columns in -y, so its length_x is our
    # column count and its length_y our row count.
    message.info.length_x = float(n_x * resolution)
    message.info.length_y = float(n_y * resolution)
    message.info.pose.position.x = float((origin_ix + n_x / 2.0) * resolution)
    message.info.pose.position.y = float((origin_iy + n_y / 2.0) * resolution)
    message.info.pose.position.z = 0.0
    message.info.pose.orientation.w = 1.0
    message.layers = names
    message.basic_layers = [names[0]]
    message.data = [
        _layer_message(np.asarray(layers[name], dtype=np.float32)[::-1, ::-1].T)
        for name in names]
    message.outer_start_index = 0
    message.inner_start_index = 0
    return message


def build_occupancy_grid(cost: np.ndarray, origin_ix: int, origin_iy: int,
                         resolution: float, frame_id: str, stamp) -> OccupancyGrid:
    """A storage-convention int8 cost grid as an OccupancyGrid.

    No flip here: OccupancyGrid's data is row-major from the origin corner
    with x fastest and y ascending, which is exactly the storage convention.
    `info.origin` is the **corner** of cell (0, 0), not its centre.
    """
    cost = np.ascontiguousarray(cost, dtype=np.int8)
    if cost.ndim != 2:
        raise ValueError(f"a cost grid is 2-D, got shape {cost.shape}")
    n_y, n_x = cost.shape
    message = OccupancyGrid()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    message.info.map_load_time = stamp
    message.info.resolution = float(resolution)
    message.info.width = int(n_x)
    message.info.height = int(n_y)
    message.info.origin.position.x = float(origin_ix * resolution)
    message.info.origin.position.y = float(origin_iy * resolution)
    message.info.origin.position.z = 0.0
    message.info.origin.orientation.w = 1.0
    buffer = array.array('b')
    buffer.frombytes(cost.tobytes())
    message.data = buffer
    return message
```

- [ ] Run → expect `10 passed`.

- [ ] Commit: `git add rover/src/navi_autonomy/navi_autonomy/grid_map_io.py rover/src/navi_autonomy/test/test_grid_map_io.py && git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "Read and write the GridMap contract, with a round trip against the rover's own tile publisher"`.

---

### Task 5: the `tile_aggregator` node

**Files:**
- Create `rover/src/navi_autonomy/navi_autonomy/tile_aggregator.py`
- Test: create `rover/src/navi_autonomy/test/test_tile_aggregator.py`

**Interfaces:**
- Consumes: `/localization/map_tile` (`grid_map_msgs/GridMap`, reliable/volatile/depth 64), `/localization/pose` (`nav_msgs/Odometry`, depth 1).
- Produces: `/autonomy/map` (`grid_map_msgs/GridMap`, one `elevation` layer, reliable/**transient_local**/depth 1), 960 × 960 at 0.05 m; parameters `map_tile_topic`, `pose_topic`, `map_topic`, `frame_id` (`'map'`), `window_cells` (960), `publish_period_s` (1.0).

**Steps:**

- [ ] Write the failing test `rover/src/navi_autonomy/test/test_tile_aggregator.py`:
```python
"""tile_aggregator's plumbing, with the publishers replaced by recorders - no
ROS graph, the same shape as test_elevation_mapper.py.

  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization \
    python3 -m pytest rover/src/navi_autonomy/test/test_tile_aggregator.py -q'
"""
import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry

from navi_autonomy.grid_map_io import layer_from_message
from navi_autonomy.tile_aggregator import MAP_TILE_TOPIC, MAP_TOPIC, POSE_TOPIC, TileAggregator
from navi_autonomy.window import WINDOW_CELLS
from navi_localization.elevation_mapper import build_tile_message


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    node = TileAggregator()
    node._map_publisher = Recorder()
    yield node
    node.destroy_node()


def tile(value=1.0):
    out = np.full((51, 51), np.nan, dtype=np.float32)
    out[:50, :50] = value
    return out


def pose_at(x, y):
    odom = Odometry()
    odom.pose.pose.position.x = float(x)
    odom.pose.pose.position.y = float(y)
    return odom


def test_the_topics_are_the_spec_names():
    assert MAP_TILE_TOPIC == '/localization/map_tile'
    assert POSE_TOPIC == '/localization/pose'
    assert MAP_TOPIC == '/autonomy/map'


def test_the_tile_subscription_is_as_deep_as_the_mappers_publisher(node):
    """A dropped tile is indistinguishable from unseen ground, and the mapper
    bursts up to 25 tiles a tick."""
    from navi_localization.elevation_mapper import TILE_QUEUE_DEPTH
    from navi_autonomy.tile_aggregator import tile_subscription_qos
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
    assert node.tile_queue_depth == TILE_QUEUE_DEPTH == 64
    qos = tile_subscription_qos()
    assert qos.depth == 64
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.VOLATILE


def test_a_tile_lands_in_the_window_and_goes_out_on_the_next_tick(node):
    node._on_tile(build_tile_message((0, 0), tile(2.0), 'map', Time()))
    node._tick()
    assert len(node._map_publisher.messages) == 1
    message = node._map_publisher.messages[0]
    assert list(message.layers) == ['elevation']
    assert message.header.frame_id == 'map'
    assert message.info.resolution == pytest.approx(0.05)
    assert message.info.length_x == pytest.approx(WINDOW_CELLS * 0.05)
    elevation = layer_from_message(message, 'elevation')
    assert elevation.shape == (WINDOW_CELLS, WINDOW_CELLS)
    half = WINDOW_CELLS // 2
    assert elevation[half:half + 50, half:half + 50] == pytest.approx(2.0)
    assert np.isnan(elevation[0, 0])


def test_two_tiles_stitch_into_one_map(node):
    node._on_tile(build_tile_message((0, 0), tile(1.0), 'map', Time()))
    node._on_tile(build_tile_message((1, 0), tile(2.0), 'map', Time()))
    node._tick()
    elevation = layer_from_message(node._map_publisher.messages[0], 'elevation')
    half = WINDOW_CELLS // 2
    assert elevation[half, half] == pytest.approx(1.0)
    assert elevation[half, half + 50] == pytest.approx(2.0)


def test_an_all_nan_tile_erases_what_it_named(node):
    node._on_tile(build_tile_message((0, 0), tile(1.0), 'map', Time()))
    node._on_tile(build_tile_message(
        (0, 0), np.full((51, 51), np.nan, dtype=np.float32), 'map', Time()))
    node._tick()
    elevation = layer_from_message(node._map_publisher.messages[0], 'elevation')
    half = WINDOW_CELLS // 2
    assert not np.isfinite(elevation[half:half + 50, half:half + 50]).any()


def test_a_tile_at_the_wrong_resolution_is_dropped_not_resampled(node):
    message = build_tile_message((0, 0), tile(1.0), 'map', Time())
    message.info.resolution = 0.10
    node._on_tile(message)                     # must not raise out of a callback
    node._tick()
    assert node.rejected_tiles == 1
    assert node.tiles_received == 0
    assert node._map_publisher.messages == []  # nothing was ever seen
    assert not np.isfinite(node.window.elevation).any()


def test_nothing_is_published_before_the_first_tile(node):
    node._tick()
    assert node._map_publisher.messages == []


def test_a_pose_close_to_the_centre_does_not_move_the_window(node):
    node._on_tile(build_tile_message((0, 0), tile(1.0), 'map', Time()))
    before = (node.window.origin_ix, node.window.origin_iy)
    node._on_pose(pose_at(5.0, -5.0))
    node._tick()
    assert (node.window.origin_ix, node.window.origin_iy) == before


def test_a_distant_pose_recentres_the_window_and_keeps_the_ground_where_it_is(node):
    node._on_tile(build_tile_message((0, 0), tile(3.0), 'map', Time()))
    node._on_pose(pose_at(20.0, 0.0))
    node._tick()
    assert node.window.origin_ix > -WINDOW_CELLS // 2
    elevation = layer_from_message(node._map_publisher.messages[0], 'elevation')
    column = 0 - node.window.origin_ix         # lattice cell 0
    row = 0 - node.window.origin_iy
    assert elevation[row, column] == pytest.approx(3.0)
```

- [ ] Run → expect `ModuleNotFoundError: No module named 'navi_autonomy.tile_aggregator'`.

- [ ] Write `rover/src/navi_autonomy/navi_autonomy/tile_aggregator.py`:
```python
"""The 2.5 m map tiles, stitched into one 48 m window around the rover.

Spec section 5: "tile_aggregator subscribes /localization/map_tile, stitches
tiles into a rolling window around the rover (48 m, so the 60 m map cap is
never the binding constraint), and publishes a whole-map GridMap for
downstream use. This is the piece the old spec assumed already existed."

Queue depth is the thing to get right here. The mapper can emit 25 tiles in a
single tick (8 dirty + 16 blanks + 1 keepalive), a map load marks all ~576
tiles dirty at once, and the start-of-run burst was measured on the Orin at
714 KB/s settling within ~10 s. A shallow subscription drops tiles silently,
and a dropped tile is indistinguishable from unseen ground in the window it
should have filled - so the depth here is exactly the mapper's own
TILE_QUEUE_DEPTH, imported rather than repeated. Durability must be volatile
to match the publisher; a durability mismatch means no data at all.

/autonomy/map is transient_local so traversability_layer may start after this
node and still get a map immediately, rather than waiting a tick.
"""

import rclpy
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from navi_autonomy.grid_map_io import ELEVATION_LAYER, build_grid_map, tile_from_message
from navi_autonomy.window import WINDOW_CELLS, RollingWindow
from navi_localization.elevation_grid import RESOLUTION
from navi_localization.elevation_mapper import TILE_QUEUE_DEPTH

MAP_TILE_TOPIC = '/localization/map_tile'
POSE_TOPIC = '/localization/pose'
MAP_TOPIC = '/autonomy/map'


def tile_subscription_qos(depth: int = TILE_QUEUE_DEPTH) -> QoSProfile:
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=depth)


def latched_qos() -> QoSProfile:
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST, depth=1)


class TileAggregator(Node):

    def __init__(self):
        super().__init__('tile_aggregator')
        self.declare_parameter('map_tile_topic', MAP_TILE_TOPIC)
        self.declare_parameter('pose_topic', POSE_TOPIC)
        self.declare_parameter('map_topic', MAP_TOPIC)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('window_cells', WINDOW_CELLS)
        self.declare_parameter('publish_period_s', 1.0)
        self.declare_parameter('tile_queue_depth', TILE_QUEUE_DEPTH)

        self._frame_id = str(self.get_parameter('frame_id').value)
        self.tile_queue_depth = int(self.get_parameter('tile_queue_depth').value)
        self.window = RollingWindow(cells=int(self.get_parameter('window_cells').value),
                                    resolution=RESOLUTION)
        self.tiles_received = 0
        self.rejected_tiles = 0
        self._pose = None
        self._rejected_logged = False

        self._map_publisher = self.create_publisher(
            GridMap, str(self.get_parameter('map_topic').value), latched_qos())
        self.create_subscription(
            GridMap, str(self.get_parameter('map_tile_topic').value), self._on_tile,
            tile_subscription_qos(self.tile_queue_depth))
        self.create_subscription(
            Odometry, str(self.get_parameter('pose_topic').value), self._on_pose, 1)
        self.create_timer(float(self.get_parameter('publish_period_s').value), self._tick)

    # -- inputs -----------------------------------------------------------

    def _on_tile(self, message: GridMap) -> None:
        try:
            elevation, ix, iy = tile_from_message(message)
        except ValueError as error:
            # A callback that raises takes the executor down with it, and a
            # mapper that changed its contract must be a log line, not a
            # dead node.
            self.rejected_tiles += 1
            if not self._rejected_logged:
                self._rejected_logged = True
                self.get_logger().warn(f"dropping map tiles: {error}")
            return
        self.window.paste_tile(ix, iy, elevation)
        self.tiles_received += 1

    def _on_pose(self, message: Odometry) -> None:
        self._pose = (float(message.pose.pose.position.x),
                      float(message.pose.pose.position.y))

    # -- output -----------------------------------------------------------

    def _tick(self) -> None:
        if self.tiles_received == 0:
            return                      # nothing to say yet; do not publish an empty map
        if self._pose is not None:
            self.window.recentre(*self._pose)
        self._map_publisher.publish(build_grid_map(
            {ELEVATION_LAYER: self.window.elevation},
            self.window.origin_ix, self.window.origin_iy, self.window.resolution,
            self._frame_id, self.get_clock().now().to_msg()))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TileAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] Run → expect `9 passed`.

- [ ] Commit: `git add rover/src/navi_autonomy/navi_autonomy/tile_aggregator.py rover/src/navi_autonomy/test/test_tile_aggregator.py && git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "tile_aggregator: the 2.5 m tiles stitched into one 48 m rolling window on /autonomy/map"`.

---

### Task 6: the `traversability_layer` node

**Files:**
- Create `rover/src/navi_autonomy/navi_autonomy/traversability_layer.py`
- Test: create `rover/src/navi_autonomy/test/test_traversability_layer.py`

**Interfaces:**
- Consumes: `/autonomy/map` (`grid_map_msgs/GridMap`, one `elevation` layer, reliable/transient_local/depth 1).
- Produces: `/autonomy/traversability` (`grid_map_msgs/GridMap`, layers `slope`, `step`, `roughness`, `valid`, `basic_layers = ['valid']`, reliable/volatile/depth 1, published only while `count_subscribers > 0`); `/autonomy/costmap_seed` (`nav_msgs/OccupancyGrid`, reliable/**transient_local**/depth 1, always published). Parameters `map_topic`, `traversability_topic`, `costmap_seed_topic`, `frame_id`.

**Steps:**

- [ ] Write the failing test `rover/src/navi_autonomy/test/test_traversability_layer.py`:
```python
"""traversability_layer's plumbing and the end of the chain: a pit in, lethal
cells out. Publishers replaced by recorders; no ROS graph.

  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization \
    python3 -m pytest rover/src/navi_autonomy/test/test_traversability_layer.py -q'
"""
import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time

from navi_autonomy.grid_map_io import build_grid_map, layer_from_message
from navi_autonomy.traversability import LETHAL, UNKNOWN
from navi_autonomy.traversability_layer import (
    COSTMAP_SEED_TOPIC, MAP_TOPIC, TRAVERSABILITY_TOPIC, TraversabilityLayer)


class Recorder:
    def __init__(self, subscribers=1):
        self.messages = []
        self.subscribers = subscribers

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    node = TraversabilityLayer()
    node._traversability_publisher = Recorder()
    node._seed_publisher = Recorder()
    node._traversability_subscribers = lambda: 1
    yield node
    node.destroy_node()


def pit_map(depth=0.2, size=6, extent=24, origin_ix=-12, origin_iy=-12):
    grid = np.zeros((extent, extent), dtype=np.float32)
    lo = (extent - size) // 2
    grid[lo:lo + size, lo:lo + size] = -depth
    return build_grid_map({'elevation': grid}, origin_ix, origin_iy, 0.05,
                          'map', Time()), lo


def test_the_topics_are_the_spec_names():
    assert MAP_TOPIC == '/autonomy/map'
    assert TRAVERSABILITY_TOPIC == '/autonomy/traversability'
    assert COSTMAP_SEED_TOPIC == '/autonomy/costmap_seed'


def test_a_pit_publishes_lethal_cells_on_its_rim(node):
    message, lo = pit_map()
    node._on_map(message)
    assert len(node._seed_publisher.messages) == 1
    seed = node._seed_publisher.messages[0]
    cost = np.asarray(seed.data, dtype=np.int8).reshape(seed.info.height, seed.info.width)
    assert cost[lo - 1, lo - 1] == LETHAL
    assert cost[lo - 1, lo + 2] == LETHAL
    assert cost[2, 2] == 0
    assert (cost == LETHAL).sum() == 48


def test_the_seed_carries_the_maps_geometry(node):
    message, _ = pit_map(origin_ix=-12, origin_iy=40)
    node._on_map(message)
    seed = node._seed_publisher.messages[0]
    assert seed.header.frame_id == 'map'
    assert seed.info.resolution == pytest.approx(0.05)
    assert (seed.info.width, seed.info.height) == (24, 24)
    assert seed.info.origin.position.x == pytest.approx(-0.6)
    assert seed.info.origin.position.y == pytest.approx(2.0)


def test_unseen_ground_is_unknown_in_the_seed(node):
    grid = np.zeros((10, 10), dtype=np.float32)
    grid[5, 5] = np.nan
    node._on_map(build_grid_map({'elevation': grid}, 0, 0, 0.05, 'map', Time()))
    seed = node._seed_publisher.messages[0]
    cost = np.asarray(seed.data, dtype=np.int8).reshape(seed.info.height, seed.info.width)
    assert cost[5, 5] == UNKNOWN
    assert cost[0, 0] == UNKNOWN


def test_the_traversability_grid_map_carries_all_four_layers(node):
    message, _ = pit_map()
    node._on_map(message)
    published = node._traversability_publisher.messages[0]
    assert list(published.layers) == ['slope', 'step', 'roughness', 'valid']
    assert list(published.basic_layers) == ['valid']
    assert published.info.resolution == pytest.approx(0.05)
    step = layer_from_message(published, 'step')
    assert np.nanmax(step) == pytest.approx(0.2)


def test_the_expensive_grid_map_is_not_built_when_nobody_is_listening(node):
    node._traversability_subscribers = lambda: 0
    message, _ = pit_map()
    node._on_map(message)
    assert node._traversability_publisher.messages == []
    assert len(node._seed_publisher.messages) == 1     # the seed always goes out


def test_a_map_at_the_wrong_resolution_is_refused(node):
    message, _ = pit_map()
    message.info.resolution = 0.10
    node._on_map(message)
    assert node._seed_publisher.messages == []
    assert node.rejected_maps == 1


def test_a_map_without_an_elevation_layer_is_refused(node):
    message, _ = pit_map()
    message.layers = ['colour']
    node._on_map(message)
    assert node._seed_publisher.messages == []
    assert node.rejected_maps == 1
```

- [ ] Run → expect `ModuleNotFoundError: No module named 'navi_autonomy.traversability_layer'`.

- [ ] Write `rover/src/navi_autonomy/navi_autonomy/traversability_layer.py`:
```python
"""slope, step, roughness and valid from the aggregated map, and the costmap
seed Nav2 plans on.

Spec section 5: "traversability_layer reads that map and derives slope, step,
roughness, valid ... Publishes /autonomy/traversability (GridMap, for the
view - it can also drive the pit colouring in the sim) and
/autonomy/costmap_seed (OccupancyGrid, latched)."

Event-driven, not on a timer: the map arrives at about 1 Hz and there is
nothing to recompute in between. The derive is ~150 ms at 960 x 960 on the
laptop (see traversability.derive).

The four-layer GridMap is 14.7 MB per message and nothing on the rover
subscribes to it - it is for the view and the sim - so it is built only when
someone is listening, the same count_subscribers guard the ZED wrapper uses
for its fused cloud. The 0.92 MB OccupancyGrid seed is what Nav2 reads and
always goes out, latched, so a Nav2 that starts later gets a map instantly
instead of planning on nothing.
"""

import rclpy
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from navi_autonomy.grid_map_io import (
    ELEVATION_LAYER, build_grid_map, build_occupancy_grid, layer_from_message)
from navi_autonomy.tile_aggregator import MAP_TOPIC, latched_qos
from navi_autonomy.traversability import seed_from_elevation
from navi_localization.elevation_grid import RESOLUTION

TRAVERSABILITY_TOPIC = '/autonomy/traversability'
COSTMAP_SEED_TOPIC = '/autonomy/costmap_seed'
LAYER_ORDER = ('slope', 'step', 'roughness', 'valid')


def view_qos() -> QoSProfile:
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=1)


class TraversabilityLayer(Node):

    def __init__(self):
        super().__init__('traversability_layer')
        self.declare_parameter('map_topic', MAP_TOPIC)
        self.declare_parameter('traversability_topic', TRAVERSABILITY_TOPIC)
        self.declare_parameter('costmap_seed_topic', COSTMAP_SEED_TOPIC)
        self.declare_parameter('frame_id', 'map')

        self._frame_id = str(self.get_parameter('frame_id').value)
        self.maps_processed = 0
        self.rejected_maps = 0
        self._rejected_logged = False

        self._traversability_topic = str(self.get_parameter('traversability_topic').value)
        self._traversability_publisher = self.create_publisher(
            GridMap, self._traversability_topic, view_qos())
        self._seed_publisher = self.create_publisher(
            OccupancyGrid, str(self.get_parameter('costmap_seed_topic').value),
            latched_qos())
        self.create_subscription(
            GridMap, str(self.get_parameter('map_topic').value), self._on_map,
            latched_qos())

    def _traversability_subscribers(self) -> int:
        return self.count_subscribers(self._traversability_topic)

    def _on_map(self, message: GridMap) -> None:
        resolution = float(message.info.resolution)
        try:
            if abs(resolution - RESOLUTION) > 1e-9:
                raise ValueError(
                    f"map resolution {resolution} is not {RESOLUTION}; this node "
                    "does not resample")
            elevation = layer_from_message(message, ELEVATION_LAYER)
        except ValueError as error:
            self.rejected_maps += 1
            if not self._rejected_logged:
                self._rejected_logged = True
                self.get_logger().warn(f"dropping /autonomy/map: {error}")
            return

        n_y, n_x = elevation.shape
        # grid_map gives the map's centre; the corner cell's lattice index is
        # half a map back, and it is what both output messages are anchored on.
        origin_ix = int(round(float(message.info.pose.position.x) / resolution - n_x / 2.0))
        origin_iy = int(round(float(message.info.pose.position.y) / resolution - n_y / 2.0))

        layers, cost = seed_from_elevation(elevation, resolution)
        stamp = message.header.stamp
        self._seed_publisher.publish(build_occupancy_grid(
            cost, origin_ix, origin_iy, resolution, self._frame_id, stamp))
        if self._traversability_subscribers() > 0:
            grid_map = build_grid_map(
                {name: layers[name] for name in LAYER_ORDER},
                origin_ix, origin_iy, resolution, self._frame_id, stamp)
            grid_map.basic_layers = ['valid']
            self._traversability_publisher.publish(grid_map)
        self.maps_processed += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TraversabilityLayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] Run → expect `8 passed`. Then run the whole pure suite: `bash -c 'source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization:$PYTHONPATH python3 -m pytest rover/src/navi_autonomy/test -q -p no:cacheprovider'` → expect `62 passed`.

- [ ] Commit: `git add rover/src/navi_autonomy/navi_autonomy/traversability_layer.py rover/src/navi_autonomy/test/test_traversability_layer.py && git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "traversability_layer: the four derived layers and the latched costmap seed, with holes lethal"`.

---

### Task 7: launch file and the ROS-graph acceptance test

**Files:**
- Create `rover/src/navi_autonomy/launch/autonomy_perception.launch.py`
- Test: create `rover/src/navi_autonomy/test/test_autonomy_graph.py`

**Interfaces:** Consumes `/localization/map_tile`, `/localization/pose`. Produces `/autonomy/map`, `/autonomy/traversability`, `/autonomy/costmap_seed`. No node in this launch file publishes a twist of any kind.

**Steps:**

- [ ] Write the failing test `rover/src/navi_autonomy/test/test_autonomy_graph.py`:
```python
"""Both nodes on a real ROS graph, on a throwaway domain: tiles in on
/localization/map_tile, a lethal pit rim out on /autonomy/costmap_seed.

  bash -c 'source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=91 \
    PYTHONPATH=$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization \
    python3 -m pytest rover/src/navi_autonomy/test/test_autonomy_graph.py -q'

Domain 91 is a throwaway (spec section 9 and this repo's standing rule);
never domain 0, where the rover and the simulation live. Nothing here
publishes /manual_twist.
"""
import os
import time

import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from navi_autonomy.tile_aggregator import (
    MAP_TILE_TOPIC, TileAggregator, latched_qos, tile_subscription_qos)
from navi_autonomy.traversability import LETHAL
from navi_autonomy.traversability_layer import COSTMAP_SEED_TOPIC, TraversabilityLayer
from navi_localization.elevation_mapper import build_tile_message

assert os.environ.get('ROS_DOMAIN_ID') == '91', \
    "run this file with ROS_DOMAIN_ID=91; never on domain 0"


class Sink(Node):
    def __init__(self):
        super().__init__('graph_test_sink')
        self.seeds = []
        self.create_subscription(OccupancyGrid, COSTMAP_SEED_TOPIC,
                                 self.seeds.append, latched_qos())


class Source(Node):
    def __init__(self):
        super().__init__('graph_test_source')
        self.publisher = self.create_publisher(
            GridMap, MAP_TILE_TOPIC, tile_subscription_qos())


def pit_tile(depth=0.2):
    """Tile (0, 0): flat at z = 0 with a 6 x 6 pit at cells [20, 26)."""
    tile = np.full((51, 51), np.nan, dtype=np.float32)
    tile[:50, :50] = 0.0
    tile[20:26, 20:26] = -depth
    return tile


def spin(executor, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)


@pytest.fixture
def graph():
    rclpy.init()
    aggregator = TileAggregator()          # 1 Hz publish timer, the default
    layer = TraversabilityLayer()
    source, sink = Source(), Sink()
    executor = SingleThreadedExecutor()
    for node in (aggregator, layer, source, sink):
        executor.add_node(node)
    spin(executor, 1.0)                      # discovery
    yield executor, source, sink, aggregator, layer
    for node in (aggregator, layer, source, sink):
        executor.remove_node(node)
        node.destroy_node()
    rclpy.shutdown()


def test_a_pit_published_as_a_tile_comes_back_as_lethal_cells(graph):
    executor, source, sink, aggregator, layer = graph
    source.publisher.publish(build_tile_message((0, 0), pit_tile(), 'map', Time()))
    spin(executor, 3.0)

    assert aggregator.tiles_received >= 1
    assert layer.maps_processed >= 1
    assert sink.seeds, "no /autonomy/costmap_seed arrived"
    seed = sink.seeds[-1]
    cost = np.asarray(seed.data, dtype=np.int8).reshape(seed.info.height, seed.info.width)
    assert cost.shape == (960, 960)
    assert seed.info.resolution == pytest.approx(0.05)

    # The pit's rim in map coordinates: lattice cell 19 is the flat ring just
    # outside it, and the window's column 0 is lattice cell origin_ix.
    row = 19 - aggregator.window.origin_iy
    column = 19 - aggregator.window.origin_ix
    assert cost[row, column] == LETHAL
    assert (cost == LETHAL).sum() == 48
    assert cost[row - 5, column - 5] == 0            # flat ground a few cells away
    assert (cost == -1).sum() > 0                    # everything never seen


def test_the_seed_is_latched_for_a_late_subscriber(graph):
    executor, source, sink, aggregator, layer = graph
    source.publisher.publish(build_tile_message((0, 0), pit_tile(), 'map', Time()))
    spin(executor, 3.0)
    late = Sink()
    executor.add_node(late)
    spin(executor, 2.0)
    assert late.seeds, "transient_local did not deliver the seed to a late joiner"
    executor.remove_node(late)
    late.destroy_node()
```

- [ ] Run it: `bash -c 'source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=91 PYTHONPATH=$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization:$PYTHONPATH python3 -m pytest rover/src/navi_autonomy/test/test_autonomy_graph.py -q -p no:cacheprovider'`. This test asserts wiring, not new code — Tasks 5 and 6 already wrote both nodes — so it may well pass first time, and that is the point of running it before the launch file exists. **If it fails, the failure is real** and is fixed here, in the nodes: the two candidates are a QoS mismatch on `/autonomy/map` (both ends must be reliable + transient_local, which is why `latched_qos()` is shared rather than repeated) and too short a spin for discovery on a cold domain (raise the 3.0 s, never lower the assertions). Also run `bash -c 'source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=91 ros2 topic list'` after it and confirm `/manual_twist` is **not** there.

- [ ] Write `rover/src/navi_autonomy/launch/autonomy_perception.launch.py`:
```python
"""tile_aggregator and traversability_layer, the Orin's autonomy perception.

Not included by rover/start_navi.sh: SP9's Nav2 bringup includes this file,
and the two of them start together or not at all - a costmap seed with no
planner is dead weight, and a planner with no seed plans through holes.

Nothing in this file publishes a twist. Both nodes are read-only with respect
to the chassis.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    frame_id = LaunchConfiguration('frame_id')
    window_cells = LaunchConfiguration('window_cells')
    publish_period_s = LaunchConfiguration('publish_period_s')
    return LaunchDescription([
        DeclareLaunchArgument('frame_id', default_value='map'),
        # 48 m at 0.05 m. Spec section 5's documented fallback, if the Orin
        # cannot hold 1 Hz, is 480 (a 24 m window) - measured in SP9/SP10 on
        # the Orin (spec section 5, section 11 risk 6), not guessed here;
        # SP12 re-measures it in the yard.
        DeclareLaunchArgument('window_cells', default_value='960'),
        DeclareLaunchArgument('publish_period_s', default_value='1.0'),
        Node(package='navi_autonomy', executable='tile_aggregator',
             name='tile_aggregator', output='screen',
             parameters=[{'frame_id': frame_id,
                          'window_cells': window_cells,
                          'publish_period_s': publish_period_s}]),
        Node(package='navi_autonomy', executable='traversability_layer',
             name='traversability_layer', output='screen',
             parameters=[{'frame_id': frame_id}]),
    ])
```

- [ ] Build and check the entry points resolve: `bash -c 'source /opt/ros/humble/setup.bash && cd rover && colcon build --packages-up-to navi_autonomy'` → expect `Summary: 1 package finished`. Then `bash -c 'source /opt/ros/humble/setup.bash && source rover/install/local_setup.bash && ros2 pkg executables navi_autonomy'` → expect exactly `navi_autonomy tile_aggregator` and `navi_autonomy traversability_layer`.

- [ ] Start the launch file and confirm both nodes stay up and the topics exist:
  `bash -c 'source /opt/ros/humble/setup.bash && source rover/install/setup.bash &&
    ROS_DOMAIN_ID=91 timeout 12 ros2 launch navi_autonomy autonomy_perception.launch.py'`
  -> expect two "process started" lines and no traceback. In a second shell,
  `bash -c 'source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=91 ros2 topic list'`
  -> `/autonomy/map`, `/autonomy/traversability`, `/autonomy/costmap_seed` present,
  `/manual_twist` absent.

- [ ] Re-run the graph test → expect `2 passed`, and re-run the pure suite (with `--ignore=rover/src/navi_autonomy/test/test_autonomy_graph.py`) → expect `62 passed`.

- [ ] Commit: `git add rover/src/navi_autonomy/launch rover/src/navi_autonomy/test/test_autonomy_graph.py && git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "Launch both autonomy perception nodes, and prove on a throwaway domain that a published pit comes back as lethal cells"`.

---

### Task 8: measure, and write down what deploying this needs

**Files:**
- Modify `rover/src/navi_autonomy/navi_autonomy/tile_aggregator.py` (measured numbers into the module docstring)
- Modify `rover/src/navi_autonomy/launch/autonomy_perception.launch.py` (measured numbers into the docstring, as `localization.launch.py` does)
- Modify `docs/superpowers/specs/autonomy-plan.md` (§5: record the topic name, the cost curve and the SP7 status; §13: the deb-carry list)

**Interfaces:** none new.

**Steps:**

- [ ] Measure the full-size derive on this laptop and write the number down. Run:
```
bash -c 'source /opt/ros/humble/setup.bash && PYTHONPATH=$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization:$PYTHONPATH python3 -c "
import time, numpy as np
from navi_autonomy.traversability import seed_from_elevation
from navi_autonomy.grid_map_io import build_grid_map, build_occupancy_grid
from builtin_interfaces.msg import Time
rng = np.random.default_rng(0)
e = (rng.normal(0, 0.02, (960, 960)) + np.linspace(0, 3, 960)[:, None]).astype(np.float32)
e[rng.random((960, 960)) < 0.2] = np.nan
for _ in range(3):
    t = time.perf_counter(); layers, cost = seed_from_elevation(e); derive_ms = (time.perf_counter() - t) * 1000
    t = time.perf_counter(); build_grid_map(layers, 0, 0, 0.05, chr(109)+chr(97)+chr(112), Time()); gm_ms = (time.perf_counter() - t) * 1000
    t = time.perf_counter(); build_occupancy_grid(cost, 0, 0, 0.05, chr(109)+chr(97)+chr(112), Time()); og_ms = (time.perf_counter() - t) * 1000
print(round(derive_ms, 1), round(gm_ms, 1), round(og_ms, 1))
"'
```
  Expected on this laptop: roughly `150 12 1`. Put the three real numbers, the date and the machine into `autonomy_perception.launch.py`'s docstring in the style `localization.launch.py` uses, and note that the Orin figure is SP9/SP10's to measure (spec §5, §11 risk 6), which SP12 re-measures in the yard, with the documented 24 m fallback if 1 Hz is not held.

- [ ] Add to `docs/superpowers/specs/autonomy-plan.md` §5, immediately after the `traversability_layer` block, without editing any sentence already there:
```markdown
**SP7 as built** (2026-08-30): `rover/src/navi_autonomy`, two nodes.
`tile_aggregator` publishes the stitched 960 x 960 window on `/autonomy/map`
(GridMap, one `elevation` layer, transient_local); `traversability_layer`
reads it and publishes `/autonomy/traversability` (GridMap: `slope`, `step`,
`roughness`, `valid`; only while something subscribes, since it is 14.7 MB a
message) and `/autonomy/costmap_seed` (OccupancyGrid, transient_local,
0.92 MB). The seed's exact curve:

    s = clip(slope / radians(25), 0, 1);  t = clip(step / 0.14, 0, 1)
    r = clip(roughness / 0.05, 0, 1)      # roughness = |z - mean of the seen
                                          # 4-neighbours|: zero on any plane
    cost = round(99 * max(s, t, r))       # 0..99, 100 reserved for lethal
    cost = -1  where valid == 0           # never-seen ground is unknown, not free
    cost = 100 where step >= 0.14 or slope >= radians(25)   # applied last, so a
                                          # measured lethal step beats an
                                          # incomplete neighbourhood

The window recentres only when the rover is more than 8 m from its centre;
cells that leave it are dropped and return via the mapper's keepalive. Tiles
are subscribed at depth 64, the mapper's own `TILE_QUEUE_DEPTH`, because the
mapper bursts up to 25 tiles a tick and a dropped tile is indistinguishable
from unseen ground.
```

- [ ] Add to `docs/superpowers/specs/autonomy-plan.md` §13, after the existing sentence:
```markdown
SP7 itself needs no new package on the Orin: `navi_autonomy` depends only on
`rclpy`, `nav_msgs`, `std_msgs` and `grid_map_msgs`, and `grid_map_msgs` is
already there because `navi_localization` runs there. What must be carried
over is `ros-humble-grid-map-costmap-2d` (2.0.1-1jammy) **and its
dependencies** `ros-humble-grid-map-core`, `ros-humble-grid-map-cv`,
`ros-humble-grid-map-ros`, for SP9's Nav2 costmap plugin - all five are
installed on the laptop and can be fetched with
`apt-get download` / `apt-get install --reinstall -d` and copied with the
`msgpack` wheels.
```

- [ ] If §5 or §13 has changed under you, re-read it and append below whatever is now there; never rewrite another agent's text.

- [ ] Run the full pure suite one last time and the graph test: `62 passed` and `2 passed`. Then confirm nothing outside this sub-project moved: `git status --short` → only `rover/src/navi_autonomy/` and `docs/superpowers/specs/autonomy-plan.md`, plus the untracked `bemacontroller/` and `graphify-out/` that were already there.

- [ ] Commit: `git add rover/src/navi_autonomy docs/superpowers/specs/autonomy-plan.md && git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit -m "Record SP7's measured cost, its topic and cost-curve contract in the spec, and what the Orin needs carried over"`.

---

## Self-review

- **Spec coverage.** §5 first block: `tile_aggregator`, `/localization/map_tile`, 48 m rolling window, whole-map GridMap — Tasks 1, 4, 5. §5 second block: `slope`/`step`/`roughness`/`valid`, step lethal above 0.14 m as a **max neighbour difference catching negative steps**, slope lethal above 25°, roughness scaled below that, `/autonomy/traversability` GridMap and `/autonomy/costmap_seed` latched OccupancyGrid, resolution 0.05 m with no resampling — Tasks 2, 3, 6. §8 SP7 row ("holes become lethal") — the acceptance assertions in Tasks 2, 3, 6 and 7. §9 rung 1 (pure functions on synthetic grids, laptop) — Tasks 1–4 import no `rclpy` in the module under test. §13 (Orin debs) — Task 8. §12 defers nothing this plan builds.
- **Placeholder scan.** No "TBD", no "handle edge cases", no "similar to Task N", no "…". Every test body and every module body above is complete text; the only deliberately unwritten content is the three measured numbers in Task 8, which the step's own command produces.
- **Type consistency.** Elevation and all four derived layers are `float32` throughout; the cost grid is `int8` from `costmap_seed` onward and is written to `OccupancyGrid.data` through `array.array('b')`. `origin_ix`/`origin_iy` are `int` lattice indices everywhere — `RollingWindow` holds them, `build_grid_map`/`build_occupancy_grid` take them, and `traversability_layer` recovers them from the GridMap centre by the same arithmetic `build_grid_map` used to write it. `tile_index_of` returns `(int, int)` and is the mapper's own function, not a copy. `valid` is `float32` 0.0/1.0 because a GridMap layer must be float; it is compared with `< 0.5`, never with `==`.
- **Cross-task consistency.** `latched_qos()` is defined once in `tile_aggregator` and imported by `traversability_layer` and the graph test, so the publisher and subscriber of `/autonomy/map` cannot drift apart. `tile_subscription_qos()` likewise, so the graph test's source matches the aggregator's sink. The test counts (13 + 22 + 10 + 9 + 8 = 62 pure, 2 graph) are stated in the run steps so a mismatch is a visible failure rather than a silent one.
