"""A growing 2.5-D elevation grid built from the ZED's fused point cloud.

Pure numpy on purpose: no rclpy, no message types, no camera. The frame
arithmetic and the growth rules are the part that can be wrong in a way a
picture would not show, so they are the part that gets tested on a laptop in
milliseconds rather than on a rover in an afternoon.

Storage convention, which is *not* grid_map's: row 0 is the smallest y and
column 0 the smallest x, both ascending, because that is what everything
except grid_map means by an array of ground. The conversion into grid_map's
own convention happens once, in elevation_mapper.build_grid_map_message.

Per-cell height is the **20th percentile** of that cell's z values in the
update, not the mean: a wall, a person or a flying pixel that lands in a
cell would otherwise pull the mean up into a spike, and the terrain drawn
from this grid has to be the ground the rover can drive on, not an average
of the ground and whatever else was overhead. Each cell also keeps `top`,
the maximum z seen in the update, so a later change can draw obstacles from
it without another pass over the cloud; `top` is not published anywhere
yet.

Each cell also keeps `stamp`, the ROS time its height/top/count were last
*written* by fusion - the prerequisite a downstream decay (obstacles that
were never re-confirmed expire, so a phantom self-heals) needs to know how
old an observation is. What "written" means here is worth being precise
about, because the ZED SDK's fused cloud is ambiguous at first glance:
`update()`'s own comment below explains that the cloud is the *whole* map
fused so far, re-sent every cycle - but the wrapper zero-fills chunks that
did not change and `finite_points` throws those zeros away (see its
docstring), so a message's *surviving* points are exactly the chunks the
SDK actually re-fused this cycle. `unique_flat` in `update()` is therefore
already the true "touched this cycle" set, at the effective, filtered
level - not a meaningless whole-map re-stamp - and `stamp` records exactly
that set's write time. It is still one step removed from "last time the
camera's line of sight crossed this cell": it is "last time the SDK's own
chunk-update bookkeeping decided this region needed re-fusing", which
tracks recent coverage closely but at the SDK's chunk granularity, not the
camera's shutter. A downstream consumer computing age from this should
read it as "time since fusion last wrote this cell", not "time since it
was last in view".
"""

from dataclasses import dataclass

import numpy as np

# The spec's numbers. RESOLUTION also has to match the ZED wrapper's
# mapping.resolution in config/zed_front.yaml - a grid finer than the cloud
# would produce a comb of empty cells - and test_localization_launch.py
# checks that it does.
RESOLUTION = 0.05
MAX_EXTENT_M = 60.0
MAX_CELLS = int(round(MAX_EXTENT_M / RESOLUTION))   # 1200
TILE_CELLS = 50            # cells per 2.5 m tile; tiles.py imports this

# The percentile a cell's drawn height is taken at, and how it is turned
# into a rank within a sorted cell (see update()'s docstring for why this
# has to be `floor`, not `round`).
HEIGHT_PERCENTILE = 0.2


def finite_points(points) -> np.ndarray:
    """The points of a fused cloud that are worth binning, as (N, 3) float64.

    Two filters, both earned:

    * Non-finite: the wrapper publishes the fused cloud with
      is_dense=false, so NaN is legal padding in it.
    * Exactly (0, 0, 0): ZedCamera::callback_pubFusedPc
      (zed_camera_component.cpp:9490 on the Orin's checkout) allocates a
      fresh, zero-filled buffer for `width` points and then memcpy's only
      the chunks whose has_been_updated flag is set. Every chunk that did
      not change this cycle therefore arrives as a run of exact zeros.
      Without this filter they pile thousands of points onto the single
      cell at the map origin and put a spike under the rover's start point.
      The cost is that a genuine measurement at exactly the origin is lost,
      which is one cell out of up to 360,000.
    """
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    finite = np.isfinite(points).all(axis=1)
    nonzero = np.any(points != 0.0, axis=1)
    return points[finite & nonzero]


@dataclass(frozen=True, eq=False)
class GridState:
    """Everything a grid is, for saving and for replacing another grid."""

    elevation: np.ndarray      # (rows, cols) float32, NaN where unseen; 20th percentile height
    top: np.ndarray            # (rows, cols) float32, NaN where unseen; max z of the cell
    count: np.ndarray          # (rows, cols) int32, points behind each cell's last update
    origin_ix: int
    origin_iy: int
    resolution: float
    # (rows, cols) float64, NaN where never observed; ROS time (seconds)
    # each cell was last written by fusion. float64, not float32 like the
    # others: this holds an absolute epoch-scale second (~1.7e9), and
    # float32's 24-bit mantissa only carries ~256 s of resolution at that
    # magnitude - the exact trap the age_s wire layer downstream is built
    # to avoid (see elevation_mapper.build_tile_message). None is the
    # backwards-compatible default: a GridState built by map_store.py from
    # an .npz saved before this field existed has no stamp at all, and
    # replace() below turns that into "every seen cell observed now"
    # rather than raising or decaying the resumed map away instantly.
    stamp: np.ndarray | None = None


# eq=False: a generated __eq__ would compare numpy arrays with ==, which
# returns an array and then raises when the dataclass takes its truth value.
# Comparison goes through equals(), which knows about NaN.
@dataclass(frozen=True, eq=False)
class GridSnapshot:
    """One published state of the map. Comparable, so republishing can stop."""

    elevation: np.ndarray      # (rows, cols) float32; row 0 = min y, col 0 = min x
    top: np.ndarray            # (rows, cols) float32, same layout; max z, never clamped
    center_x: float
    center_y: float
    resolution: float
    origin_ix: int             # lattice index of column 0
    origin_iy: int             # lattice index of row 0
    # (rows, cols) float64, NaN where never observed; see GridState.stamp
    # for why this is float64 and not float32. Optional, defaulting to
    # None, purely so code outside this module that still builds a
    # GridSnapshot by hand (navi_localization/test/test_tiles.py, which
    # this task does not own) keeps constructing one without it.
    stamp: np.ndarray | None = None

    def equals(self, other) -> bool:
        if other is None:
            return False
        # top is not published, so it does not gate republishing either;
        # stamp is excluded for the same reason, plus a sharper one - a
        # cell re-fused to the *same* height still gets a fresh stamp, so
        # including it here would mean every single cloud counts as a
        # change and the "unchanged map is not republished" optimisation
        # this compares for would never fire again.
        return (self.elevation.shape == other.elevation.shape
                and self.center_x == other.center_x
                and self.center_y == other.center_y
                and self.resolution == other.resolution
                # equal_nan, or every empty cell would count as a change and
                # the map would go out at 0.5 Hz forever.
                and np.array_equal(self.elevation, other.elevation, equal_nan=True))


class ElevationGrid:
    """20th-percentile height per cell over a window that grows with the
    map, up to 60 x 60 m. `top`, the cell's max z, rides alongside for a
    later obstacle layer."""

    def __init__(self, resolution: float = RESOLUTION, max_cells: int = MAX_CELLS):
        self.resolution = float(resolution)
        self.max_cells = int(max_cells)
        self.points_outside_cap = 0
        self._touched_tiles = set()
        # Lattice index of column 0 / row 0. None until the first point.
        self._origin_ix = None
        self._origin_iy = None
        # Raw storage: meaningless where _count == 0, masked to NaN only in
        # snapshot()/state(). Not sums any more - update() writes a cell's
        # height and top directly, since both are order statistics of this
        # message's points rather than something an old value can be
        # incorporated into.
        self._height = np.zeros((0, 0), dtype=np.float64)
        self._top = np.zeros((0, 0), dtype=np.float64)
        self._count = np.zeros((0, 0), dtype=np.int64)
        # Unlike height/top/count, NaN is its own sentinel here - a real
        # stamp is never NaN - so this needs no separate count-based
        # masking at read time the way height/top do; it is simply correct
        # everywhere count is 0 by construction (see _fit_window and
        # update() below, which only ever write the two together).
        self._stamp = np.full((0, 0), np.nan, dtype=np.float64)

    def update(self, points, already_filtered: bool = False, observed_at: float = None) -> None:
        """`already_filtered=True`: `points` is already the output of
        `finite_points` (the obstacle voxeliser needs the same filtered
        array, and `finite_points` + index flooring cost ~40 ms per 200k
        points - see elevation_mapper._on_cloud, which calls it once and
        hands the result to both this and `ObstacleMap.update`).

        `observed_at`: the ROS time (float seconds) fusion is happening at,
        stamped onto exactly the cells this call writes. `None` (the
        default, and every call in this module's own tests) means the
        caller is not telling us when - the update still happens, but the
        cells it touches keep whatever stamp they already had rather than
        this being treated as "observed now": a grid with no notion of
        time of its own must not invent one silently."""
        kept = points if already_filtered else finite_points(points)
        if kept.size == 0:
            return
        ix = np.floor(kept[:, 0] / self.resolution).astype(np.int64)
        iy = np.floor(kept[:, 1] / self.resolution).astype(np.int64)
        self._fit_window(ix, iy)

        rows, cols = self._height.shape
        inside = ((ix >= self._origin_ix) & (ix < self._origin_ix + cols)
                  & (iy >= self._origin_iy) & (iy < self._origin_iy + rows))
        self.points_outside_cap += int((~inside).sum())
        if not inside.any():
            return

        flat = ((iy[inside] - self._origin_iy) * cols
                + (ix[inside] - self._origin_ix))
        z = kept[inside, 2]

        # Vectorised order statistics per cell, no Python loop over cells:
        # lexsort by (cell, z) puts every cell's points together and, within
        # a cell, in ascending z; np.unique's return_index then finds where
        # each cell's run starts (return_counts, how long it is), so the
        # 20th-percentile rank `start + floor(0.2 * (count - 1))` and the
        # max's rank `start + count - 1` are both plain index arithmetic.
        # floor (not round) so a count of 1 lands on index 0, its only
        # point, matching "a single-point cell reports that point".
        order = np.lexsort((z, flat))
        flat_sorted = flat[order]
        z_sorted = z[order]
        unique_flat, start, counts = np.unique(
            flat_sorted, return_index=True, return_counts=True)
        percentile_rank = start + np.floor(HEIGHT_PERCENTILE * (counts - 1)).astype(np.int64)
        max_rank = start + counts - 1

        # Replace, do not accumulate. The SDK's fused cloud is the whole map
        # it has fused so far, re-sent every cycle, so a cell's height and
        # top come only from this message's points; a cell this message
        # does not mention keeps what it had, because the wrapper omits the
        # chunks that did not change.
        self._height.flat[unique_flat] = z_sorted[percentile_rank]
        self._top.flat[unique_flat] = z_sorted[max_rank]
        self._count.flat[unique_flat] = counts
        if observed_at is not None:
            # Exactly the cells height/top/count were just replaced for -
            # see the module docstring on why that set is the honest
            # "touched this cycle" set even though the SDK's cloud is
            # cumulative.
            self._stamp.flat[unique_flat] = float(observed_at)
        self._note_touched(unique_flat, cols)

    def _note_touched(self, unique_flat: np.ndarray, cols: int) -> None:
        """Remembers the 2.5 m tiles whose published 51x51 samples include
        any of these (flat, already unique) cells. A cell `c` is published
        by tile `c // TILE_CELLS` and - as the halo sample on the far edge -
        by tile `(c - 1) // TILE_CELLS`, so both are taken per axis. Works
        from the cells, not the points: a cloud is ~200k points but ~25k
        cells, and the sort is already paid. Read and reset with
        `take_touched_tiles()`."""
        cx = unique_flat % cols + self._origin_ix
        cy = unique_flat // cols + self._origin_iy
        tx = np.stack([cx // TILE_CELLS, (cx - 1) // TILE_CELLS])      # own, halo
        ty = np.stack([cy // TILE_CELLS, (cy - 1) // TILE_CELLS])
        # All four (x-choice, y-choice) combinations per cell - a tile
        # corner cell is the halo sample of three neighbours.
        all_tx = np.concatenate([tx[0], tx[1], tx[0], tx[1]])
        all_ty = np.concatenate([ty[0], ty[0], ty[1], ty[1]])
        # Packed into one int64 for a 1-D unique: np.unique(axis=0) sorts a
        # structured view and is ~50x slower on 100k rows.
        packed = np.unique((all_tx.astype(np.int64) << 32) + (all_ty.astype(np.int64) & 0xFFFFFFFF))
        xs = (packed >> 32).astype(np.int32)
        ys = (packed & 0xFFFFFFFF).astype(np.uint32).astype(np.int32)   # two's complement back
        self._touched_tiles.update(zip(xs.tolist(), ys.tolist()))

    def take_touched_tiles(self) -> set:
        """The tiles touched by update()s since the last call, then none."""
        touched, self._touched_tiles = self._touched_tiles, set()
        return touched

    def height_at(self, ix_cells: np.ndarray, iy_cells: np.ndarray) -> np.ndarray:
        """The 20th-percentile height already stored for these absolute
        grid cells (same index convention as `update()`: `floor(x /
        resolution)`) - NaN where the cell is outside the grid's current
        window or has never been seen. This reads whatever `update()` last
        wrote, so a caller after `update()` in the same cloud sees *this*
        update's heights, which is what the obstacle voxeliser needs as its
        ground reference."""
        ix_cells = np.asarray(ix_cells, dtype=np.int64)
        iy_cells = np.asarray(iy_cells, dtype=np.int64)
        out = np.full(ix_cells.shape, np.nan, dtype=np.float64)
        if self._origin_ix is None:
            return out
        rows, cols = self._height.shape
        inside = ((ix_cells >= self._origin_ix) & (ix_cells < self._origin_ix + cols)
                  & (iy_cells >= self._origin_iy) & (iy_cells < self._origin_iy + rows))
        if not inside.any():
            return out
        flat = ((iy_cells[inside] - self._origin_iy) * cols
                + (ix_cells[inside] - self._origin_ix))
        seen = self._count.flat[flat] > 0
        values = np.full(flat.shape, np.nan, dtype=np.float64)
        values[seen] = self._height.flat[flat[seen]]
        out[inside] = values
        return out

    def _fit_window(self, ix: np.ndarray, iy: np.ndarray) -> None:
        low_x, high_x = int(ix.min()), int(ix.max())
        low_y, high_y = int(iy.min()), int(iy.max())

        if self._origin_ix is None:
            self._origin_ix, self._origin_iy = low_x, low_y
            cols = min(high_x - low_x + 1, self.max_cells)
            rows = min(high_y - low_y + 1, self.max_cells)
            self._height = np.zeros((rows, cols), dtype=np.float64)
            self._top = np.zeros((rows, cols), dtype=np.float64)
            self._count = np.zeros((rows, cols), dtype=np.int64)
            self._stamp = np.full((rows, cols), np.nan, dtype=np.float64)
            return

        rows, cols = self._height.shape
        new_ox, new_cols = self._clipped_axis(self._origin_ix, cols, low_x, high_x)
        new_oy, new_rows = self._clipped_axis(self._origin_iy, rows, low_y, high_y)

        if (new_rows, new_cols) == (rows, cols):
            return

        grown_height = np.zeros((new_rows, new_cols), dtype=np.float64)
        grown_top = np.zeros((new_rows, new_cols), dtype=np.float64)
        grown_count = np.zeros((new_rows, new_cols), dtype=np.int64)
        grown_stamp = np.full((new_rows, new_cols), np.nan, dtype=np.float64)
        r0 = self._origin_iy - new_oy
        c0 = self._origin_ix - new_ox
        grown_height[r0:r0 + rows, c0:c0 + cols] = self._height
        grown_top[r0:r0 + rows, c0:c0 + cols] = self._top
        grown_count[r0:r0 + rows, c0:c0 + cols] = self._count
        grown_stamp[r0:r0 + rows, c0:c0 + cols] = self._stamp
        self._height, self._top, self._count = grown_height, grown_top, grown_count
        self._stamp = grown_stamp
        self._origin_ix, self._origin_iy = new_ox, new_oy

    def clear(self) -> None:
        self._touched_tiles = set()
        self._origin_ix = self._origin_iy = None
        self._height = np.zeros((0, 0), dtype=np.float64)
        self._top = np.zeros((0, 0), dtype=np.float64)
        self._count = np.zeros((0, 0), dtype=np.int64)
        self._stamp = np.full((0, 0), np.nan, dtype=np.float64)
        self.points_outside_cap = 0

    def state(self) -> GridState | None:
        if self._origin_ix is None:
            return None
        snapshot = self.snapshot()
        return GridState(elevation=snapshot.elevation, top=snapshot.top,
                         count=self._count.astype(np.int32),
                         stamp=snapshot.stamp,
                         origin_ix=int(self._origin_ix), origin_iy=int(self._origin_iy),
                         resolution=self.resolution)

    def replace(self, state: GridState, now: float = None) -> None:
        """The grid becomes `state`, exactly. Internal storage is rebuilt
        directly from `state.elevation`/`state.top` (NaN -> 0 where unseen,
        harmless since `count` masks those cells everywhere they are read),
        so a later update() replaces a touched cell exactly as it would
        have on the original.

        `now`: the ROS time (float seconds) to stamp every seen cell at
        when `state.stamp` is `None` - an .npz saved before stamps existed,
        which is every .npz today, since map_store.py's save()/load() have
        not been taught this field yet. "Observed now" is deliberate, not
        an approximation of the truth: the alternative, leaving every cell
        NaN ("never observed"), would make a freshly resumed map decay
        away the instant a downstream consumer starts reading age from it,
        which is worse than not knowing the real age at all. `now` is
        required in that case - this grid has no clock of its own, and a
        silent fallback would risk mixing time bases (wall clock vs a ROS
        clock under sim time) the caller never asked for."""
        self._touched_tiles = set()
        if state.resolution != self.resolution:
            raise ValueError(
                f"map was built at {state.resolution} m cells, this grid "
                f"uses {self.resolution} m; refusing to mix them")
        rows, cols = state.elevation.shape
        if max(rows, cols) > self.max_cells:
            raise ValueError(f"map is {cols} x {rows} cells, above the cap {self.max_cells}")
        self._origin_ix, self._origin_iy = int(state.origin_ix), int(state.origin_iy)
        self._count = state.count.astype(np.int64)
        self._height = np.nan_to_num(state.elevation.astype(np.float64), nan=0.0)
        self._top = np.nan_to_num(state.top.astype(np.float64), nan=0.0)
        if state.stamp is not None:
            self._stamp = np.asarray(state.stamp, dtype=np.float64)
        else:
            if now is None:
                raise ValueError(
                    "state has no per-cell observation stamp (an old save) "
                    "and no `now` was given to time-stamp it as 'observed "
                    "now' - the caller must supply the current time here")
            self._stamp = np.where(self._count > 0, float(now), np.nan)
        self.points_outside_cap = 0

    def _clipped_axis(self, origin: int, size: int, low: int, high: int):
        """New (origin, size) for one axis, grown to cover [low, high] but
        never past max_cells.

        Existing cells are never dropped, so when the full union would
        overshoot the cap this clips rather than aborting: whichever side
        still has room is extended - the low side first, then whatever
        budget is left goes to the high side, the same anchor-low,
        clip-high bias the very first window uses (`min(required,
        max_cells)`). A point that still falls outside the clipped window
        is left for update()'s own inside-check to count as outside the
        60 m cap, rather than silently frozen out by a window that never
        grew at all.
        """
        low_deficit = max(0, origin - low)
        high_deficit = max(0, (high + 1) - (origin + size))
        available = self.max_cells - size

        if low_deficit + high_deficit <= available:
            return origin - low_deficit, size + low_deficit + high_deficit

        low_ext = min(low_deficit, available)
        high_ext = min(high_deficit, available - low_ext)
        return origin - low_ext, size + low_ext + high_ext

    def snapshot(self):
        if self._origin_ix is None:
            return None
        filled = self._count > 0
        elevation = np.full(self._height.shape, np.nan, dtype=np.float32)
        elevation[filled] = self._height[filled].astype(np.float32)
        top = np.full(self._top.shape, np.nan, dtype=np.float32)
        top[filled] = self._top[filled].astype(np.float32)
        rows, cols = elevation.shape
        return GridSnapshot(
            elevation=elevation,
            top=top,
            # Copied, not handed out live: a caller mutating this must
            # never be able to corrupt the grid's own notion of when a
            # cell was last written. float64 throughout - see the module
            # docstring and GridState.stamp for why an absolute ROS
            # second must never be narrowed to float32.
            stamp=self._stamp.copy(),
            center_x=(self._origin_ix + cols / 2.0) * self.resolution,
            center_y=(self._origin_iy + rows / 2.0) * self.resolution,
            resolution=self.resolution,
            origin_ix=int(self._origin_ix), origin_iy=int(self._origin_iy))

    def stamp_snapshot(self):
        """(stamp, origin_ix, origin_iy) - the raw per-cell observation-time
        array (float64 seconds, NaN where never written) and the lattice
        origin it is indexed from, or None before the first point.

        Kept separate from snapshot(): a stamp is not read at the moment a
        cloud lands, but later, right before a tile is cut for publishing,
        when it is turned into an age relative to "now" (see
        elevation_mapper's tile builder). Bundling it into every
        snapshot() call would cost two extra full-grid copies (elevation,
        top) on every read of a value nothing but that one call site
        needs."""
        if self._origin_ix is None:
            return None
        return self._stamp.copy(), self._origin_ix, self._origin_iy
