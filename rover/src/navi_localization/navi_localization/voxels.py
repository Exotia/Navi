"""Voxelise the fused cloud into per-tile obstacle voxels - pure numpy, no ROS.

Everything the ZED sees that is not ground: from the same update the
elevation grid bins, a point is an obstacle *candidate* when it sits more
than `OBSTACLE_MIN_ABOVE_GROUND` above the 20th-percentile ground height of
its own 5 cm grid cell and at most `OBSTACLE_MAX_ABOVE_ROVER` above the
rover's footprint (no pose yet -> no upper limit). A cell with no ground
height uses the rover's footprint z - 1.0 m as its reference; with no rover
pose either there is no reference at all and the point is dropped (a NaN
reference makes every comparison against it False, which is exactly "can't
tell, so don't call it an obstacle"). This reference is always taken at the
grid's own 5 cm cell, independent of the obstacle voxel size below.

A voxel (a `voxel_m`-metre cube, a parameter of `ObstacleMap` - **0.10** is
the node's default, independent of the grid's 5 cm cell; the fused cloud
carries only ~1 point per 5 cm on a surface, so 5 cm voxels leave holes) is
*occupied* when at least `MIN_POINTS_PER_VOXEL` candidate points fall in it
- a flying pixel has one. Voxels are grouped per 2.5 m tile, whose boundary
is still the 5 cm cell's (`floor(x / 0.05) // 50`) regardless of the voxel
size, so tiles stay 2.5 m even when voxels are coarser. Within a tile,
voxels are sorted so equal content gives equal bytes (`ObstacleMap.state()`
and the scheduler's change detection both depend on that).

`ObstacleMap` replaces per (x, y) VOXEL COLUMN, not per whole tile: a
column touched by this update's points (by x/y, regardless of whether any
of them became a voxel - a column seen with only ground points must lose
its old obstacle voxels too) has its stored voxels in that column replaced
wholesale, including with nothing at all if none of its points qualified -
the SDK re-sends its whole fused map and drops what it re-fuses away, and
that is the only "decay" there is. A column no point of this update
touched is left exactly as it was; other columns of the same tile are
never disturbed. (Replacing whole tiles used to wipe every voxel of a tile
whenever any one point in it moved, shrinking a mapped room to stripes
every cycle - see docs/superpowers/specs/2026-08-30-obstacle-voxels-design.md.)
Column membership is tracked at the voxel's own size: since `voxel_m` is
always a whole multiple of the 5 cm cell, "this voxel column was touched"
and "one of the cells it covers was touched" are the same test.
"""

import numpy as np

from navi_localization.elevation_grid import finite_points
from navi_localization.tiles import TILE_M

from navi_localization.elevation_grid import RESOLUTION as VOXEL  # the grid's 5 cm cell
OBSTACLE_MIN_ABOVE_GROUND = 0.10
OBSTACLE_MAX_ABOVE_ROVER = 2.5
MIN_POINTS_PER_VOXEL = 2   # with the fused cloud at 2 cm a 5 cm voxel on a real surface holds ~6 points; a lone point is noise (1 was needed when the cloud was 5 cm)

_EMPTY_VOXELS = np.zeros((0, 3), dtype=np.int32)


def _floor_index(values: np.ndarray, size: float) -> np.ndarray:
    return np.floor(values / size).astype(np.int64)


def _cells_per_tile(voxel_m: float) -> int:
    """How many `voxel_m` voxel-columns span one 2.5 m tile.

    `round`, not a bare division: floats make `2.5 / 0.1` alone
    `24.999999999999996`. `voxel_m` is expected to be a whole multiple of
    the grid's 5 cm cell (0.05 or 0.10 today), so this comes out exact."""
    return int(round(TILE_M / voxel_m))


def _lexsort_rows(voxels: np.ndarray) -> np.ndarray:
    """Sort (N, 3) int rows by (ix, iy, iz) ascending, numerically - not by
    raw bytes, which would not respect sign for negative indices."""
    order = np.lexsort((voxels[:, 2], voxels[:, 1], voxels[:, 0]))
    return voxels[order]


# `np.unique(..., axis=0)` builds a structured view and is orders of
# magnitude slower than a 1-D unique - at 200k points the difference is the
# whole performance budget. Packing (ix, iy, iz) into one int64 (21 bits
# each, offset to stay non-negative - plenty for a 60 m map at 5 cm cells)
# lets uniqueness/counting run as a single 1-D np.unique instead.
_PACK_BITS = 21
_PACK_OFFSET = 1 << (_PACK_BITS - 1)
_PACK_MASK = (1 << _PACK_BITS) - 1


def _pack(ix: np.ndarray, iy: np.ndarray) -> np.ndarray:
    return ((ix + _PACK_OFFSET) << _PACK_BITS) | (iy + _PACK_OFFSET)


def _pack3(ix: np.ndarray, iy: np.ndarray, iz: np.ndarray) -> np.ndarray:
    return (_pack(ix, iy) << _PACK_BITS) | (iz + _PACK_OFFSET)


def _touched_tiles(tile_x: np.ndarray, tile_y: np.ndarray) -> set:
    """Which (tile_x, tile_y) pairs occur at all, found through a small
    dense grid scoped to their own bounding box rather than a sort - a tile
    is 2.5 m, so even a map spanning the full 60 m cap is only ~24 tiles a
    side, however many points touch it."""
    min_x, min_y = int(tile_x.min()), int(tile_y.min())
    range_x = int(tile_x.max()) - min_x + 1
    range_y = int(tile_y.max()) - min_y + 1
    grid = np.zeros((range_x, range_y), dtype=bool)
    grid[tile_x - min_x, tile_y - min_y] = True
    xs, ys = np.nonzero(grid)
    return {(int(x) + min_x, int(y) + min_y) for x, y in zip(xs.tolist(), ys.tolist())}


def _unpack3(keys: np.ndarray):
    iz = (keys & _PACK_MASK) - _PACK_OFFSET
    rest = keys >> _PACK_BITS
    iy = (rest & _PACK_MASK) - _PACK_OFFSET
    ix = (rest >> _PACK_BITS) - _PACK_OFFSET
    return ix, iy, iz


def _bucket_by_tile(sorted_voxels: np.ndarray, voxel_m: float = VOXEL) -> dict:
    """`sorted_voxels`: (M, 3) int32, already sorted by (ix, iy, iz) in
    `voxel_m` units. Split into per-tile arrays without disturbing that
    order - a contiguous slice of a sorted array is itself sorted."""
    if sorted_voxels.shape[0] == 0:
        return {}
    cells_per_tile = _cells_per_tile(voxel_m)
    tile_x = sorted_voxels[:, 0] // cells_per_tile
    tile_y = sorted_voxels[:, 1] // cells_per_tile
    # Sorting by (ix, iy, iz) does NOT put a tile's rows together: along
    # iy the rows of tile (0, 0) and (0, 1) alternate for every ix, so each
    # tile appears as many separate runs and a "one run per tile" split
    # would keep only the last run of each. A stable sort by tile key first
    # groups the runs while keeping the (ix, iy, iz) order inside a tile.
    order = np.lexsort((tile_y, tile_x))
    sorted_voxels = sorted_voxels[order]
    tile_x = tile_x[order]
    tile_y = tile_y[order]
    change = np.empty(len(sorted_voxels), dtype=bool)
    change[0] = True
    change[1:] = (tile_x[1:] != tile_x[:-1]) | (tile_y[1:] != tile_y[:-1])
    starts = np.flatnonzero(change)
    ends = np.append(starts[1:], len(sorted_voxels))
    return {(int(tile_x[s]), int(tile_y[s])): sorted_voxels[s:e]
            for s, e in zip(starts, ends)}


def _touched_and_voxels(points, ground_height, rover_z, voxel_m: float = VOXEL,
                        already_filtered=False):
    """The tile keys any finite point landed in, the voxel-column keys any
    finite point landed in (both regardless of candidacy), and the occupied
    voxels per tile, each already sorted.

    `already_filtered=True`: `points` is already `finite_points`' output -
    the elevation grid needs the same filtered array for the same cloud,
    and filtering twice would double a ~40 ms cost for 200k points with
    nothing gained (elevation_mapper._on_cloud filters once and hands the
    result to both)."""
    pts = points if already_filtered else finite_points(points)
    if pts.shape[0] == 0:
        return set(), np.zeros(0, dtype=np.int64), {}

    # The grid's own 5 cm cell: the ground reference is always taken here,
    # independent of the obstacle voxel size (the spec's rule).
    ix = _floor_index(pts[:, 0], VOXEL)
    iy = _floor_index(pts[:, 1], VOXEL)

    # This map's own voxel columns, for every finite point (candidate or
    # not) - what "touched" means for ObstacleMap.update's per-column
    # replace. voxel_m is always a whole multiple of the 5 cm cell, so a
    # voxel column's floor(x / voxel_m) already tells you whether any of
    # the cells it covers got a point - no need to track cells separately.
    vx = _floor_index(pts[:, 0], voxel_m)
    vy = _floor_index(pts[:, 1], voxel_m)
    cells_per_tile = _cells_per_tile(voxel_m)
    touched_tiles = _touched_tiles(vx // cells_per_tile, vy // cells_per_tile)
    touched_columns = np.unique(_pack(vx, vy))

    ground = np.asarray(ground_height(ix, iy), dtype=np.float64)
    if rover_z is not None:
        reference = np.where(np.isnan(ground), rover_z - 1.0, ground)
    else:
        reference = ground

    with np.errstate(invalid="ignore"):
        candidate = (pts[:, 2] - reference) > OBSTACLE_MIN_ABOVE_GROUND
        if rover_z is not None:
            candidate &= pts[:, 2] <= rover_z + OBSTACLE_MAX_ABOVE_ROVER

    if not candidate.any():
        return touched_tiles, touched_columns, {}

    vz = _floor_index(pts[candidate, 2], voxel_m)
    packed = _pack3(vx[candidate], vy[candidate], vz)
    packed.sort()   # in place: no need to trace back to the source points
    change = np.empty(len(packed), dtype=bool)
    change[0] = True
    change[1:] = packed[1:] != packed[:-1]
    starts = np.flatnonzero(change)
    counts = np.diff(np.append(starts, len(packed)))
    occupied_keys = packed[starts][counts >= MIN_POINTS_PER_VOXEL]
    if occupied_keys.shape[0] == 0:
        return touched_tiles, touched_columns, {}
    ox, oy, oz = _unpack3(occupied_keys)
    occupied = np.stack([ox, oy, oz], axis=1).astype(np.int32)
    return touched_tiles, touched_columns, _bucket_by_tile(_lexsort_rows(occupied), voxel_m)


def occupied_voxels(points, ground_height, rover_z, already_filtered=False,
                    voxel_m: float = VOXEL) -> dict:
    """`points`: (N, 3) float64. `ground_height(ix, iy)`: 5 cm cell indices
    -> float array, NaN where the cell has no ground yet. `rover_z`: the
    rover's footprint z, or None before the first pose. `voxel_m`: the
    obstacle voxel's own edge length, independent of the grid's 5 cm cell.
    Returns `{(tile_ix, tile_iy): (M, 3) int32}`, sorted, for tiles with at
    least one occupied voxel. `already_filtered`: see `_touched_and_voxels`."""
    _, _, voxels = _touched_and_voxels(points, ground_height, rover_z, voxel_m, already_filtered)
    return voxels


class ObstacleMap:
    """Per-tile storage of occupied obstacle voxels, mirroring the grid's
    tile store: `state()`/`replace()`/`clear()` for save/load, `tiles()` for
    what to publish. `voxel_m`: the edge length of one obstacle voxel -
    independent of the grid's 5 cm cell, see the module docstring."""

    def __init__(self, voxel_m: float = VOXEL):
        self.voxel_m = float(voxel_m)
        self._tiles = {}   # key -> (M, 3) int32, sorted; only non-empty tiles

    def update(self, points, ground_height, rover_z, already_filtered=False) -> None:
        touched_tiles, touched_columns, new_voxels = _touched_and_voxels(
            points, ground_height, rover_z, self.voxel_m, already_filtered)
        for key in touched_tiles:
            old = self._tiles.get(key)
            if old is not None and old.shape[0] > 0:
                # Columns this update did not mention keep their old
                # voxels; a column it did touch (candidate or not) loses
                # them here, and gets whatever `new_voxels` found there.
                # int64, not the array's own int32: _pack shifts left by
                # 2 * _PACK_BITS (42 bits), which overflows int32 for any
                # index at all - touched_columns is already int64 from
                # _floor_index, and this must match it bit for bit.
                old_columns = _pack(old[:, 0].astype(np.int64), old[:, 1].astype(np.int64))
                survivors = old[~np.isin(old_columns, touched_columns)]
            else:
                survivors = _EMPTY_VOXELS
            added = new_voxels.get(key)
            if added is None or added.shape[0] == 0:
                merged = survivors
            elif survivors.shape[0] == 0:
                merged = added
            else:
                merged = _lexsort_rows(np.concatenate([survivors, added]))
            if merged.shape[0] == 0:
                self._tiles.pop(key, None)
            else:
                self._tiles[key] = merged

    def tiles(self) -> dict:
        return dict(self._tiles)

    def state(self) -> np.ndarray:
        if not self._tiles:
            return _EMPTY_VOXELS.copy()
        return _lexsort_rows(np.concatenate(list(self._tiles.values()), axis=0))

    def replace(self, voxels, voxel_m: float = None) -> None:
        """The map becomes `voxels` exactly, at `voxel_m` if given.

        `voxel_m=None` (the caller does not know, or does not need to
        change it - e.g. map_store.py's npz `voxel_m` key defaults to 0.05
        for a map saved before the obstacle voxel size became a parameter)
        leaves this map's own `voxel_m` as it was; a given `voxel_m` is
        adopted outright, exactly like `state()`/`replace()` round-trip the
        grid's own resolution."""
        if voxel_m is not None:
            self.voxel_m = float(voxel_m)
        voxels = np.asarray(voxels, dtype=np.int32).reshape(-1, 3)
        if voxels.shape[0] == 0:
            self._tiles = {}
            return
        self._tiles = _bucket_by_tile(_lexsort_rows(voxels), self.voxel_m)

    def clear(self) -> None:
        self._tiles = {}

    @property
    def voxel_count(self) -> int:
        return sum(v.shape[0] for v in self._tiles.values())
