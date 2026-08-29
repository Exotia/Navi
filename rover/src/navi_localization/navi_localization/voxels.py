"""Voxelise the fused cloud into per-tile obstacle voxels - pure numpy, no ROS.

Everything the ZED sees that is not ground: from the same update the
elevation grid bins, a point is an obstacle *candidate* when it sits more
than `OBSTACLE_MIN_ABOVE_GROUND` above the 20th-percentile ground height of
its own cell and at most `OBSTACLE_MAX_ABOVE_ROVER` above the rover's
footprint (no pose yet -> no upper limit). A cell with no ground height
uses the rover's footprint z - 1.0 m as its reference; with no rover pose
either there is no reference at all and the point is dropped (a NaN
reference makes every comparison against it False, which is exactly
"can't tell, so don't call it an obstacle").

A voxel (5 cm cube, the same resolution as the grid) is *occupied* when at
least `MIN_POINTS_PER_VOXEL` candidate points fall in it - a flying pixel
has one. Voxels are grouped per 2.5 m tile `(ix // 50, iy // 50)`, sorted so
equal content gives equal bytes (`ObstacleMap.state()` and the scheduler's
change detection both depend on that).

`ObstacleMap` mirrors the grid's replace-per-tile rule: a tile touched by
this update's points (by x/y cell, regardless of whether any of them
became a voxel) has its stored voxels replaced wholesale, including with
nothing at all if none of its points qualified - the SDK re-sends its whole
fused map and drops what it re-fuses away, and that is the only "decay"
there is. A tile no point of this update touched is left exactly as it
was.
"""

import numpy as np

from navi_localization.elevation_grid import finite_points
from navi_localization.tiles import TILE_CELLS

VOXEL = 0.05
OBSTACLE_MIN_ABOVE_GROUND = 0.10
OBSTACLE_MAX_ABOVE_ROVER = 2.5
MIN_POINTS_PER_VOXEL = 2


def _floor_index(values: np.ndarray) -> np.ndarray:
    return np.floor(values / VOXEL).astype(np.int64)


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


def _bucket_by_tile(sorted_voxels: np.ndarray) -> dict:
    """`sorted_voxels`: (M, 3) int32, already sorted by (ix, iy, iz). Split
    into per-tile arrays without disturbing that order - a contiguous slice
    of a sorted array is itself sorted."""
    if sorted_voxels.shape[0] == 0:
        return {}
    tile_x = sorted_voxels[:, 0] // TILE_CELLS
    tile_y = sorted_voxels[:, 1] // TILE_CELLS
    change = np.empty(len(sorted_voxels), dtype=bool)
    change[0] = True
    change[1:] = (tile_x[1:] != tile_x[:-1]) | (tile_y[1:] != tile_y[:-1])
    starts = np.flatnonzero(change)
    ends = np.append(starts[1:], len(sorted_voxels))
    return {(int(tile_x[s]), int(tile_y[s])): sorted_voxels[s:e]
            for s, e in zip(starts, ends)}


def _touched_and_voxels(points, ground_height, rover_z):
    """The tile keys any finite point landed in (by x/y cell, regardless of
    candidacy) and the occupied voxels per tile, each already sorted."""
    pts = finite_points(points)
    if pts.shape[0] == 0:
        return set(), {}

    # iz is only needed for the (usually far fewer) candidate points, so it
    # is computed after filtering rather than for every point up front.
    ix = _floor_index(pts[:, 0])
    iy = _floor_index(pts[:, 1])
    touched = _touched_tiles(ix // TILE_CELLS, iy // TILE_CELLS)

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
        return touched, {}

    iz = _floor_index(pts[candidate, 2])
    packed = _pack3(ix[candidate], iy[candidate], iz)
    packed.sort()   # in place: no need to trace back to the source points
    change = np.empty(len(packed), dtype=bool)
    change[0] = True
    change[1:] = packed[1:] != packed[:-1]
    starts = np.flatnonzero(change)
    counts = np.diff(np.append(starts, len(packed)))
    occupied_keys = packed[starts][counts >= MIN_POINTS_PER_VOXEL]
    if occupied_keys.shape[0] == 0:
        return touched, {}
    vx, vy, vz = _unpack3(occupied_keys)
    occupied = np.stack([vx, vy, vz], axis=1).astype(np.int32)
    return touched, _bucket_by_tile(_lexsort_rows(occupied))


def occupied_voxels(points, ground_height, rover_z) -> dict:
    """`points`: (N, 3) float64. `ground_height(ix, iy)`: cell indices ->
    float array, NaN where the cell has no ground yet. `rover_z`: the
    rover's footprint z, or None before the first pose. Returns
    `{(tile_ix, tile_iy): (M, 3) int32}`, sorted, for tiles with at least
    one occupied voxel."""
    _, voxels = _touched_and_voxels(points, ground_height, rover_z)
    return voxels


class ObstacleMap:
    """Per-tile storage of occupied obstacle voxels, mirroring the grid's
    tile store: `state()`/`replace()`/`clear()` for save/load, `tiles()` for
    what to publish."""

    def __init__(self):
        self._tiles = {}   # key -> (M, 3) int32, sorted; only non-empty tiles

    def update(self, points, ground_height, rover_z) -> None:
        touched, new_voxels = _touched_and_voxels(points, ground_height, rover_z)
        for key in touched:
            voxels = new_voxels.get(key)
            if voxels is None or voxels.shape[0] == 0:
                self._tiles.pop(key, None)
            else:
                self._tiles[key] = voxels

    def tiles(self) -> dict:
        return dict(self._tiles)

    def state(self) -> np.ndarray:
        if not self._tiles:
            return np.zeros((0, 3), dtype=np.int32)
        return _lexsort_rows(np.concatenate(list(self._tiles.values()), axis=0))

    def replace(self, voxels) -> None:
        voxels = np.asarray(voxels, dtype=np.int32).reshape(-1, 3)
        if voxels.shape[0] == 0:
            self._tiles = {}
            return
        self._tiles = _bucket_by_tile(_lexsort_rows(voxels))

    def clear(self) -> None:
        self._tiles = {}

    @property
    def voxel_count(self) -> int:
        return sum(v.shape[0] for v in self._tiles.values())
