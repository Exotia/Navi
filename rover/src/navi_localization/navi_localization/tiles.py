"""The map as 2.5 m tiles: partition, halo, dirty tracking, scheduling.

Pure numpy, no ROS. A tile is 50 x 50 cells of the 5 cm grid aligned to the
map frame (tile (ix, iy) covers x in [2.5 ix, 2.5 (ix + 1))), published as
51 x 51 samples: its own cells plus one row and column of the +x / +y
neighbours, so adjacent tile meshes share their boundary vertices and the
terrain has no seams.

Why tiles: the whole map is 2 MB at 5 cm over the yard; a tile is 10 KB.
Publishing only the tiles that changed keeps the map traffic at a few tens
of KB/s however long the tour. Why the schedule: at most eight dirty tiles
a tick and one clean tile round-robin as keepalive, so a restarted sim gets
the rover's surroundings first and the whole yard within `tiles` seconds.
"""

from collections import deque

import numpy as np

TILE_CELLS = 50
TILE_SAMPLES = TILE_CELLS + 1
RESOLUTION = 0.05
TILE_M = TILE_CELLS * RESOLUTION                       # 2.5
DIRTY_THRESHOLD_M = 0.01
MAX_DIRTY_PER_TICK = 8
MIN_INTERVAL_S = 1.0
# Centre of the 51-sample lattice relative to the tile's origin corner:
# sample k sits at (k + 0.5) * RESOLUTION, so the centre is at 25.5 cells.
_CENTER_OFFSET = (TILE_SAMPLES / 2.0) * RESOLUTION     # 1.275


def tile_center(ix: int, iy: int) -> tuple[float, float]:
    return (TILE_M * ix + _CENTER_OFFSET, TILE_M * iy + _CENTER_OFFSET)


def tile_index_of(pose_x: float, pose_y: float) -> tuple[int, int]:
    return (int(round((pose_x - _CENTER_OFFSET) / TILE_M)),
            int(round((pose_y - _CENTER_OFFSET) / TILE_M)))


def tiles_of_snapshot(snapshot) -> dict:
    """Every tile with at least one seen cell of its own, as (51, 51) arrays."""
    elevation = np.asarray(snapshot.elevation, dtype=np.float32)
    rows, cols = elevation.shape
    ox, oy = int(snapshot.origin_ix), int(snapshot.origin_iy)
    # Floor division works for negative lattice indices too.
    first_tx, last_tx = ox // TILE_CELLS, (ox + cols - 1) // TILE_CELLS
    first_ty, last_ty = oy // TILE_CELLS, (oy + rows - 1) // TILE_CELLS

    out = {}
    for ty in range(first_ty, last_ty + 1):
        for tx in range(first_tx, last_tx + 1):
            # Lattice indices of the tile's 51 samples.
            xs = np.arange(tx * TILE_CELLS, tx * TILE_CELLS + TILE_SAMPLES) - ox
            ys = np.arange(ty * TILE_CELLS, ty * TILE_CELLS + TILE_SAMPLES) - oy
            tile = np.full((TILE_SAMPLES, TILE_SAMPLES), np.nan, dtype=np.float32)
            in_x = (xs >= 0) & (xs < cols)
            in_y = (ys >= 0) & (ys < rows)
            if not (in_x.any() and in_y.any()):
                continue
            tile[np.ix_(in_y, in_x)] = elevation[np.ix_(ys[in_y], xs[in_x])]
            own = tile[:TILE_CELLS, :TILE_CELLS]
            if np.isfinite(own).any():
                out[(tx, ty)] = tile
    return out


def _changed(new: np.ndarray, old) -> bool:
    if old is None:
        return True
    new_seen, old_seen = np.isfinite(new), np.isfinite(old)
    if (new_seen & ~old_seen).any():
        return True
    both = new_seen & old_seen
    return bool((np.abs(new[both] - old[both]) > DIRTY_THRESHOLD_M).any())


class TileScheduler:

    def __init__(self):
        self._latest = {}         # key -> latest tile offered
        self._published = {}      # key -> tile as last published (None = never)
        self._published_at = {}   # key -> time of last publication
        self._dirty = {}          # key -> time it became dirty (insertion ordered)
        self._round_robin = deque()

    def is_dirty(self, key) -> bool:
        return key in self._dirty

    def offer(self, tiles: dict, now: float) -> None:
        for key, tile in tiles.items():
            self._latest[key] = tile
            if key not in self._published:
                self._published[key] = None
                self._round_robin.append(key)
            if key not in self._dirty and _changed(tile, self._published[key]):
                self._dirty[key] = now

    def due(self, now: float) -> list:
        out = []
        for key in sorted(self._dirty, key=self._dirty.get):
            if len(out) == MAX_DIRTY_PER_TICK:
                break
            last = self._published_at.get(key)
            if last is None or now - last >= MIN_INTERVAL_S:
                out.append((key, self._latest[key]))
        chosen = {key for key, _ in out}
        # One keepalive: the next clean tile in round-robin order that is
        # not already going out this tick and was not published just now.
        for _ in range(len(self._round_robin)):
            key = self._round_robin[0]
            self._round_robin.rotate(-1)
            if key in chosen or key in self._dirty:
                continue
            last = self._published_at.get(key)
            # Strictly more than the interval, not >=: a tile that just left
            # the dirty batch this very tick boundary should not double as
            # this tick's keepalive too - that's the dirty path's job.
            if last is not None and now - last <= MIN_INTERVAL_S:
                continue
            out.append((key, self._latest[key]))
            break
        return out

    def published(self, key, tile: np.ndarray, now: float) -> None:
        self._published[key] = tile
        self._published_at[key] = now
        if key in self._dirty and not _changed(self._latest[key], tile):
            del self._dirty[key]

    def mark_all_dirty(self) -> None:
        for key in self._latest:
            self._dirty.setdefault(key, 0.0)

    def forget_all(self) -> list:
        keys = [key for key, tile in self._published.items() if tile is not None]
        self.__init__()
        return keys
