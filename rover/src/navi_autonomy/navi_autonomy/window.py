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
