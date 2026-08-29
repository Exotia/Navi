"""A growing 2.5-D elevation grid built from the ZED's fused point cloud.

Pure numpy on purpose: no rclpy, no message types, no camera. The frame
arithmetic and the growth rules are the part that can be wrong in a way a
picture would not show, so they are the part that gets tested on a laptop in
milliseconds rather than on a rover in an afternoon.

Storage convention, which is *not* grid_map's: row 0 is the smallest y and
column 0 the smallest x, both ascending, because that is what everything
except grid_map means by an array of ground. The conversion into grid_map's
own convention happens once, in elevation_mapper.build_grid_map_message.
"""

from dataclasses import dataclass

import numpy as np

# The spec's numbers. RESOLUTION also has to match the ZED wrapper's
# mapping.resolution in config/zed_front.yaml - a grid finer than the cloud
# would produce a comb of empty cells - and test_localization_launch.py
# checks that it does.
RESOLUTION = 0.10
MAX_EXTENT_M = 60.0
MAX_CELLS = int(round(MAX_EXTENT_M / RESOLUTION))   # 600


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


# eq=False: a generated __eq__ would compare numpy arrays with ==, which
# returns an array and then raises when the dataclass takes its truth value.
# Comparison goes through equals(), which knows about NaN.
@dataclass(frozen=True, eq=False)
class GridSnapshot:
    """One published state of the map. Comparable, so republishing can stop."""

    elevation: np.ndarray      # (rows, cols) float32; row 0 = min y, col 0 = min x
    center_x: float
    center_y: float
    resolution: float

    def equals(self, other) -> bool:
        if other is None:
            return False
        return (self.elevation.shape == other.elevation.shape
                and self.center_x == other.center_x
                and self.center_y == other.center_y
                and self.resolution == other.resolution
                # equal_nan, or every empty cell would count as a change and
                # the map would go out at 0.5 Hz forever.
                and np.array_equal(self.elevation, other.elevation, equal_nan=True))


class ElevationGrid:
    """Mean z per cell over a window that grows with the map, up to 60 x 60 m."""

    def __init__(self, resolution: float = RESOLUTION, max_cells: int = MAX_CELLS):
        self.resolution = float(resolution)
        self.max_cells = int(max_cells)
        self.points_outside_cap = 0
        # Lattice index of column 0 / row 0. None until the first point.
        self._origin_ix = None
        self._origin_iy = None
        self._sum = np.zeros((0, 0), dtype=np.float64)
        self._count = np.zeros((0, 0), dtype=np.int64)

    def update(self, points) -> None:
        kept = finite_points(points)
        if kept.size == 0:
            return
        ix = np.floor(kept[:, 0] / self.resolution).astype(np.int64)
        iy = np.floor(kept[:, 1] / self.resolution).astype(np.int64)
        self._fit_window(ix, iy)

        rows, cols = self._sum.shape
        inside = ((ix >= self._origin_ix) & (ix < self._origin_ix + cols)
                  & (iy >= self._origin_iy) & (iy < self._origin_iy + rows))
        self.points_outside_cap += int((~inside).sum())
        if not inside.any():
            return

        flat = ((iy[inside] - self._origin_iy) * cols
                + (ix[inside] - self._origin_ix))
        size = self._sum.size
        new_count = np.bincount(flat, minlength=size).reshape(rows, cols)
        new_sum = np.bincount(flat, weights=kept[inside, 2],
                              minlength=size).reshape(rows, cols)

        # Replace, do not accumulate. The SDK's fused cloud is the whole map
        # it has fused so far, re-sent every cycle, so adding each message's
        # points to a running total would count the same point once per
        # publish and drag every mean towards whatever was seen most often.
        # Cells this message does not mention keep what they had, because
        # the wrapper omits the chunks that did not change.
        touched = new_count > 0
        self._count[touched] = new_count[touched]
        self._sum[touched] = new_sum[touched]

    def _fit_window(self, ix: np.ndarray, iy: np.ndarray) -> None:
        low_x, high_x = int(ix.min()), int(ix.max())
        low_y, high_y = int(iy.min()), int(iy.max())

        if self._origin_ix is None:
            self._origin_ix, self._origin_iy = low_x, low_y
            cols = min(high_x - low_x + 1, self.max_cells)
            rows = min(high_y - low_y + 1, self.max_cells)
            self._sum = np.zeros((rows, cols), dtype=np.float64)
            self._count = np.zeros((rows, cols), dtype=np.int64)
            return

        rows, cols = self._sum.shape
        new_ox = min(self._origin_ix, low_x)
        new_oy = min(self._origin_iy, low_y)
        new_cols = max(self._origin_ix + cols, high_x + 1) - new_ox
        new_rows = max(self._origin_iy + rows, high_y + 1) - new_oy

        if new_cols > self.max_cells or new_rows > self.max_cells:
            # At the ceiling the honest thing is to keep the ground already
            # mapped rather than slide the window and silently forget it.
            # The points that fall outside are counted, not hidden.
            return
        if (new_rows, new_cols) == (rows, cols):
            return

        grown_sum = np.zeros((new_rows, new_cols), dtype=np.float64)
        grown_count = np.zeros((new_rows, new_cols), dtype=np.int64)
        r0 = self._origin_iy - new_oy
        c0 = self._origin_ix - new_ox
        grown_sum[r0:r0 + rows, c0:c0 + cols] = self._sum
        grown_count[r0:r0 + rows, c0:c0 + cols] = self._count
        self._sum, self._count = grown_sum, grown_count
        self._origin_ix, self._origin_iy = new_ox, new_oy

    def snapshot(self):
        if self._origin_ix is None:
            return None
        elevation = np.full(self._sum.shape, np.nan, dtype=np.float32)
        filled = self._count > 0
        elevation[filled] = (self._sum[filled] / self._count[filled]).astype(np.float32)
        rows, cols = elevation.shape
        return GridSnapshot(
            elevation=elevation,
            center_x=(self._origin_ix + cols / 2.0) * self.resolution,
            center_y=(self._origin_iy + rows / 2.0) * self.resolution,
            resolution=self.resolution)
