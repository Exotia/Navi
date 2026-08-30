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
