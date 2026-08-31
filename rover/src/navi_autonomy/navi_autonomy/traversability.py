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


# nav_msgs/OccupancyGrid conventions: 0..100 cost, -1 unknown. 100 is
# reserved for lethal, so the scaled band tops out at 99 and a planner can
# tell "as bad as it gets while still driveable" from "do not".
LETHAL = 100
UNKNOWN = -1
MAX_SCALED_COST = 99


def costmap_seed(slope, step, roughness, valid,
                 step_lethal_m: float = STEP_LETHAL_M) -> np.ndarray:
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

    # step_lethal_m defaults to the spec's 0.14; the layer node exposes it
    # as a parameter because night sessions showed ZED depth noise in the
    # dark fabricating >0.14 m phantom steps - lethal cells on ground the
    # operator can see is flat. Raising it at night trades hole-margin for
    # not walling off the whole yard; the spec value stays the default.
    worst = np.maximum(
        np.maximum(np.clip(slope / SLOPE_LETHAL_RAD, 0.0, 1.0),
                   np.clip(step / float(step_lethal_m), 0.0, 1.0)),
        np.clip(roughness / ROUGHNESS_REF_M, 0.0, 1.0))
    cost = np.rint(MAX_SCALED_COST * worst).astype(np.int8)
    cost[valid < 0.5] = UNKNOWN
    cost[(step >= float(step_lethal_m)) | (slope >= SLOPE_LETHAL_RAD)] = LETHAL
    return cost


def _disc_offsets(radius_cells: int) -> np.ndarray:
    """(dy, dx) integer offsets of every cell within `radius_cells` of (0, 0),
    on a Euclidean disc. Computed once per `clear_startup_patch` call."""
    r = int(radius_cells)
    if r < 0:
        raise ValueError(f"radius_cells must be >= 0, got {radius_cells}")
    ys, xs = np.mgrid[-r:r + 1, -r:r + 1]
    disc = (ys * ys + xs * xs) <= r * r
    return np.stack((ys[disc], xs[disc]), axis=1)


def stamp_wheel_trail(cost: np.ndarray, trail_cells, radius_cells: int) -> np.ndarray:
    """Ground the wheels have actually rolled over is proof-of-traversable,
    and that proof outranks every camera opinion - including a measured
    LETHAL. Live night finding (2026-09-01): the ZED's z drifted 0.8 m in
    the dark, old and new elevation data met in metre-high phantom "steps",
    and the rover ended stranded on cost-99 ground it had just driven over,
    with every planner refusing the start pose.

    `trail_cells` is an iterable of (row, col) centres - the rover's pose
    history in THIS seed's cell frame. Each gets a free (0) disc of
    `radius_cells`. The radius MUST stay inside the footprint's inscribed
    circle: this is what separates the trail from the rejected moving
    clearing disc (see clear_startup_patch below) - it never touches ground
    beside the chassis, only ground the chassis provably covered.

    The known trade, accepted: an obstacle that ARRIVES on the old trail
    later (a rolled rock) is erased until the camera maps it again on top -
    the yard is static during a run, and a stranded rover was the real
    failure. Mutates `cost` in place and returns it.
    """
    cost = np.asarray(cost)
    if cost.ndim != 2:
        raise ValueError(f"a cost grid is 2-D, got shape {cost.shape}")
    if radius_cells <= 0:
        return cost
    rows, cols = cost.shape
    offsets = _disc_offsets(radius_cells)
    for row, col in trail_cells:
        ry = offsets[:, 0] + int(row)
        rx = offsets[:, 1] + int(col)
        in_bounds = (ry >= 0) & (ry < rows) & (rx >= 0) & (rx < cols)
        cost[ry[in_bounds], rx[in_bounds]] = 0
    return cost


def clear_startup_patch(cost: np.ndarray, centre_cell, radius_cells: int) -> np.ndarray:
    """"The wheels have been here" - the ground the rover is already sitting
    on at start-up is proof-of-traversable, so it overrides the "unknown =
    wall" default for one disc around the rover's first-seen pose.

    `centre_cell` is a single (row, col) cell coordinate - the node's first
    pose after start-up, converted once and reused for the node's lifetime.
    Every cell within `radius_cells` (a disc, not a square) of it that is
    currently UNKNOWN (-1) is set to free (0). Pass `None` to clear nothing
    (no pose seen yet).

    This is deliberately a single fixed patch, not a clearing disc that
    follows the rover as it drives (a "track"). A moving disc is wider than
    the wheels that supposedly proved it, so as the rover drives past a big
    obstacle the disc would slowly clear unseen ground beside it that the
    wheels never touched - "chipping away" at ground that may not be
    traversable at all. One startup patch avoids that: it proves exactly the
    ground the rover was demonstrably sitting on, once. A node restart
    re-seeds the patch at wherever the rover is then, which the wheels prove
    again.

    A *measured* cell is never touched, in either direction: anything already
    >= 0 - including a camera-seen LETHAL 100 - is left exactly as it is,
    now and forever after. The instant the camera maps a patch cell, that
    measurement wins and keeps winning; the patch only fills in what the
    camera has not yet reached.

    Mutates `cost` in place (and returns it, for chaining) - the caller
    already owns a fresh seed array by the time this runs, so there is
    nothing to protect by copying.
    """
    cost = np.asarray(cost)
    if cost.ndim != 2:
        raise ValueError(f"a cost grid is 2-D, got shape {cost.shape}")
    rows, cols = cost.shape
    if centre_cell is None or radius_cells <= 0:
        return cost

    row, col = centre_cell
    offsets = _disc_offsets(radius_cells)
    ry = offsets[:, 0] + int(row)
    rx = offsets[:, 1] + int(col)
    in_bounds = (ry >= 0) & (ry < rows) & (rx >= 0) & (rx < cols)
    ry = ry[in_bounds]
    rx = rx[in_bounds]
    unknown = cost[ry, rx] == UNKNOWN
    cost[ry[unknown], rx[unknown]] = 0
    return cost


def seed_from_elevation(elevation, resolution: float = RESOLUTION,
                        step_lethal_m: float = STEP_LETHAL_M) -> tuple:
    """(the four layers, the int8 cost grid) - the whole derive in one call."""
    layers = derive(elevation, resolution)
    cost = costmap_seed(layers['slope'], layers['step'],
                        layers['roughness'], layers['valid'],
                        step_lethal_m=step_lethal_m)
    return layers, cost
