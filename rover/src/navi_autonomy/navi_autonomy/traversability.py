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

import functools
import math
import warnings

import numpy as np

from navi_localization.elevation_grid import RESOLUTION

# Spec section 5. "step lethal above 0.14 m - just over the 0.125 m wheel
# radius. Computed as max neighbour difference, so it catches negative steps
# (holes) as well as positive ones." "slope lethal above 25 degrees,
# roughness scaled below that."
#
# Raised from the spec's 0.14 to 0.25 on the operator's judgement after live
# runs: the rover was refusing ground it drives over without noticing. The
# number is the chassis geometry rather than a guess. Wheels are 0.125 m in
# radius, and the belly - a 0.254 m body box centred 0.409 m up (see
# asterope_iiI.urdf) - clears the ground by 0.282 m. So at 0.25 m an
# obstacle is either climbed, on wheels with twice that radius' worth of
# leverage, or passed under the chassis with 3 cm to spare. 0.282 m is the
# wall: past it the rover high-centres, and no operator judgement moves a
# number past the point where the body touches the rock.
#
# KNOWN COST, accepted deliberately: `step` is the maximum ABSOLUTE
# difference to a neighbour, so this one number governs "a rock I can climb"
# and "a hole I can fall into" alike. A 0.25 m hole drops a wheel in past
# its axle, which is not the same bet as climbing a 0.25 m rock. Splitting
# the two - a separate, tighter limit for ground that falls AWAY - is the
# fix, and it is deliberately not in this commit.
#
# What this does NOT change is the slope threshold below. A step is a thing
# to climb and getting it wrong strands a wheel; a slope is a thing to stand
# on and getting it wrong rolls the rover onto its side. They deserve
# different courage.
STEP_LETHAL_M = 0.25

# Rover-relative lethality, alongside `step` rather than instead of it.
# `step` is local by construction - the max difference to a cell's own eight
# neighbours - and that has a blind spot: a rise that accumulates gradually
# is invisible to it. A staircase of five 0.10 m steps climbs half a metre,
# and every individual step reads comfortably under STEP_LETHAL_M, so every
# cell along it comes back drivable while the rover cannot in fact mount
# what is in front of it. What actually determines whether the rover can
# mount something is that thing's height above the ground the rover is
# standing on, not its height above its own neighbours - see
# `height_relative_to` and the `rover_z`/`rover_cell` arguments below.
#
# How far above the rover's own ground a cell may sit and still be
# something the rover could mount. Same 0.25 m as STEP_LETHAL_M and for
# the same chassis reason.
CLIMB_LETHAL_M = 0.25

# How far BELOW the rover's ground a cell may sit. Tighter than the climb
# on purpose: a wheel 0.125 m in radius climbs a 0.25 m rock with help
# from the chassis, but drops into a 0.25 m hole past its axle and lands
# the belly on the rim. Getting a climb wrong strands a wheel; getting a
# drop wrong ends the run on the rover's stomach.
DROP_LETHAL_M = 0.14

# Default radius (metres) within which the rover-relative test above is
# allowed to say anything at all. See costmap_seed's docstring - this
# number is not a tuning knob for sensitivity, it is what keeps the test
# from being asked a question it cannot answer.
RELATIVE_RADIUS_M = 3.0

# 30 degrees - the operator's final number, after a full tour: the spec
# said 25, live runs pushed it to 35 and then 45 while measurement noise
# was inflating every reading, and once the fitted slope removed the noise
# the operator settled on 30 as what the rover actually climbs. It is also
# exactly the ground/obstacle boundary the reference stack that keeps
# winning ERC uses (kalman_robot, max_ground_angle 0.7 rad). The static
# tip is ~47 degrees sideways, so 30 carries a wide margin; with the
# fitted slope a 30 here is a real 30, not 30 minus whatever the noise
# added. Live-retunable (slope_lethal_deg) as ever.
SLOPE_LETHAL_DEG = 30.0
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
    """Ground inclination in radians: atan of the gradient magnitude.

    The raw two-cell gradient - see slope_layer_fitted for the smoothed
    slope the cost actually judges by. This one stays available for
    anyone debugging the fit against the noise it exists to average out."""
    grid = _as_grid(elevation)
    rows, cols = grid.shape
    padded = _padded(grid)
    gx = _gradient(_shift(padded, 0, 1, rows, cols), _shift(padded, 0, -1, rows, cols),
                   grid, resolution)
    gy = _gradient(_shift(padded, 1, 0, rows, cols), _shift(padded, -1, 0, rows, cols),
                   grid, resolution)
    slope = np.arctan(np.hypot(gx, gy)).astype(np.float32)
    return np.where(np.isfinite(grid), slope, np.nan).astype(np.float32)


#: The half-size of the neighbourhood a cell's ground plane is fitted
#: over. 0.2 m matches the reference stack that beat everyone at ERC, and
#: it is the scale at which centimetre depth noise stops looking like
#: terrain: 2 cm over this plane's 0.4 m span is under 3 degrees, where
#: the raw two-cell gradient read the same noise as 11.
SLOPE_FIT_RADIUS_M = 0.2

# A plane through 3 points is exact by construction, whether the 3 points
# are real ground or 3 points of pure sensor noise - it cannot disagree
# with itself and say so. 6 is the first count at which a least-squares
# fit has a genuine residual: more equations than unknowns, so the fit
# can actually be wrong about a cell rather than merely pass through it.
_SLOPE_FIT_MIN_SUPPORT = 6


def _windowed_sum(values: np.ndarray, half: int) -> np.ndarray:
    """Sum of `values` over the (2*half+1) square centred on each cell,
    clipped at the grid edge - a border cell gets a smaller, one-sided
    window, the same convention `_padded`'s NaN neighbours use elsewhere in
    this module: a missing cell contributes nothing, it never wraps and it
    is never invented.

    A summed-area table: the double cumulative sum turns a window query of
    ANY radius into four array lookups, so this function's cost does not
    grow with `half`. That is what keeps slope_fit_radius_m safe to widen
    on a live retune - a bigger neighbourhood costs nothing extra here,
    unlike a sliding-window convolution whose cost is the window's area.

    Zero-padded by `half` on every side before the integral image is
    built, rather than clipped by indexing into it: the padding turns
    every window - edge cells included - into the SAME fixed (2*half+1)
    square, so the four corners of that square are always a plain slice of
    the integral image, never a per-pixel gather (which measured 2x slower
    on a 960 x 960 grid). A zero contributes nothing to any of the sums
    this function is used for (a count, or a value already zeroed at an
    unseen cell), so the padding is exactly the missing neighbour it
    stands in for. The padded values are written directly into the
    integral array's own border and cumulatively summed in place
    (`out=integral`), which is the other half of that measured saving -
    one array carries both the padding and the running sum, instead of a
    padded copy handed to a second, freshly allocated one."""
    rows, cols = values.shape
    span = 2 * half + 1
    integral = np.zeros((rows + span, cols + span), dtype=np.float64)
    integral[half + 1:half + 1 + rows, half + 1:half + 1 + cols] = values
    np.cumsum(integral, axis=0, out=integral)
    np.cumsum(integral, axis=1, out=integral)
    return (integral[span:, span:] - integral[:-span, span:]
            - integral[span:, :-span] + integral[:-span, :-span])


def slope_layer_fitted(elevation, resolution: float = RESOLUTION,
                       fit_radius_m: float = SLOPE_FIT_RADIUS_M) -> np.ndarray:
    """Ground inclination in radians from a least-squares plane fitted to
    each cell's neighbourhood, replacing the raw two-cell gradient as the
    slope the costmap judges.

    slope_layer takes its gradient from two adjacent cells - a 0.10 m
    baseline at this grid's resolution - so 2 cm of ZED depth noise reads
    as atan(0.02 / 0.10) = 11 degrees of phantom incline, and doubles when
    the noise runs the other way on the far side. Fitting a plane over a
    wider neighbourhood is the reference stack's whole trick (AGH's
    kalman_robot, multiple ERC wins): the same 2 cm of noise spread over
    `fit_radius_m`'s 0.4 m span averages down to under 3 degrees, while a
    real slope's gradient survives the averaging unchanged, because every
    cell in the window agrees with it.

    The window is a square of half-width round(fit_radius_m / resolution)
    cells, not a Euclidean disc: a square's row and column sums are exactly
    what a summed-area table gives for free, where a disc's round edge
    would need a per-radius mask rebuilt on every retune. The corners a
    square adds over a disc are a small fraction of the window and do not
    change which cells the fit calls noise.

    A cell's own fit uses every finite cell in its window; a NaN cell
    contributes to no fit but its own absence. The result is NaN where the
    cell itself is unseen, or where fewer than 6 finite cells support its
    fit - see _SLOPE_FIT_MIN_SUPPORT for why 3 is not enough."""
    grid = _as_grid(elevation)
    rows, cols = grid.shape
    half = int(round(float(fit_radius_m) / float(resolution)))
    if half < 1:
        # A window under one cell wide cannot report a gradient at all -
        # this only happens if a live retune sets the radius to (near)
        # zero, and a single-cell window is a saner fallback than a
        # guaranteed-NaN layer.
        half = 1

    finite = np.isfinite(grid)
    z = np.where(finite, grid, 0.0).astype(np.float64)
    m = finite.astype(np.float64)
    # Row vectors, not two full (rows, cols) coordinate grids: x only
    # varies along columns and y only along rows, so broadcasting keeps
    # every array below at its natural size until the multiply that
    # actually needs the full grid forces it. Centred on the grid's own
    # middle rather than on cell (0, 0) - a linear fit's slope is
    # invariant to shifting every coordinate by the same constant, so this
    # changes no answer, it only keeps the numbers in the normal equations
    # small regardless of how large the grid is.
    x = (np.arange(cols, dtype=np.float64) - cols / 2.0)[None, :]
    y = (np.arange(rows, dtype=np.float64) - rows / 2.0)[:, None]

    n = _windowed_sum(m, half)
    sx = _windowed_sum(x * m, half)
    sy = _windowed_sum(y * m, half)
    sxx = _windowed_sum(x * x * m, half)
    syy = _windowed_sum(y * y * m, half)
    sxy = _windowed_sum(x * y * m, half)
    sz = _windowed_sum(z, half)
    sxz = _windowed_sum(x * z, half)
    syz = _windowed_sum(y * z, half)

    # Cramer's rule on the 3x3 normal equations for z = a + b*x + c*y,
    # solved elementwise across the whole grid at once - the vectorised
    # stand-in for fitting each cell's plane in a Python loop. Only b and c
    # (the gradient) are used; the intercept a is never computed.
    det = (n * (sxx * syy - sxy * sxy)
           - sx * (sx * syy - sxy * sy)
           + sy * (sx * sxy - sxx * sy))
    det_b = (n * (sxz * syy - sxy * syz)
             - sz * (sx * syy - sxy * sy)
             + sy * (sx * syz - sxz * sy))
    det_c = (n * (sxx * syz - sxz * sxy)
             - sx * (sx * syz - sxz * sy)
             + sz * (sx * sxy - sxx * sy))

    with np.errstate(divide='ignore', invalid='ignore'):
        gx = np.where(det != 0.0, det_b / det, np.nan) / float(resolution)
        gy = np.where(det != 0.0, det_c / det, np.nan) / float(resolution)

    slope = np.arctan(np.hypot(gx, gy)).astype(np.float32)
    supported = n >= _SLOPE_FIT_MIN_SUPPORT
    return np.where(finite & supported, slope, np.nan).astype(np.float32)


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


def mask_floating_cells(elevation, gap_m: float) -> np.ndarray:
    """NaN out cells that hang in the air with no connection to the floor.

    A cell's height is the 20th percentile of its returns (elevation_grid),
    so a cell containing any real ground reads near that ground, and a rock
    or wall face - whose points run down to its base - reads near its base
    too. A cell whose height floats more than `gap_m` ABOVE the median of
    its valid 8-neighbours can only be a return cloud with nothing beneath
    it: ZED noise blobs in mid-air (sun glare, night grain), which the
    operator confirmed do not exist as physical objects in this yard.
    Masked cells become NaN = unseen, not free: the ground under the blob
    genuinely was not measured. gap_m <= 0 disables. Returns a copy.
    """
    e = np.asarray(elevation, dtype=np.float32).copy()
    if gap_m <= 0.0 or e.ndim != 2:
        return e
    padded = np.pad(e, 1, mode='constant', constant_values=np.nan)
    rows, cols = e.shape
    neighbours = np.stack([
        padded[1 + dy:1 + dy + rows, 1 + dx:1 + dx + cols]
        for dy in (-1, 0, 1) for dx in (-1, 0, 1) if (dy, dx) != (0, 0)])
    with np.errstate(invalid='ignore'), warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)   # all-NaN slices
        floor = np.nanmedian(neighbours, axis=0)
    floating = (e - floor) > float(gap_m)      # NaN comparisons are False
    e[floating] = np.nan
    return e


def derive(elevation, resolution: float = RESOLUTION,
          fit_radius_m: float = SLOPE_FIT_RADIUS_M) -> dict:
    """The four layers, in one pass over the window.

    'slope' is slope_layer_fitted, not the raw two-cell gradient - this is
    the layer both the cost and the published GridMap use, because the
    operator reads that layer to understand why ground was refused, and it
    must be the slope the refusal was actually based on. The raw gradient
    stays available by calling slope_layer directly, for debugging the fit
    itself against the noise it exists to average out.

    Measured on the laptop, 2026-09-01, at the full 960 x 960: step 25 ms,
    slope (fitted, 0.2 m radius) ~290 ms, roughness 25 ms, valid 3 ms -
    about 300 ms, against a 1 Hz tick. That is roughly 3x the old raw
    two-cell gradient's ~110 ms total, and it is the honest cost of the
    fit: nine windowed sums over a 9x9 window in float64, because the
    normal equations' cross terms are exactly what cancels catastrophically
    in float32 at grid-index magnitude. The Orin's cores are 2-3x slower
    single-thread; SP9/SP10 measure it there (spec section 5, section 11
    risk 6), SP12 re-measures it in the yard, and the documented fallback
    is spec section 5's 24 m window - now joined by a second one if the
    Orin needs it: narrowing slope_fit_radius_m shrinks the window, but
    the summed-area table's cost does not scale with it, so the fallback
    that actually helps is the same 24 m window the raw gradient always
    had."""
    grid = _as_grid(elevation)
    return {
        'slope': slope_layer_fitted(grid, resolution, fit_radius_m),
        'step': step_layer(grid),
        'roughness': roughness_layer(grid),
        'valid': valid_layer(grid),
    }


def ground_under(elevation, cell, radius_cells: int):
    """The height of the ground the rover is standing on, read from the
    map itself: the median of the seen cells within `radius_cells` of
    `cell` - the footprint. None when nothing under the rover has been
    mapped yet.

    This exists because the pose's own z is the wrong reference for the
    rover-relative test. The ZED's z drifts - 0.8 m in one measured night -
    and it drifts against the very grid the test judges, so a drifted pose
    makes level ground read as a climb and the rover walls itself in and
    refuses to move ("glitches under the ground", in the operator's words,
    which is exactly what it looks like). The map's height under the rover
    cannot disagree with the map: wherever the frame drifts, it drifts with
    both.

    The median, deliberately not the maximum: the wheels stand on the
    common ground, and a rover straddling a rock is standing beside it,
    not on it - a maximum would lift the reference onto every rock and
    noise spike under the belly, and everything else would then read as a
    lethal drop.
    """
    grid = np.asarray(elevation)
    rows, cols = grid.shape
    row, col = int(cell[0]), int(cell[1])
    offsets = _disc_offsets(radius_cells)
    ry = offsets[:, 0] + row
    rx = offsets[:, 1] + col
    in_bounds = (ry >= 0) & (ry < rows) & (rx >= 0) & (rx < cols)
    values = grid[ry[in_bounds], rx[in_bounds]]
    values = values[np.isfinite(values)]
    if values.size == 0:
        return None
    return float(np.median(values))


def height_relative_to(elevation, rover_z: float) -> np.ndarray:
    """Each cell's height above (positive) or below (negative) the ground
    the rover is standing on. NaN stays NaN.

    Plain subtraction already has that property: an unseen cell is NaN in
    `elevation`, and NaN minus anything is NaN, so there is nothing here that
    needs to special-case it. What keeps this meaningful is the caller
    restricting it to cells near the rover - see costmap_seed."""
    grid = _as_grid(elevation)
    return (grid - np.float32(rover_z)).astype(np.float32)


# nav_msgs/OccupancyGrid conventions: 0..100 cost, -1 unknown. 100 is
# reserved for lethal, so the scaled band tops out at 99 and a planner can
# tell "as bad as it gets while still driveable" from "do not".
LETHAL = 100
UNKNOWN = -1
MAX_SCALED_COST = 99


def costmap_seed(slope, step, roughness, valid,
                 step_lethal_m: float = STEP_LETHAL_M,
                 slope_lethal_rad: float = SLOPE_LETHAL_RAD,
                 elevation=None,
                 resolution: float = RESOLUTION,
                 rover_z=None,
                 rover_cell=None,
                 relative_radius_m: float = RELATIVE_RADIUS_M,
                 climb_lethal_m: float = CLIMB_LETHAL_M,
                 drop_lethal_m: float = DROP_LETHAL_M) -> np.ndarray:
    """The four layers as one int8 cost grid for /autonomy/costmap_seed.

        s = clip(slope     / SLOPE_LETHAL_RAD, 0, 1)
        t = clip(step      / STEP_LETHAL_M,    0, 1)
        r = clip(roughness / ROUGHNESS_REF_M,  0, 1)
        cost = round(99 * max(s, t, r))
        cost = -1   where valid == 0
        cost = -1   where slope is NaN (too few cells to fit a plane)
        cost = 100  where step >= STEP_LETHAL_M or slope >= SLOPE_LETHAL_RAD

    `max`, not a mean: one bad indicator must not be averaged away by two
    good ones. The order of the last two lines is deliberate - a cell with a
    measured lethal step is lethal even where its neighbourhood is too
    incomplete for `valid` or for the slope fit, because that is the safe
    direction and because it is exactly what a hole's frontier looks like.
    An unsupported slope fit (fewer than 6 finite cells in its window) is
    unknown for the same reason an unseen cell is: the fit has nothing to
    say, and nothing to say is not the same as flat.

    `rover_z` and `rover_cell`, given together with `elevation`, turn on a
    second and independent lethal test: a cell also goes lethal when it sits
    more than `climb_lethal_m` above the rover's own ground, or more than
    `drop_lethal_m` below it (see `height_relative_to`). This is what
    catches a rise `step` cannot: a staircase of small steps, each one under
    STEP_LETHAL_M, that nonetheless climbs more than the rover can mount
    between where it stands and the cell in question.

    This test is masked to `relative_radius_m` of `rover_cell` and
    contributes NOTHING outside it - not a cost, not a lethal. That radius is
    not a tuning knob, it is the whole safeguard: applied to the entire map,
    a gentle 5 degree yard slope rises 0.87 m over 10 m, so every cell more
    than about 3 m uphill would read as more than CLIMB_LETHAL_M above the
    rover and the rover would wall itself into a small disc of the world and
    refuse to plan anywhere. Extended terrain is exactly what `slope_layer`
    is for, and it already handles it correctly (lethal above
    SLOPE_LETHAL_RAD). The rover-relative test answers a narrower question -
    "can I mount the thing immediately in front of me" - which is only
    meaningful nearby: far away the rover will have climbed or descended
    before it arrives, and its own reference ground (`rover_z`) will have
    moved with it. Disabled whenever `elevation`, `rover_z` or `rover_cell`
    is None, or when `relative_radius_m` <= 0.
    """
    slope = np.asarray(slope, dtype=np.float32)
    # A NaN here now means two different things: the cell itself is
    # unseen (valid already catches that), or the fitted plane had fewer
    # than 6 finite cells to lean on - a real case at the mapped frontier
    # that the 4-neighbour `valid` test does not see, because 6 cells
    # across a 0.2 m radius is a wider ask than 4 immediate neighbours.
    # Captured before the NaN is zeroed away, so it can still mark the
    # cell unknown rather than silently reading as flat.
    slope_unsupported = ~np.isfinite(slope)
    slope = np.nan_to_num(slope, nan=0.0)
    step = np.nan_to_num(np.asarray(step, dtype=np.float32), nan=0.0)
    roughness = np.nan_to_num(np.asarray(roughness, dtype=np.float32), nan=0.0)
    valid = np.asarray(valid, dtype=np.float32)

    # step_lethal_m defaults to the spec's 0.14; the layer node exposes it
    # as a parameter because night sessions showed ZED depth noise in the
    # dark fabricating >0.14 m phantom steps - lethal cells on ground the
    # operator can see is flat. Raising it at night trades hole-margin for
    # not walling off the whole yard; the spec value stays the default.
    worst = np.maximum(
        np.maximum(np.clip(slope / float(slope_lethal_rad), 0.0, 1.0),
                   np.clip(step / float(step_lethal_m), 0.0, 1.0)),
        np.clip(roughness / ROUGHNESS_REF_M, 0.0, 1.0))
    cost = np.rint(MAX_SCALED_COST * worst).astype(np.int8)
    cost[valid < 0.5] = UNKNOWN
    cost[slope_unsupported] = UNKNOWN
    cost[(step >= float(step_lethal_m))
         | (slope >= float(slope_lethal_rad))] = LETHAL

    if (elevation is not None and rover_z is not None and rover_cell is not None
            and relative_radius_m > 0.0):
        # Confined to a disc around the rover, on purpose - see the
        # docstring above. _disc_offsets is the same Euclidean-disc helper
        # the startup patch and the wheel trail use, so "near the rover"
        # means the same thing everywhere in this module.
        relative = height_relative_to(elevation, rover_z)
        rows, cols = relative.shape
        row, col = rover_cell
        radius_cells = int(round(float(relative_radius_m) / float(resolution)))
        offsets = _disc_offsets(radius_cells)
        ry = offsets[:, 0] + int(row)
        rx = offsets[:, 1] + int(col)
        in_bounds = (ry >= 0) & (ry < rows) & (rx >= 0) & (rx < cols)
        ry = ry[in_bounds]
        rx = rx[in_bounds]
        local = relative[ry, rx]
        # NaN (unseen) local cells are left exactly as `valid` already put
        # them - this test only ever adds a lethal, never a guess.
        bad = np.isfinite(local) & ((local > float(climb_lethal_m))
                                    | (local < -float(drop_lethal_m)))
        cost[ry[bad], rx[bad]] = LETHAL

    return cost


@functools.lru_cache(maxsize=8)
def _disc_offsets(radius_cells: int) -> np.ndarray:
    """(dy, dx) integer offsets of every cell within `radius_cells` of (0, 0),
    on a Euclidean disc.

    Cached: the rover-relative test rebuilds a 3 m disc (a 121 x 121 mgrid)
    on every 1 Hz map tick otherwise, for a radius that only changes when an
    operator retunes it. The cache hands back the SAME array each time, so
    callers must treat it as read-only - every current caller derives new
    arrays from it and writes to none of it."""
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


def heal_goal_patch(cost: np.ndarray, centre_cell, radius_cells: int) -> np.ndarray:
    """The operator says a goal is reachable, so the map does not get to
    veto it: every cell within `radius_cells` of the goal becomes free.

    Unlike clear_startup_patch this clears MEASURED cost, LETHAL included -
    which is the whole point. A goal typed from the judges' site plan sat on
    ground the elevation layer called an obstacle ("the target seemed to be
    in the ground"), and both planners refused a goal inside a wall, so the
    run ended before it began.

    The trade is real and deliberate: an obstacle that genuinely stands at
    the goal is erased here too, and the rover will drive into it. The
    radius is the whole safeguard - it must stay small enough that the
    operator can see what they are clearing when they place the waypoint,
    and it clears one disc around one commanded point, never a corridor.

    `centre_cell` is (row, col) in this tick's grid, or None to heal
    nothing. Mutates `cost` in place and returns it.
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
    cost[ry[in_bounds], rx[in_bounds]] = 0
    return cost


def seed_from_elevation(elevation, resolution: float = RESOLUTION,
                        step_lethal_m: float = STEP_LETHAL_M,
                        floating_gap_m: float = 0.0,
                        slope_lethal_rad: float = SLOPE_LETHAL_RAD,
                        slope_fit_radius_m: float = SLOPE_FIT_RADIUS_M,
                        rover_z=None,
                        rover_cell=None,
                        relative_radius_m: float = RELATIVE_RADIUS_M,
                        climb_lethal_m: float = CLIMB_LETHAL_M,
                        drop_lethal_m: float = DROP_LETHAL_M) -> tuple:
    """(the four layers, the int8 cost grid) - the whole derive in one call.
    floating_gap_m > 0 first drops cells hanging in the air with no
    connection to the floor (see mask_floating_cells).

    slope_fit_radius_m is the neighbourhood the cost's slope is fitted
    over - see slope_layer_fitted. rover_z and rover_cell, given together,
    turn on the rover-relative lethal test within relative_radius_m of
    rover_cell - see costmap_seed's docstring for what it catches and why
    it must not run past that radius. rover_z is None (the default)
    disables it entirely."""
    if floating_gap_m > 0.0:
        elevation = mask_floating_cells(elevation, floating_gap_m)
    layers = derive(elevation, resolution, slope_fit_radius_m)
    cost = costmap_seed(layers['slope'], layers['step'],
                        layers['roughness'], layers['valid'],
                        step_lethal_m=step_lethal_m,
                        slope_lethal_rad=slope_lethal_rad,
                        elevation=elevation,
                        resolution=resolution,
                        rover_z=rover_z,
                        rover_cell=rover_cell,
                        relative_radius_m=relative_radius_m,
                        climb_lethal_m=climb_lethal_m,
                        drop_lethal_m=drop_lethal_m)
    return layers, cost
