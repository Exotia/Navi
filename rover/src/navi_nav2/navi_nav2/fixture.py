"""The elevation window the offline planning test plans over.

Spec section 9 rung 3 says "a recorded elevation map".  There is none: no
bag, no npz, no camera on the rover the night this was written.  So the
fixture is *generated* - deterministically, out of SP7's own modules, so
that what is tested is the real cost curve and the real hole detection and
not a hand-written cost grid that agrees with itself.

It is code rather than a committed array on purpose: 960 x 960 float32 is
3.7 MB and its seed is 0.92 MB, and neither belongs in a repository whose
rover side is entirely text.  The constants below pin it instead - the
structure of the int8 seed, not a hash of the float32 elevation, because
numpy's transcendental kernels are not bit-identical across architectures
and this suite runs on the Orin as well as the laptop.

The terrain, in the map frame, 48 m centred on the origin:

    y
    ^         (6, 6.0) boulder r 0.9 h 0.60
    |                                    wall x = 9, y in [-8, 1.5]
    |   START (0,0) ---- pit (6,0) r 1.2 d 0.50 ---- GOAL (12,0)
    |         (6,-6.0) boulder r 0.9 h 0.60
    +---------------------------------------------> x

The pit sits on the straight line from start to goal, so a planner that
never received the seed cannot produce a passing path.  The wall, which
reaches from the southern edge to y = 1.5, closes the southern detour and
forces the way round to be the northern one.  The two boulders narrow that
corridor without closing it: beside the pit the clear band for the rover's
centre runs from y = 2.05 (pit rim plus the 0.85 m circle) to y = 4.25
(boulder minus the same), 2.2 m of it.  The problem is hard, not
impossible - a planner that cannot solve it is broken, and that is the
whole point of choosing the numbers this way.

When SP12 records a real yard bag, elevation_from_npz() replaces
elevation() and every assertion in test_offline_planning.py stands.
"""

import numpy as np
from nav_msgs.msg import OccupancyGrid

from navi_autonomy.grid_map_io import build_occupancy_grid
from navi_autonomy.traversability import seed_from_elevation
from navi_autonomy.window import WINDOW_CELLS
from navi_localization.elevation_grid import RESOLUTION

# The window is centred on the map origin: cell (0, 0) is the corner at
# (-24, -24), which is what info.origin carries.
ORIGIN_IX = -WINDOW_CELLS // 2
ORIGIN_IY = -WINDOW_CELLS // 2

START = (0.0, 0.0)
GOAL = (12.0, 0.0)
PIT_CENTRE = (6.0, 0.0)
PIT_RADIUS_M = 1.2
PIT_DEPTH_M = 0.50
BOULDERS = (((6.0, 6.0), 0.9, 0.60), ((6.0, -6.0), 0.9, 0.60))
WALL_X = 9.0
WALL_Y = (-8.0, 1.5)
WALL_HALF_THICKNESS_M = 0.15
WALL_HEIGHT_M = 0.50

# Gentle ground, well under the 25 degree slope and 0.14 m step thresholds:
# a 0.02 m amplitude over a 6 m wavelength is a 1.2 degree slope, plus
# 0.004 m of seeded noise.  Flat-as-a-table terrain would let a broken
# traversability layer pass.
GROUND_AMPLITUDE_M = 0.02
GROUND_WAVELENGTH_M = 6.0
NOISE_M = 0.004
NOISE_SEED = 20260831

# What the fixture is pinned by.  NOT a hash of the float32 elevation: numpy's
# sin/cos are SIMD-dispatched and are not bit-identical between amd64 and
# arm64, a 1-ULP float64 difference survives the float32 cast on a rounding
# tie, and deploy_rover.sh --test runs this suite on the Orin.  What matters
# is the int8 seed the planner actually consumes, so that is what is pinned -
# structurally.  (The default_rng(20260831).normal half *is* stream-stable by
# NumPy's compatibility policy; the trig half is not.)
LETHAL_CELLS = 1745         # measured on the laptop, 2026-08-31 (see test-5-report)
LETHAL_CELLS_REL_TOLERANCE = 1e-3   # a threshold-straddling cell may flip
PIT_CELL = (600, 480)       # cell_of(*PIT_CENTRE), exact by construction
ELEVATION_MIN_M = -0.50     # the pit floor
ELEVATION_MAX_M = 0.60      # the boulder tops
# The ground wave (+-0.02 m) and the seeded noise (~4 sigma of 0.004 m) ride
# on top of both extremes, so the extremes are pinned to a band, not a value.
ELEVATION_EXTREME_TOLERANCE_M = 0.05


def _axes():
    """Cell-centre coordinates, storage convention: row 0 is the smallest y,
    column 0 the smallest x, both ascending (elevation_grid.py)."""
    xs = (np.arange(WINDOW_CELLS, dtype=np.float64) + ORIGIN_IX + 0.5) * RESOLUTION
    ys = (np.arange(WINDOW_CELLS, dtype=np.float64) + ORIGIN_IY + 0.5) * RESOLUTION
    return np.meshgrid(xs, ys)


def cell_of(x: float, y: float) -> tuple:
    """(column, row) of the cell containing the point - the same lattice
    build_occupancy_grid writes."""
    return (int(np.floor(x / RESOLUTION)) - ORIGIN_IX,
            int(np.floor(y / RESOLUTION)) - ORIGIN_IY)


def elevation() -> np.ndarray:
    x, y = _axes()
    z = GROUND_AMPLITUDE_M * np.sin(2 * np.pi * x / GROUND_WAVELENGTH_M) \
        * np.cos(2 * np.pi * y / GROUND_WAVELENGTH_M)

    rng = np.random.default_rng(NOISE_SEED)
    z = z + rng.normal(0.0, NOISE_M, size=z.shape)

    pit = np.hypot(x - PIT_CENTRE[0], y - PIT_CENTRE[1]) <= PIT_RADIUS_M
    z[pit] -= PIT_DEPTH_M

    for (cx, cy), radius, height in BOULDERS:
        z[np.hypot(x - cx, y - cy) <= radius] += height

    wall = ((np.abs(x - WALL_X) <= WALL_HALF_THICKNESS_M)
            & (y >= WALL_Y[0]) & (y <= WALL_Y[1]))
    z[wall] += WALL_HEIGHT_M

    return z.astype(np.float32)


def elevation_from_npz(path: str) -> np.ndarray:
    """A recorded window, when SP12 has one.  Same shape, same convention."""
    grid = np.load(path)['elevation'].astype(np.float32)
    if grid.shape != (WINDOW_CELLS, WINDOW_CELLS):
        raise ValueError(f"expected a {WINDOW_CELLS}^2 window, got {grid.shape}")
    return grid


def seed(grid=None) -> np.ndarray:
    """The int8 cost grid, straight through SP7's own cost curve."""
    _, cost = seed_from_elevation(elevation() if grid is None else grid, RESOLUTION)
    return cost


def occupancy_grid(stamp, grid=None) -> OccupancyGrid:
    from builtin_interfaces.msg import Time
    return build_occupancy_grid(seed(grid), ORIGIN_IX, ORIGIN_IY, RESOLUTION,
                                'map', stamp if stamp is not None else Time())


def clearance_cells(cost, x: float, y: float, radius_m: float) -> int:
    """How many lethal cells lie within radius_m of (x, y).

    Zero is the assertion the planning test makes at every point of the
    path: the rover's inscribed circle never touches a lethal cell.
    """
    from navi_autonomy.traversability import LETHAL
    ix, iy = cell_of(x, y)
    reach = int(np.ceil(radius_m / RESOLUTION))
    y0, y1 = max(iy - reach, 0), min(iy + reach + 1, cost.shape[0])
    x0, x1 = max(ix - reach, 0), min(ix + reach + 1, cost.shape[1])
    patch = cost[y0:y1, x0:x1]
    rows = (np.arange(y0, y1) - iy)[:, None]
    cols = (np.arange(x0, x1) - ix)[None, :]
    inside = (rows ** 2 + cols ** 2) <= reach ** 2
    return int(((patch == LETHAL) & inside).sum())
