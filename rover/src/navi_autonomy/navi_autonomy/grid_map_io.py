"""grid_map_msgs/GridMap and nav_msgs/OccupancyGrid, to and from numpy.

The one place in this package that knows grid_map's index convention: index
(0, 0) at the **largest** x and y, rows running in -x, columns in -y, data
column-major. Everything else here uses the repo's storage convention -
row 0 the smallest y, column 0 the smallest x - so the flip lives here and
nowhere else.

`layer_from_message` is adapted from
sim/src/navi_sim_bringup/navi_sim_bringup/terrain_writer.py's
`elevation_from_message`, which already reads exactly these messages for the
simulation; it is generalised to any layer name and paired with a writer,
rather than reinvented. That module could not simply be imported: it lives
in the laptop's colcon workspace, not the rover's.

Why array.array and not a list or a numpy array: the generated
Float32MultiArray setter has exactly one fast path, `array.array` with type
code 'f'; anything else goes through a per-element assert. Measured on this
laptop at 960 x 960 = 921,600 floats, 2026-08-30: array.array 3.5 ms per
layer, `ndarray.tolist()` 21 ms. Serialising the whole four-layer GridMap
then costs 65 ms, and the OccupancyGrid 1.9 ms.
"""

import array

import numpy as np
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from navi_localization.elevation_grid import RESOLUTION
from navi_localization.tiles import TILE_SAMPLES, tile_index_of

ELEVATION_LAYER = 'elevation'


def layer_from_message(message: GridMap, name: str) -> np.ndarray:
    """One layer in the storage convention: row 0 the smallest y, column 0
    the smallest x."""
    if name not in message.layers:
        raise ValueError(f"no {name!r} layer in {list(message.layers)}")
    if message.outer_start_index or message.inner_start_index:
        raise ValueError(
            "the grid_map circular-buffer start indices are not zero. This "
            "reader does not unroll them, and neither elevation_mapper nor "
            "grid_map_io ever sets them.")
    layer = message.data[message.layers.index(name)]
    n_cols = layer.layout.dim[0].size
    n_rows = layer.layout.dim[1].size
    grid = np.asarray(layer.data, dtype=np.float32).reshape(n_cols, n_rows).T
    return grid.T[::-1, ::-1]


def tile_from_message(message: GridMap) -> tuple:
    """(elevation (51, 51) float32, ix, iy) from one /localization/map_tile.

    The tile's identity is not in the message anywhere except its centre:
    unlike an obstacle tile, a map tile's header.frame_id is the plain map
    frame, so the index comes back through navi_localization.tiles'
    `tile_index_of`, which is the exact inverse of the `tile_center` the
    mapper wrote.

    Resolution is checked, never resampled: spec section 5 puts the costmap
    at 0.05 m precisely because "resampling smears the step edges that matter
    most", and a tile at another resolution means the mapper changed under us.
    """
    resolution = float(message.info.resolution)
    if abs(resolution - RESOLUTION) > 1e-9:
        raise ValueError(
            f"map tile resolution {resolution} is not {RESOLUTION}; this node "
            "does not resample - resampling smears the step edges that matter most")
    elevation = layer_from_message(message, ELEVATION_LAYER)
    if elevation.shape != (TILE_SAMPLES, TILE_SAMPLES):
        raise ValueError(
            f"a map tile is {TILE_SAMPLES}x{TILE_SAMPLES} samples, got {elevation.shape}")
    ix, iy = tile_index_of(float(message.info.pose.position.x),
                           float(message.info.pose.position.y))
    return elevation, ix, iy


def _layer_message(grid: np.ndarray) -> Float32MultiArray:
    """`grid` already flipped into grid_map's index order."""
    n_rows, n_cols = grid.shape
    layer = Float32MultiArray()
    layer.layout.dim = [
        MultiArrayDimension(label='column_index', size=n_cols, stride=n_rows * n_cols),
        MultiArrayDimension(label='row_index', size=n_rows, stride=n_rows),
    ]
    layer.layout.data_offset = 0
    buffer = array.array('f')
    buffer.frombytes(np.ascontiguousarray(grid, dtype=np.float32)
                     .flatten(order='F').tobytes())
    layer.data = buffer
    return layer


def build_grid_map(layers: dict, origin_ix: int, origin_iy: int, resolution: float,
                   frame_id: str, stamp) -> GridMap:
    """Several storage-convention layers as one GridMap.

    `origin_ix` / `origin_iy` are the lattice indices of column 0 and row 0;
    grid_map wants the map's *centre*, which is half a window further on.
    """
    names = list(layers)
    if not names:
        raise ValueError("a GridMap needs at least one layer")
    first = np.asarray(layers[names[0]], dtype=np.float32)
    n_y, n_x = first.shape
    message = GridMap()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    # GridMapInfo carries no header of its own in grid_map_msgs 2.0.1 -
    # checked with `ros2 interface show grid_map_msgs/msg/GridMapInfo`, which
    # is resolution, length_x, length_y, pose and nothing else.
    message.info.resolution = float(resolution)
    # grid_map's rows run in -x and its columns in -y, so its length_x is our
    # column count and its length_y our row count.
    message.info.length_x = float(n_x * resolution)
    message.info.length_y = float(n_y * resolution)
    message.info.pose.position.x = float((origin_ix + n_x / 2.0) * resolution)
    message.info.pose.position.y = float((origin_iy + n_y / 2.0) * resolution)
    message.info.pose.position.z = 0.0
    message.info.pose.orientation.w = 1.0
    message.layers = names
    message.basic_layers = [names[0]]
    message.data = [
        _layer_message(np.asarray(layers[name], dtype=np.float32)[::-1, ::-1].T)
        for name in names]
    message.outer_start_index = 0
    message.inner_start_index = 0
    return message


def build_occupancy_grid(cost: np.ndarray, origin_ix: int, origin_iy: int,
                         resolution: float, frame_id: str, stamp) -> OccupancyGrid:
    """A storage-convention int8 cost grid as an OccupancyGrid.

    No flip here: OccupancyGrid's data is row-major from the origin corner
    with x fastest and y ascending, which is exactly the storage convention.
    `info.origin` is the **corner** of cell (0, 0), not its centre.
    """
    cost = np.ascontiguousarray(cost, dtype=np.int8)
    if cost.ndim != 2:
        raise ValueError(f"a cost grid is 2-D, got shape {cost.shape}")
    n_y, n_x = cost.shape
    message = OccupancyGrid()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    message.info.map_load_time = stamp
    message.info.resolution = float(resolution)
    message.info.width = int(n_x)
    message.info.height = int(n_y)
    message.info.origin.position.x = float(origin_ix * resolution)
    message.info.origin.position.y = float(origin_iy * resolution)
    message.info.origin.position.z = 0.0
    message.info.origin.orientation.w = 1.0
    buffer = array.array('b')
    buffer.frombytes(cost.tobytes())
    message.data = buffer
    return message
