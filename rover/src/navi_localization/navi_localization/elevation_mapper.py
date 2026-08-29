"""Publishes the map as tiles, and answers save/load/clear commands.

Input is the ZED SDK's fused point cloud, which the wrapper publishes in the
map frame - the same frame and the same tracking as /localization/pose, so
the map and the pose cannot drift apart relative to each other. Output is a
stream of grid_map_msgs/GridMap tiles, one `elevation` layer each, on
/localization/map_tile, plus JSON status on /localization/map_status and
JSON commands accepted on /localization/map_command.

The cloud is read with numpy rather than sensor_msgs_py.point_cloud2:
ZedCamera::callback_pubFusedPc lays every point out as four float32s
(x, y, z, rgb; zed_camera_component.cpp:9550), so the payload is a float32
matrix and a per-point Python loop over ~10^5 points would cost more than
everything else in this node put together. Anything that is not that layout
raises instead of being silently misread.

The wrapper only extracts the fused cloud when the topic has a subscriber
(the count_subscribers guard at the top of that callback), so this node
running is what makes the SDK do the mapping work at all. When it is not
running, the topic is silent and the GPU is not spending anything on it -
that is by design, not a fault.

Clamped to the rover, not to the world: this node also subscribes
/localization/pose (localization_status's map -> base_footprint Odometry)
and, once a pose has arrived, clips every published cell's elevation to
[z_rover - clamp_below, z_rover + clamp_above] before cutting it into
tiles. That is what keeps a wall or an overhanging branch from drawing as
a spike or a cliff in the Gazebo terrain right under the rover, without
touching the grid itself - the clamp is applied to a copy, so a saved map
still holds true heights, and `top` (the cell's true max, for a later
obstacle layer) is never clamped. Before the first pose: no clamp at all,
since there is no rover height yet to clamp relative to.

Measured numbers: see launch/localization.launch.py.
"""

import dataclasses
import json
from collections import deque
from datetime import datetime, timezone

import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String

from navi_localization.elevation_grid import RESOLUTION, ElevationGrid
from navi_localization.map_store import DEFAULT_DIRECTORY, MapStore
from navi_localization.tiles import (
    TILE_SAMPLES, TileScheduler, tile_center, tiles_of_snapshot)

FUSED_CLOUD_TOPIC = '/zed_front/zed_node/mapping/fused_cloud'
POSE_TOPIC = '/localization/pose'
MAP_TILE_TOPIC = '/localization/map_tile'
MAP_COMMAND_TOPIC = '/localization/map_command'
MAP_STATUS_TOPIC = '/localization/map_status'
LAYER = 'elevation'

# Depth of the tile writer, and how many blanking tiles a tick may send.
#
# A tick emits at most nine tiles (eight dirty plus one keepalive), but a
# clear or a load has to tell the sim about every tile that is no longer
# there, and that is one message per previously published tile - about 196
# over the yard and 576 at the 60 m cap. Bursting those into a depth-8
# RELIABLE KEEP_LAST writer (and through sim_bridge, whose queues are the
# same size) overruns it, and the tiles that fall out are exactly the ones
# that would have removed stale terrain, so the sim keeps ground the rover
# no longer believes in. Two changes together: a deeper writer, and the
# burst paced over ticks rather than sent in one go from the callback.
TILE_QUEUE_DEPTH = 64
NAN_TILES_PER_TICK = 16


def points_from_cloud(message: PointCloud2) -> np.ndarray:
    """(N, 3) float64 of the x/y/z columns of a float32 PointCloud2."""
    names = [field.name for field in message.fields]
    offsets = [field.offset for field in message.fields]
    datatypes = [field.datatype for field in message.fields]
    if names[:3] != ['x', 'y', 'z'] or offsets[:3] != [0, 4, 8]:
        raise ValueError(
            f"fused cloud has an unexpected layout: fields={names} "
            f"offsets={offsets}. This node reads x/y/z as the first three "
            f"float32s of every point and will not guess at anything else.")
    if datatypes[:3] != [PointField.FLOAT32] * 3:
        raise ValueError(
            f"fused cloud's x/y/z fields are not float32: datatypes={datatypes[:3]}")
    if message.is_bigendian:
        raise ValueError("fused cloud is big-endian; this node reads little-endian")
    if message.point_step % 4:
        raise ValueError(f"point_step {message.point_step} is not a whole number of float32s")

    # bytes() copies, which at 0.5 Hz and a couple of MB is free and removes
    # every alignment question np.ndarray.view would raise.
    floats = np.frombuffer(bytes(message.data), dtype=np.float32)
    stride = message.point_step // 4
    return floats.reshape(-1, stride)[:, :3].astype(np.float64)


def build_tile_message(key, tile: np.ndarray, frame_id: str, stamp) -> GridMap:
    """One tile as a GridMap, in grid_map's own index order.

    grid_map's convention: index (0, 0) is the sample at the *maximum* x
    and maximum y, rows run in -x, columns in -y, data column-major. The
    tile array is the opposite (row 0 at minimum y, column 0 at minimum
    x), so both axes are reversed and the result transposed.
    """
    grid = np.asarray(tile, dtype=np.float32)[::-1, ::-1].T
    n_rows, n_cols = grid.shape
    layer = Float32MultiArray()
    layer.layout.dim = [
        MultiArrayDimension(label='column_index', size=n_cols, stride=n_rows * n_cols),
        MultiArrayDimension(label='row_index', size=n_rows, stride=n_rows),
    ]
    layer.layout.data_offset = 0
    layer.data = grid.flatten(order='F').tolist()

    center_x, center_y = tile_center(*key)
    message = GridMap()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    message.info.resolution = float(RESOLUTION)
    message.info.length_x = float(n_rows * RESOLUTION)
    message.info.length_y = float(n_cols * RESOLUTION)
    message.info.pose.position.x = float(center_x)
    message.info.pose.position.y = float(center_y)
    message.info.pose.position.z = 0.0
    message.info.pose.orientation.w = 1.0
    message.layers = [LAYER]
    message.basic_layers = [LAYER]
    message.data = [layer]
    message.outer_start_index = 0
    message.inner_start_index = 0
    return message


class ElevationMapper(Node):

    def __init__(self, map_directory: str = None) -> None:
        super().__init__('elevation_mapper')
        self.declare_parameter('cloud_topic', FUSED_CLOUD_TOPIC)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('tick_seconds', 1.0)
        self.declare_parameter('map_directory', map_directory or DEFAULT_DIRECTORY)
        # <= 0 disables that side of the clamp. Defaults are the spec's
        # numbers: a rover-height window that keeps a wall or an
        # overhanging branch out of the drawn terrain without clamping the
        # ground under a ramp or a curb the rover can actually climb.
        self.declare_parameter('clamp_above', 0.5)
        self.declare_parameter('clamp_below', 1.0)

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._grid = ElevationGrid()
        self._scheduler = TileScheduler()
        self._store = MapStore(str(self.get_parameter('map_directory').value))
        self._loaded = None
        self._last_command = None
        self._warned_about_cap = False
        self._tile_count = 0
        self._clamp_above = float(self.get_parameter('clamp_above').value)
        self._clamp_below = float(self.get_parameter('clamp_below').value)
        # None until /localization/pose's first message: no rover height to
        # clamp relative to yet, so _offer applies no clamp at all.
        self._rover_z = None
        # Tiles a clear or a load has orphaned, waiting to go out as
        # all-NaN, `NAN_TILES_PER_TICK` a tick. The set is the queue's
        # membership: a key is dropped from it (not from the deque) when
        # the tile comes back to life before its blank was sent, so a
        # freshly mapped tile is never blanked behind the sim's back.
        self._pending_nan = deque()
        self._pending_nan_keys = set()

        # Default QoS: reliable, volatile, depth 1 - sim_bridge relays this
        # topic with a generic subscription and a durability mismatch there
        # would mean no data at all. Late joiners get the keepalive.
        self._tile_publisher = self.create_publisher(
            GridMap, MAP_TILE_TOPIC, TILE_QUEUE_DEPTH)
        self._status_publisher = self.create_publisher(String, MAP_STATUS_TOPIC, 1)
        self.create_subscription(
            PointCloud2, str(self.get_parameter('cloud_topic').value), self._on_cloud, 1)
        self.create_subscription(Odometry, POSE_TOPIC, self._on_pose, 1)
        self.create_subscription(String, MAP_COMMAND_TOPIC, self._on_command, 4)
        self.create_timer(float(self.get_parameter('tick_seconds').value), self._tick)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f"mapping {self.get_parameter('cloud_topic').value} into {MAP_TILE_TOPIC} "
            f"({RESOLUTION} m cells, 2.5 m tiles); maps under {self._store.directory}")

    # -- mapping ----------------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_cloud(self, message: PointCloud2) -> None:
        try:
            points = points_from_cloud(message)
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        self._grid.update(points)
        if self._grid.points_outside_cap and not self._warned_about_cap:
            self._warned_about_cap = True
            self.get_logger().warn(
                f"{self._grid.points_outside_cap} points fell outside the "
                "60 x 60 m map and were dropped; the map stays where it "
                "started rather than sliding and forgetting ground")
        self._offer(self._now())

    def _on_pose(self, message: Odometry) -> None:
        # Only remembers the rover's height. It does not re-offer: cutting
        # the whole grid into tiles costs ~30 ms at the 60 m cap, and pose
        # arrives at 15 Hz while the cloud (the thing that actually changes
        # what there is to publish) arrives at ~1 Hz. The next _offer -
        # from the next cloud, or from a load - uses whatever z is current
        # by then, so the clamp is never more than one cloud tick stale.
        self._rover_z = float(message.pose.pose.position.z)

    def _clamped_for_tiles(self, snapshot):
        """`snapshot`, or a copy clamped to the rover's height for
        publishing. The grid itself is never touched - a saved map must
        keep true heights - and `top` is never clamped either way, since
        it is not published yet."""
        if self._rover_z is None:
            return snapshot
        if self._clamp_above <= 0 and self._clamp_below <= 0:
            return snapshot
        low = -np.inf if self._clamp_below <= 0 else self._rover_z - self._clamp_below
        high = np.inf if self._clamp_above <= 0 else self._rover_z + self._clamp_above
        elevation = np.clip(snapshot.elevation, low, high)
        return dataclasses.replace(snapshot, elevation=elevation)

    def _offer(self, now: float) -> None:
        snapshot = self._grid.snapshot()
        if snapshot is None:
            self._tile_count = 0
            return
        keys = tiles_of_snapshot(self._clamped_for_tiles(snapshot))
        # Remembered rather than recomputed in _publish_status: cutting the
        # snapshot into tiles costs ~32 ms over a full 60 m map on the
        # Jetson, and the status timer runs once a second whether or not
        # anything changed. Here it is computed anyway.
        self._tile_count = len(keys)
        for key in keys:
            # Live again before its blank went out: sending the blank now
            # would erase terrain the map does believe in.
            self._pending_nan_keys.discard(key)
        self._scheduler.offer(keys, now)

    def _queue_nan(self, keys) -> None:
        for key in keys:
            if key not in self._pending_nan_keys:
                self._pending_nan_keys.add(key)
                self._pending_nan.append(key)

    def _tick(self, now: float = None) -> None:
        now = self._now() if now is None else now
        stamp = self.get_clock().now().to_msg()
        empty = np.full((TILE_SAMPLES, TILE_SAMPLES), np.nan, dtype=np.float32)
        sent = 0
        while self._pending_nan and sent < NAN_TILES_PER_TICK:
            key = self._pending_nan.popleft()
            if key not in self._pending_nan_keys:
                continue                # came back to life; costs no budget
            self._pending_nan_keys.discard(key)
            self._tile_publisher.publish(
                build_tile_message(key, empty, self._frame_id, stamp))
            sent += 1
        for key, tile in self._scheduler.due(now):
            self._tile_publisher.publish(build_tile_message(key, tile, self._frame_id, stamp))
            self._scheduler.published(key, tile, now)

    # -- commands ---------------------------------------------------------

    def _on_command(self, message: String) -> None:
        command = {}
        try:
            command = json.loads(message.data)
            if not isinstance(command, dict):
                raise ValueError("command is not a JSON object")
            action = str(command.get('action', ''))
            name = command.get('name')
            if action == 'save':
                self._save(name, bool(command.get('overwrite', False)))
            elif action == 'load':
                self._load(name)
            elif action == 'clear':
                self._clear()
            else:
                raise ValueError(f"unknown action {action!r}; save, load or clear")
            self._record(action, name, None)
        except Exception as error:                          # noqa: BLE001
            # Deliberately everything. This is a subscription callback: an
            # exception that leaves it comes out of rclpy.spin and ends
            # mapping for the whole run, so a save onto a full disk (OSError)
            # or a load of a map file this build cannot parse would cost the
            # operator the live map rather than one refused command. The
            # spec's rule is that every command outcome is reported, and
            # "reported" includes the ones nobody predicted.
            self.get_logger().error(f"map command refused: {error}")
            self._record(command.get('action') if isinstance(command, dict) else None,
                         command.get('name') if isinstance(command, dict) else None,
                         str(error))

    def _record(self, action, name, error) -> None:
        self._last_command = {
            'action': action, 'name': name, 'ok': error is None, 'error': error,
            'at': datetime.now(timezone.utc).isoformat(timespec='seconds')}

    def _save(self, name, overwrite: bool) -> None:
        state = self._grid.state()
        if state is None:
            raise ValueError("nothing to save: the map is empty")
        path = self._store.save(name, state, overwrite=overwrite)
        self.get_logger().info(f"map saved to {path}")

    def _load(self, name) -> None:
        state = self._store.load(name)
        self._grid.replace(state)
        self._loaded = name
        # A load replaces the grid outright, so the scheduler's memory of
        # the previous grid is dropped too: without this, a tile the live
        # map had touched but the loaded map does not cover would stay in
        # _latest/_round_robin and be marked dirty below, republishing a
        # tile that no longer belongs to the map at all.
        old_keys = self._scheduler.forget_all()
        self._offer(self._now())
        new_snapshot = self._grid.snapshot()
        new_keys = set(tiles_of_snapshot(new_snapshot)) if new_snapshot is not None else set()
        # Any tile that was on screen before the load but has no seen cell
        # in the loaded map would otherwise never be mentioned again, and
        # the sim would keep its stale terrain forever - so every such tile
        # goes out once more as all-NaN, exactly as _clear does, paced by
        # _tick rather than burst out of this callback.
        self._queue_nan(key for key in old_keys if key not in new_keys)
        self._scheduler.mark_all_dirty()
        self.get_logger().info(f"map {name!r} loaded; republishing every tile")

    def _clear(self) -> None:
        self._grid.clear()
        self._loaded = None
        self._tile_count = 0
        self._queue_nan(self._scheduler.forget_all())
        self.get_logger().info("map cleared")

    # -- status -----------------------------------------------------------

    def _publish_status(self) -> None:
        snapshot = self._grid.snapshot()
        if snapshot is None:
            cells, extent = 0, [0.0, 0.0]
        else:
            seen = np.isfinite(snapshot.elevation)
            cells = int(seen.sum())
            rows, cols = snapshot.elevation.shape
            extent = [round(cols * RESOLUTION, 2), round(rows * RESOLUTION, 2)]
        self._status_publisher.publish(String(data=json.dumps({
            'resolution': RESOLUTION, 'cells_seen': cells, 'extent_m': extent,
            'tiles': self._tile_count, 'loaded': self._loaded,
            'maps': self._store.list_names(),
            'last_command': self._last_command})))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ElevationMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
