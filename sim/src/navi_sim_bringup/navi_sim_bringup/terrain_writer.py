#!/usr/bin/env python3
"""Shows the ground the rover has mapped as terrain in the Gazebo view.

One Gazebo model per 2.5 m tile. Gazebo Classic cannot change a model's
mesh in place, so a changed tile means spawning a new model and deleting
the old one - the replacement is spawned before the old one is deleted so
the ground never blinks. Per tile, at most one replacement a second; at
most 4 spawns in flight globally so a keepalive burst cannot stall Gazebo.

The terrain is a mesh, not a heightmap - see terrain_mesh.py for the
gzserver crash that decided it.

Nothing else in the world is touched. The rover model is never deleted, and
the world's ground plane at z = 0 stays, so the rover is never in the void.

Runs on the simulation's ROS domain. /localization/map_tile reaches that
domain from the rover's domain 0 through sim_bridge (sub-project 2); this
node knows nothing about domains and simply subscribes.

Each replacement gets its own mesh file name. Gazebo's MeshManager caches
meshes by file name, so respawning with the same name would put the
*previous* terrain back on the screen while every log line says the new
one was loaded. A new name every time sidesteps that entirely; the old
file is deleted once the new one is up.
"""

import os
import time

import numpy as np
import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from rclpy.executors import ExternalShutdownException
from grid_map_msgs.msg import GridMap
from rclpy.node import Node

from navi_sim_bringup.terrain_mesh import (
    GAZEBO_MODEL_NAME, model_config_xml, obj_bytes, terrain_sdf,
    terrain_mesh_from_grid)

LAYER = 'elevation'
TILE_CELLS = 50
TILE_SAMPLES = 51
TILE_M = 2.5
_CENTER_OFFSET = 1.275


def tile_center(ix: int, iy: int):
    return (TILE_M * ix + _CENTER_OFFSET, TILE_M * iy + _CENTER_OFFSET)


def tile_index_of(pose_x: float, pose_y: float):
    """Copied verbatim from navi_localization.tiles - the sim package must
    not depend on the rover package, and the round-trip test pins them."""
    return (int(round((pose_x - _CENTER_OFFSET) / TILE_M)),
            int(round((pose_y - _CENTER_OFFSET) / TILE_M)))


def model_name(key, generation: int) -> str:
    return f"terrain_{key[0]}_{key[1]}_{'ab'[generation % 2]}"


def elevation_from_message(message: GridMap):
    """(elevation, resolution, center_x, center_y) from a GridMap.

    The inverse of elevation_mapper.build_tile_message: grid_map's matrix
    has its index (0, 0) at the largest x and largest y, rows running in
    -x and columns in -y, stored column-major. The array returned here is
    the ordinary one - row 0 at the smallest y, column 0 at the smallest x -
    which is what terrain_mesh_from_grid expects.
    """
    if LAYER not in message.layers:
        raise ValueError(
            f"no '{LAYER}' layer in {list(message.layers)}; this node maps "
            "elevation and will not guess which other layer meant height")
    if message.outer_start_index or message.inner_start_index:
        raise ValueError(
            "the grid_map circular-buffer start indices are not zero. This "
            "reader does not unroll them, and elevation_mapper never sets "
            "them, so the message did not come from the rover's mapper.")

    layer = message.data[message.layers.index(LAYER)]
    n_cols = layer.layout.dim[0].size
    n_rows = layer.layout.dim[1].size
    grid = np.asarray(layer.data, dtype=np.float32).reshape(n_cols, n_rows).T
    elevation = grid.T[::-1, ::-1]
    return (elevation, float(message.info.resolution),
            float(message.info.pose.position.x),
            float(message.info.pose.position.y))


class TileRespawnPolicy:
    """Per tile at most one replacement a second, the newest payload wins,
    at most `max_in_flight` spawns outstanding, a failed spawn is retried."""

    def __init__(self, min_interval_s: float = 1.0, max_in_flight: int = 4):
        self.min_interval = float(min_interval_s)
        self.max_in_flight = int(max_in_flight)
        self._pending = {}        # key -> payload
        self._shown = {}          # key -> payload on screen
        self._finished_at = {}    # key -> time
        self._in_flight = set()

    def offer(self, key, payload, now: float) -> None:
        if payload == self._shown.get(key):
            self._pending.pop(key, None)
            return
        self._pending[key] = payload

    def next_due(self, now: float) -> list:
        out = []
        for key, payload in list(self._pending.items()):
            if len(self._in_flight) + len(out) >= self.max_in_flight:
                break
            if key in self._in_flight:
                continue
            last = self._finished_at.get(key)
            if last is not None and now - last < self.min_interval:
                continue
            out.append((key, payload))
        return out

    def started(self, key) -> None:
        self._in_flight.add(key)

    def finished(self, key, payload, now: float, ok: bool) -> None:
        self._in_flight.discard(key)
        self._finished_at[key] = now
        if ok:
            self._shown[key] = payload
            if self._pending.get(key) == payload:
                del self._pending[key]


class TerrainWriter(Node):

    def __init__(self, model_dir: str = None) -> None:
        super().__init__('terrain_writer')
        self.declare_parameter('tile_topic', '/localization/map_tile')
        self.declare_parameter('draw_resolution', 0.05)
        self.declare_parameter(
            'model_dir', model_dir or os.path.join(
                os.path.expanduser('~'), '.gazebo', 'models', GAZEBO_MODEL_NAME))

        self._model_dir = str(self.get_parameter('model_dir').value)
        self._mesh_dir = os.path.join(self._model_dir, 'meshes')
        self._draw_resolution = float(self.get_parameter('draw_resolution').value)
        self._policy = TileRespawnPolicy()
        self._generation = {}     # key -> int, flips on every replacement
        self._current = {}        # key -> model name on screen
        self._mesh_file = {}      # key -> mesh file name on screen
        self._version = 0
        self._warned_no_service = False

        os.makedirs(self._mesh_dir, exist_ok=True)
        with open(os.path.join(self._model_dir, 'model.config'), 'w') as handle:
            handle.write(model_config_xml())

        self._delete = self.create_client(DeleteEntity, '/delete_entity')
        self._spawn = self.create_client(SpawnEntity, '/spawn_entity')
        self.create_subscription(
            GridMap, str(self.get_parameter('tile_topic').value), self._on_tile, 16)
        self.create_timer(0.25, self._pump)
        self.get_logger().info(
            f"terrain tiles under {self._model_dir}; one model per 2.5 m tile, "
            "replaced at most once a second, spawned before the old one is deleted")

    def _on_tile(self, message: GridMap) -> None:
        try:
            elevation, resolution, center_x, center_y = elevation_from_message(message)
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        key = tile_index_of(center_x, center_y)
        if not np.isfinite(elevation).any():
            payload = b''                       # the tile is gone
        else:
            stride = max(1, int(round(self._draw_resolution / resolution)))
            mesh = terrain_mesh_from_grid(elevation[::stride, ::stride], resolution * stride,
                                          center_x, center_y)
            payload = obj_bytes(mesh) if mesh is not None else b''
        self._policy.offer(key, payload, time.monotonic())

    def _pump(self, now: float = None) -> None:
        now = time.monotonic() if now is None else now
        if not self._spawn.service_is_ready() or not self._delete.service_is_ready():
            if not self._warned_no_service:
                self._warned_no_service = True
                self.get_logger().warn("/spawn_entity is not up yet - is gazebo running "
                                       "with libgazebo_ros_factory.so? Retrying.")
            return
        self._warned_no_service = False
        for key, payload in self._policy.next_due(now):
            self._policy.started(key)
            if payload == b'':
                self._remove(key, payload, now)
            else:
                self._replace(key, payload, now)

    def _replace(self, key, payload: bytes, now: float) -> None:
        self._version += 1
        name = f"tile_{key[0]}_{key[1]}_v{self._version:05d}.obj"
        with open(os.path.join(self._mesh_dir, name), 'wb') as handle:
            handle.write(payload)
        generation = self._generation.get(key, -1) + 1
        model = model_name(key, generation)
        sdf = terrain_sdf(f"model://{GAZEBO_MODEL_NAME}/meshes/{name}", model)
        request = SpawnEntity.Request()
        request.name = model
        request.xml = sdf
        future = self._spawn.call_async(request)
        future.add_done_callback(
            lambda done: self._on_spawned(done, key, payload, model, name, generation, now))

    def _on_spawned(self, future, key, payload, model, mesh_name, generation, now) -> None:
        try:
            response = future.result()
            ok = bool(response.success)
            error = response.status_message
        except Exception as exc:                        # noqa: BLE001
            ok, error = False, str(exc)
        if not ok:
            self.get_logger().error(f"spawning {model} failed: {error}")
            self._unlink(mesh_name)
            self._policy.finished(key, payload, now, ok=False)
            return
        previous_model, previous_mesh = self._current.get(key), self._mesh_file.get(key)
        self._current[key], self._mesh_file[key], self._generation[key] = model, mesh_name, generation
        self._policy.finished(key, payload, now, ok=True)
        if previous_model:
            self._delete_model(previous_model, previous_mesh)

    def _remove(self, key, payload, now: float) -> None:
        model, mesh = self._current.pop(key, None), self._mesh_file.pop(key, None)
        if model:
            self._delete_model(model, mesh)
        self._policy.finished(key, payload, now, ok=True)

    def _delete_model(self, model: str, mesh_name) -> None:
        future = self._delete.call_async(DeleteEntity.Request(name=model))
        future.add_done_callback(lambda _done: self._unlink(mesh_name))

    def _unlink(self, mesh_name) -> None:
        if mesh_name:
            try:
                os.remove(os.path.join(self._mesh_dir, mesh_name))
            except OSError:
                pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TerrainWriter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException is how ros2 launch's SIGINT reaches a
        # spinning node; uncaught it is logged as a crash with exit code 1.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
