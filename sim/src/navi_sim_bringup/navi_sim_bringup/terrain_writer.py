#!/usr/bin/env python3
"""Shows the ground the rover has mapped as terrain in the Gazebo view.

Gazebo Classic cannot change a heightmap in place, so a changed map means
deleting the terrain model and spawning a new one. That is expensive and
visible, which is where the design's two rules come from: at most one
respawn every five seconds, and only when the map actually changed.

Nothing else in the world is touched. The rover model is never deleted, and
the world's ground plane at z = 0 stays, so the rover is never in the void
and never blinks while the terrain is replaced.

Runs on the simulation's ROS domain. /localization/map reaches that domain
from the rover's domain 0 through sim_bridge (sub-project 2); this node
knows nothing about domains and simply subscribes.

Each version gets its own image file name. Gazebo caches heightmap data (and
its level-of-detail paging under ~/.gazebo/paging) by image file name, so
respawning with the same name can put the *previous* terrain back on the
screen while every log line says the new one was loaded. A new name every
time sidesteps that entirely; the old file is deleted once the new one is up.
"""

import os
import time

import numpy as np
import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from grid_map_msgs.msg import GridMap
from rclpy.node import Node

from navi_sim_bringup.heightmap import (
    GAZEBO_MODEL_NAME, MODEL_NAME, heightmap_from_grid, model_config_xml,
    png_bytes, terrain_sdf)

LAYER = 'elevation'
MAP_TOPIC = '/localization/map'


def elevation_from_message(message: GridMap):
    """(elevation, resolution, center_x, center_y) from a GridMap.

    The inverse of elevation_mapper.build_grid_map_message: grid_map's
    matrix has its index (0, 0) at the largest x and largest y, rows running
    in -x and columns in -y, stored column-major. The array returned here is
    the ordinary one - row 0 at the smallest y, column 0 at the smallest x -
    which is what heightmap_from_grid expects.
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


class RespawnPolicy:
    """At most one respawn per interval, and only when the terrain changed."""

    def __init__(self, interval_seconds: float = 5.0):
        self.interval = float(interval_seconds)
        self._spawned = None
        self._pending = None
        self._last_respawn = None

    def offer(self, payload, now: float) -> bool:
        """Records a candidate terrain. True if it should be spawned now."""
        if payload == self._spawned:
            return False
        self._pending = payload
        return self.due(now)

    def due(self, now: float) -> bool:
        if self._pending is None:
            return False
        if self._last_respawn is None:
            return True
        return now - self._last_respawn >= self.interval

    @property
    def pending(self):
        return self._pending

    def respawned(self, payload, now: float) -> None:
        """Called once Gazebo has confirmed `payload` is on the screen.

        `payload` rather than whatever is pending: a newer map can arrive
        while the spawn service call is in flight, and marking that newer
        one as spawned would drop it for good.
        """
        self._spawned = payload
        if self._pending == payload:
            self._pending = None
        self._last_respawn = now


class TerrainWriter(Node):

    def __init__(self) -> None:
        super().__init__('terrain_writer')
        self.declare_parameter('map_topic', MAP_TOPIC)
        self.declare_parameter('entity_name', MODEL_NAME)
        self.declare_parameter('respawn_interval_seconds', 5.0)
        self.declare_parameter(
            'model_dir',
            os.path.join(os.path.expanduser('~'), '.gazebo', 'models',
                         GAZEBO_MODEL_NAME))

        self._entity_name = str(self.get_parameter('entity_name').value)
        self._model_dir = str(self.get_parameter('model_dir').value)
        self._texture_dir = os.path.join(self._model_dir, 'materials', 'textures')
        self._policy = RespawnPolicy(
            float(self.get_parameter('respawn_interval_seconds').value))
        self._version = 0
        self._heightmap = None
        self._image_name = None
        self._busy = False
        self._spawned_once = False
        self._warned_no_service = False

        os.makedirs(self._texture_dir, exist_ok=True)
        with open(os.path.join(self._model_dir, 'model.config'), 'w') as handle:
            handle.write(model_config_xml())

        self._delete = self.create_client(DeleteEntity, '/delete_entity')
        self._spawn = self.create_client(SpawnEntity, '/spawn_entity')
        self.create_subscription(
            GridMap, str(self.get_parameter('map_topic').value), self._on_map, 1)
        # The cap is a wall-clock rate limit protecting Gazebo from a
        # respawn storm, so it is measured in wall-clock seconds
        # (time.monotonic) and not on /clock.
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f"terrain model files under {self._model_dir}; respawning "
            f"'{self._entity_name}' at most every "
            f"{self._policy.interval:.0f} s and only when the map changes")

    def _on_map(self, message: GridMap) -> None:
        try:
            elevation, resolution, center_x, center_y = elevation_from_message(message)
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        heightmap = heightmap_from_grid(elevation, resolution, center_x, center_y)
        if heightmap is None:
            return
        self._heightmap = heightmap
        if self._policy.offer(png_bytes(heightmap.image), time.monotonic()):
            self._respawn()

    def _tick(self) -> None:
        if self._policy.due(time.monotonic()):
            self._respawn()

    def _respawn(self) -> None:
        if self._busy:
            return
        if not (self._spawn.service_is_ready()
                and (self._delete.service_is_ready() or not self._spawned_once)):
            if not self._warned_no_service:
                self._warned_no_service = True
                self.get_logger().warn(
                    "/spawn_entity is not up yet - is gazebo running with "
                    "libgazebo_ros_factory.so? Retrying every second.")
            return
        self._warned_no_service = False
        self._busy = True

        payload = self._policy.pending
        self._version += 1
        name = f"heightmap_{self._version:04d}.png"
        with open(os.path.join(self._texture_dir, name), 'wb') as handle:
            handle.write(payload)
        uri = f"model://{GAZEBO_MODEL_NAME}/materials/textures/{name}"
        sdf = terrain_sdf(uri, self._heightmap, self._entity_name)
        with open(os.path.join(self._model_dir, 'model.sdf'), 'w') as handle:
            handle.write(sdf)

        previous, self._image_name = self._image_name, name
        if self._spawned_once:
            future = self._delete.call_async(
                DeleteEntity.Request(name=self._entity_name))
            future.add_done_callback(
                lambda _future: self._send_spawn(sdf, payload, previous))
        else:
            self._send_spawn(sdf, payload, previous)

    def _send_spawn(self, sdf: str, payload: bytes, previous) -> None:
        request = SpawnEntity.Request()
        request.name = self._entity_name
        request.xml = sdf
        future = self._spawn.call_async(request)
        future.add_done_callback(
            lambda done: self._on_spawned(done, payload, previous))

    def _on_spawned(self, future, payload: bytes, previous) -> None:
        self._busy = False
        try:
            response = future.result()
        except Exception as error:                      # noqa: BLE001
            self.get_logger().error(f"spawning the terrain failed: {error}")
            return
        if not response.success:
            self.get_logger().error(
                f"Gazebo refused the terrain: {response.status_message}")
            return
        self._spawned_once = True
        self._policy.respawned(payload, time.monotonic())
        if previous:
            try:
                os.remove(os.path.join(self._texture_dir, previous))
            except OSError:
                pass
        self.get_logger().info(
            f"terrain version {self._version}: "
            f"{self._heightmap.side} x {self._heightmap.side} samples, "
            f"{self._heightmap.size_x:.1f} x {self._heightmap.size_y:.1f} m, "
            f"height span {self._heightmap.size_z:.2f} m")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TerrainWriter()
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
