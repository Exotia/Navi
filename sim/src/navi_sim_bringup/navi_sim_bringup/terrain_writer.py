#!/usr/bin/env python3
"""Shows the ground the rover has mapped as terrain in the Gazebo view.

One Gazebo model per 2.5 m tile. Gazebo Classic cannot change a model's
mesh in place, so a changed tile means spawning a new model and deleting
the old one - the replacement is spawned before the old one is deleted so
the ground never blinks. Per tile, at most one replacement a second.

The terrain is a mesh, not a heightmap - see terrain_mesh.py for the
gzserver crash that decided it.

Nothing else in the world is touched. The rover model is never deleted, and
the world's ground plane at z = 0 stays, so the rover is never in the void.

Runs on the simulation's ROS domain. /localization/map_tile reaches that
domain from the rover's domain 0 through sim_bridge (sub-project 2); this
node knows nothing about domains and simply subscribes.

Each replacement gets its own mesh file name and its own model name
(terrain_<ix>_<iy>_<run id>_g<generation>, generation only ever increasing
per tile, run id drawn once per process). Two reasons: Gazebo's MeshManager
caches meshes by file name, so respawning with the same mesh name would put
the *previous* terrain back on the screen while every log line says the new
one was loaded; and a model name can only be reused once Gazebo confirms
the old one is gone, so a name that is ever repeated (the old a/b
alternation) will eventually collide with a model whose delete is still
stuck, and Gazebo refuses the spawn outright ("Entity [...] already
exists"). The generation number rules that out within a run and the run id
rules it out across runs - see `_delete_model`/`_doomed` below for the
other half of the fix: a delete failure used to be silently dropped,
orphaning the model forever.

Round 3: under sustained tile churn (several replacements a second across
many tiles), /delete_entity was observed answering success=True while the
model stayed in the world - Gazebo's single-threaded factory service
appears to become unreliable once too many SpawnEntity/DeleteEntity
requests are outstanding at once. Two fixes:

1. One shared "factory budget" (`_factory_in_flight`, capped at
   `MAX_FACTORY_IN_FLIGHT`) now covers *both* spawns and deletes, not just
   spawns. A delete is never dispatched the instant a spawn confirms (the
   `_delete_model` call from `_on_spawned`); it only registers the model as
   doomed, and the bounded loop in `_pump` dispatches (and retries) the
   actual DeleteEntity call, waiting its turn for a slot exactly like a
   spawn does. This is what actually bounds the total request rate that
   was overwhelming Gazebo.
2. `response.success` from DeleteEntity is no longer trusted alone. A
   doomed model is only untracked - and its mesh file unlinked - once
   /get_model_list, polled once a second while anything is doomed, agrees
   it is actually gone. A model still listed `DELETE_CONFIRM_GRACE_S`
   after a "successful" delete response gets the delete re-sent.
   At start-up, any terrain_* model already in the world (left over from a
   previous, uncleanly stopped run) is found the same way and deleted
   through the same bounded path.

Round 4: the same live run showed factory requests that simply never come
back - neither success nor failure, just a future that never resolves.
Every dispatched request therefore carries its dispatch time and a token;
`_pump` treats anything older than FACTORY_REQUEST_TIMEOUT_S as failed,
frees its slot and retries it, and the token makes a late resolution a
no-op so a slot is never released twice. Without that watchdog, four
stalled requests wedged the shared budget shut forever.

Round 5 closed the two places that watchdog did not reach. The
/get_model_list poll had a watchdog of its own missing entirely: its
in-flight flag was cleared only by an answer, so one unanswered poll
stopped every further poll for the rest of the run - and since spawning
and deleting carried on regardless, the only thing lost was the
verification that deletes actually happened, silently. And a written-off
spawn used to be retried under the *same* model name and its mesh
unlinked, on the assumption that a request without an answer is a request
that did not happen. It may well have happened: `_generation[key]` is now
committed at dispatch, so a retry always carries a fresh name, and the
unanswered name is doomed like any other superseded model - if Gazebo
never created it, the model-list poll finds it absent and untracks it for
free.
"""

import os
import re
import time

import numpy as np
import rclpy
from gazebo_msgs.srv import DeleteEntity, GetModelList, SpawnEntity
from rclpy.executors import ExternalShutdownException
from grid_map_msgs.msg import GridMap
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

from navi_sim_bringup.obstacle_mesh import (
    obj_bytes as obstacle_obj_bytes, obstacle_mesh_from_voxels, obstacle_sdf)
from navi_sim_bringup.terrain_mesh import (
    GAZEBO_MODEL_NAME, model_config_xml, obj_bytes, terrain_sdf,
    terrain_mesh_from_grid)

LAYER = 'elevation'
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


def model_name(key, generation: int, run_id: str) -> str:
    """Every generation of every tile gets its own name, for this run and
    for any run that overlaps a leftover of a previous one.

    `key` is `(kind, ix, iy)` with `kind` one of `"terrain"`/`"obst"` -
    terrain and obstacle tiles at the same `(ix, iy)` are independent
    models, named `terrain_<ix>_<iy>_<run>_g<N>` / `obst_<ix>_<iy>_<run>_g<N>`.

    Not a/b alternation: that reused a name every other replacement, and a
    name can only safely be reused once Gazebo has confirmed the old model
    holding it is gone. A delete that is still stuck (see `_doomed`)
    then made the *next* spawn of that name fail with "already exists" -
    observed in the field as thousands of such errors and stray models
    left on screen.

    The guarantee is: within one process, `generation` only ever increases
    per tile, so a name is never reused however many deletes are stuck;
    across processes, `run_id` (6 random hex characters, drawn once per
    TerrainWriter) makes a fresh run's names disjoint from any leftover of
    an earlier run that the start-up sweep has not finished deleting yet.
    It is not a mathematical impossibility - two runs can in principle draw
    the same 6 hex characters (about 1 in 16.8 million) - but nothing weaker
    than a random run id would survive an unclean restart at all, because
    generation numbers restart at 0.
    """
    kind, ix, iy = key
    return f"{kind}_{ix}_{iy}_{run_id}_g{generation}"


# Names this node may have spawned, in any build: the current
# terrain_<ix>_<iy>_<runid>_g<N> / obst_<ix>_<iy>_<runid>_g<N>, the
# run-id-less form that preceded it, and the original a/b alternation (both
# pre-date the obst_ kind, so they only ever apply to terrain_ leftovers, but
# matching them under either prefix is harmless - no obst_ model has ever
# been spawned in those older forms). Used by the start-up sweep, which must
# clean up leftovers from *any* previous run, not just ones this build wrote.
LEFTOVER_MODEL_RE = re.compile(
    r'^(?:terrain|obst)_-?\d+_-?\d+_(?:[0-9a-f]{6}_g\d+|g\d+|[ab])$')


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


# The exact PointCloud2 layout the rover's obstacle-tile publisher writes
# (see sub-project 2 / Task 3-4): x/y/z float32 at these offsets, tightly
# packed with no per-point padding.
_OBSTACLE_FIELD_OFFSETS = {'x': 0, 'y': 4, 'z': 8}
_OBSTACLE_POINT_STEP = 12


def obstacle_key_from_frame_id(frame_id: str):
    """("obst", ix, iy) from a "map|<ix>|<iy>" frame_id.

    That odd home for the tile index is the spec's, not this node's choice:
    a PointCloud2 has nowhere else that can carry identity for an *empty*
    tile (width 0, no points to read an index back out of), which an
    obstacle tile going fully clear must still be able to publish.
    """
    parts = frame_id.split('|')
    if len(parts) != 3 or parts[0] != 'map':
        raise ValueError(
            f"obstacle tile frame_id {frame_id!r} is not 'map|<ix>|<iy>'")
    try:
        ix, iy = int(parts[1]), int(parts[2])
    except ValueError as error:
        raise ValueError(
            f"obstacle tile frame_id {frame_id!r} has non-integer indices"
        ) from error
    return ('obst', ix, iy)


def obstacle_centres_from_message(message: PointCloud2) -> np.ndarray:
    """(N, 3) float32 voxel centres from an obstacle-tile PointCloud2.

    Refuses (ValueError) anything but the exact layout this topic is
    specified to carry - fields x/y/z FLOAT32 at offsets 0/4/8, point_step
    12, height 1, little-endian - matched by name/offset/datatype rather
    than trusting point_step alone, so a message that merely happens to
    share point_step with some other layout is not silently misread as
    voxel centres. An empty tile (width 0) is a valid, empty result, not a
    malformed one - that is how a tile going fully clear is published.
    """
    if message.height != 1:
        raise ValueError(f"obstacle tile height {message.height} != 1")
    if message.is_bigendian:
        raise ValueError("obstacle tile is big-endian; not supported")
    if message.point_step != _OBSTACLE_POINT_STEP:
        raise ValueError(
            f"obstacle tile point_step {message.point_step} != "
            f"{_OBSTACLE_POINT_STEP}")
    fields = {field.name: field for field in message.fields}
    if set(fields) != set(_OBSTACLE_FIELD_OFFSETS):
        raise ValueError(
            f"obstacle tile fields {sorted(fields)} != x/y/z")
    for name, expected_offset in _OBSTACLE_FIELD_OFFSETS.items():
        field = fields[name]
        if field.datatype != PointField.FLOAT32:
            raise ValueError(f"obstacle tile field {name!r} is not FLOAT32")
        if field.offset != expected_offset:
            raise ValueError(
                f"obstacle tile field {name!r} at offset {field.offset}, "
                f"expected {expected_offset}")
    if message.width == 0:
        return np.zeros((0, 3), dtype=np.float32)
    data = bytes(message.data)
    expected_bytes = message.width * message.point_step
    if len(data) != expected_bytes:
        raise ValueError(
            f"obstacle tile data is {len(data)} bytes, expected "
            f"{expected_bytes}")
    return np.frombuffer(data, dtype='<f4').reshape(-1, 3)


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
        # `now` is accepted for symmetry with started/finished/next_due but
        # unused here: whether a payload is new never depends on the clock,
        # only whether it is due (next_due) does.
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

    # A delete that keeps failing is retried at most this many times before
    # this node gives up on it (logged at ERROR) - a stuck Gazebo/rosbridge
    # would otherwise retry forever, once a second, per stuck model.
    MAX_DELETE_ATTEMPTS = 20
    DELETE_RETRY_INTERVAL_S = 1.0

    # Total SpawnEntity + DeleteEntity requests outstanding at once, across
    # every tile. Bounding this - not just spawns - is what round 3 added:
    # deletes fired the instant a spawn confirmed, uncapped, and under
    # sustained churn that flood of concurrent DeleteEntity calls is what
    # made Gazebo's factory service unreliable.
    MAX_FACTORY_IN_FLIGHT = 4

    # How long a dispatched SpawnEntity/DeleteEntity request may go
    # unanswered before it is written off as failed and its budget slot
    # freed. Gazebo answers in milliseconds when it answers at all; the
    # failure mode this guards against is a request that never comes back,
    # which without a watchdog holds its slot forever and (four of them)
    # wedges the whole node.
    FACTORY_REQUEST_TIMEOUT_S = 10.0

    # How often /get_model_list is polled while anything is doomed, and how
    # long a model may stay listed after a "successful" delete response
    # before that response is disbelieved and the delete is re-sent.
    MODEL_LIST_POLL_INTERVAL_S = 1.0
    DELETE_CONFIRM_GRACE_S = 2.0

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
        # Injectable so tests can control completion time independently of
        # dispatch time (`_pump`'s `now` argument) - see `_on_spawned`/
        # `_remove`, which stamp the policy with `self._clock_fn()` at their own
        # execution time, not with whenever `_pump` happened to be called.
        self._clock_fn = time.monotonic
        # Six random hex characters, drawn once per process and baked into
        # every model name (see `model_name`). Generation numbers restart at
        # 0 on every start, so without this a fresh spawn of tile (0, 0)
        # after an unclean restart would ask for terrain_0_0_g0 while the
        # previous run's terrain_0_0_g0 was still in the world, waiting for
        # the start-up sweep's delete to confirm - and Gazebo would refuse
        # it with "already exists".
        self._run_id = os.urandom(3).hex()
        self._policy = TileRespawnPolicy()
        self._generation = {}     # key -> int, only ever increases
        self._current = {}        # key -> model name on screen
        self._mesh_file = {}      # key -> mesh file name on screen
        self._max_delete_attempts = self.MAX_DELETE_ATTEMPTS
        self._delete_retry_interval = self.DELETE_RETRY_INTERVAL_S
        # model -> {'mesh_name', 'attempts', 'last_attempt', 'in_flight',
        # 'confirmed_at'}. A model is doomed the instant it is superseded
        # and stays here - retried by `_pump` - until /get_model_list
        # confirms it is actually gone, or this node gives up after
        # `_max_delete_attempts`. Without this a failed (or falsely
        # "successful") delete was silently forgotten: the model dropped
        # out of `_current`/`_mesh_file` bookkeeping but never actually
        # left Gazebo, orphaning it forever and, under the old a/b naming,
        # eventually blocking a same-named spawn with "already exists".
        self._doomed = {}
        # Total SpawnEntity + DeleteEntity requests dispatched and not yet
        # answered, across every tile - see MAX_FACTORY_IN_FLIGHT.
        self._max_factory_in_flight = self.MAX_FACTORY_IN_FLIGHT
        self._factory_timeout = self.FACTORY_REQUEST_TIMEOUT_S
        # token -> {'kind', 'dispatched_at', 'model', and, for a spawn, the
        # 'key'/'payload'/'mesh_name' needed to hand the tile back to the
        # policy if the request is written off. `_factory_in_flight` is just
        # this dict's size; the token is what makes releasing a slot
        # idempotent, so a response that arrives after the watchdog already
        # wrote the request off cannot free a second, unrelated slot.
        self._factory_requests = {}
        self._next_factory_token = 0
        self._model_list_interval = self.MODEL_LIST_POLL_INTERVAL_S
        self._delete_confirm_grace = self.DELETE_CONFIRM_GRACE_S
        self._last_list_poll = None
        # The token of the /get_model_list poll that is outstanding, or None.
        # A token rather than a bare flag for the same reason the factory
        # requests carry one: a poll Gazebo never answers is written off
        # after `_factory_timeout`, and the answer that turns up afterwards
        # must not be mistaken for the answer to the poll that replaced it.
        self._model_list_token = None
        self._model_list_dispatched_at = None
        self._next_model_list_token = 0
        self._startup_scan_done = False
        self._version = 0
        self._warned_no_service = False

        os.makedirs(self._mesh_dir, exist_ok=True)
        with open(os.path.join(self._model_dir, 'model.config'), 'w') as handle:
            handle.write(model_config_xml())

        self._delete = self.create_client(DeleteEntity, '/delete_entity')
        self._spawn = self.create_client(SpawnEntity, '/spawn_entity')
        self._model_list = self.create_client(GetModelList, '/get_model_list')
        self.create_subscription(
            GridMap, str(self.get_parameter('tile_topic').value), self._on_tile, 64)
        self.create_subscription(
            PointCloud2, '/localization/obstacle_tile', self._on_obstacle_tile, 64)
        self.create_timer(0.25, self._pump)
        self.get_logger().info(
            f"terrain tiles under {self._model_dir}; one model per 2.5 m tile, "
            "replaced at most once a second, spawned before the old one is "
            f"deleted; this run's models are named terrain_<ix>_<iy>_"
            f"{self._run_id}_g<n>")

    @property
    def _factory_in_flight(self) -> int:
        """SpawnEntity + DeleteEntity requests dispatched and unanswered."""
        return len(self._factory_requests)

    def _claim_factory_slot(self, kind: str, now, **context) -> int:
        """Takes one of the shared factory slots and returns its token."""
        self._next_factory_token += 1
        token = self._next_factory_token
        self._factory_requests[token] = dict(
            kind=kind,
            dispatched_at=self._clock_fn() if now is None else now,
            **context)
        return token

    def _release_factory_slot(self, token: int) -> bool:
        """True if this call released the slot, False if it was already gone.

        False means the watchdog wrote the request off first, and the caller
        (a late `_on_spawned`/`_on_deleted`) must not do any accounting: the
        tile has already been handed back to the policy and possibly
        re-dispatched, so a second release would free somebody else's slot.
        """
        return self._factory_requests.pop(token, None) is not None

    def _expire_stalled_factory_requests(self, now: float) -> None:
        """Writes off factory requests Gazebo never answered.

        Round 2's live run saw SpawnEntity/DeleteEntity futures that never
        resolved either way. Their slots were only ever freed by a response,
        so four of them permanently wedged the shared budget and the node
        stopped spawning and deleting anything at all, silently. A written-off
        request is treated exactly like a failed one: the tile goes back to
        the policy for a retry, a doomed model keeps its counted attempt and
        becomes due again.
        """
        for token, entry in list(self._factory_requests.items()):
            waited = now - entry['dispatched_at']
            if waited < self._factory_timeout:
                continue
            del self._factory_requests[token]
            self.get_logger().warn(
                f"{entry['kind']} of {entry['model']} was never answered by "
                f"Gazebo ({waited:.1f}s); treating it as failed and freeing "
                "the factory slot")
            if entry['kind'] == 'spawn':
                # An unanswered spawn is not a spawn that did not happen:
                # Gazebo may well have created the model and merely lost
                # the reply. Dropping the name here left a model nothing
                # would ever delete, so it is doomed like any other
                # superseded model - and if it truly never existed, the
                # /get_model_list poll finds it absent and untracks it (and
                # unlinks the mesh) at no cost. `due_at` gives the poll
                # that first word, so the common case costs no DeleteEntity
                # traffic at all.
                self._delete_model(entry['model'], entry['mesh_name'], due_at=now)
                self._policy.finished(
                    entry['key'], entry['payload'], self._clock_fn(), ok=False)
                continue
            doomed = self._doomed.get(entry['model'])
            if doomed is not None:
                doomed['in_flight'] = False            # due again after the interval
            self._maybe_give_up(entry['model'])

    def _on_tile(self, message: GridMap) -> None:
        try:
            elevation, resolution, center_x, center_y = elevation_from_message(message)
        except (ValueError, IndexError) as error:
            # IndexError too: a message whose layout carries fewer than two
            # dimensions indexes off the end rather than failing a check,
            # and this is a subscription callback - anything that escapes
            # it ends the node and takes the terrain view with it.
            self.get_logger().error(f"unreadable tile message: {error!r}")
            return
        key = ('terrain',) + tile_index_of(center_x, center_y)
        if not np.isfinite(elevation).any():
            payload = b''                       # the tile is gone
        else:
            stride = max(1, int(round(self._draw_resolution / resolution)))
            mesh = terrain_mesh_from_grid(elevation[::stride, ::stride], resolution * stride,
                                          center_x, center_y)
            payload = obj_bytes(mesh) if mesh is not None else b''
        self._policy.offer(key, payload, self._clock_fn())

    def _on_obstacle_tile(self, message: PointCloud2) -> None:
        try:
            key = obstacle_key_from_frame_id(message.header.frame_id)
            centres = obstacle_centres_from_message(message)
        except (ValueError, IndexError) as error:
            # Same reasoning as _on_tile: this is a subscription callback,
            # and anything that escapes it ends the node and takes the
            # terrain/obstacle view down with it.
            self.get_logger().error(f"unreadable obstacle tile message: {error!r}")
            return
        if centres.shape[0] == 0:
            payload = b''                       # the tile is gone
        else:
            mesh = obstacle_mesh_from_voxels(centres)
            payload = obstacle_obj_bytes(mesh) if mesh is not None else b''
        self._policy.offer(key, payload, self._clock_fn())

    def _pump(self, now: float = None) -> None:
        now = self._clock_fn() if now is None else now
        if not (self._spawn.service_is_ready() and self._delete.service_is_ready()
                and self._model_list.service_is_ready()):
            if not self._warned_no_service:
                self._warned_no_service = True
                missing = [name for name, client in
                           (('/spawn_entity', self._spawn),
                            ('/delete_entity', self._delete),
                            ('/get_model_list', self._model_list))
                           if not client.service_is_ready()]
                self.get_logger().warn(
                    f"waiting for {', '.join(missing)}: this node needs all three "
                    "and draws nothing until every one is up. /spawn_entity and "
                    "/delete_entity come from libgazebo_ros_factory.so, "
                    "/get_model_list from libgazebo_ros_state.so - is gazebo "
                    "running with both plugins? Retrying.")
            return
        self._warned_no_service = False

        if not self._startup_scan_done:
            self._startup_scan_done = True
            self._scan_for_leftover_models()

        self._expire_stalled_factory_requests(now)
        self._expire_stalled_model_list_poll(now)
        self._maybe_poll_model_list(now)

        for model, entry in list(self._doomed.items()):
            if entry['in_flight']:
                continue
            if entry.get('confirmed_at') is not None:
                # A "successful" delete is not trusted alone - wait for
                # /get_model_list to either confirm it is gone (untracks it,
                # in `_on_model_list`) or say it is still there long enough
                # to reset `confirmed_at` and make it due again below.
                continue
            if now - entry['last_attempt'] < self._delete_retry_interval:
                continue
            if self._factory_in_flight >= self._max_factory_in_flight:
                continue                    # no slot this tick; its turn comes later
            self._attempt_delete(model, now)

        for key, payload in self._policy.next_due(now):
            if payload == b'':
                # A removal never itself calls the factory (it only
                # registers the old model as doomed - see `_remove`), so it
                # never needs a budget slot.
                self._policy.started(key)
                self._remove(key, payload)
                continue
            if self._factory_in_flight >= self._max_factory_in_flight:
                continue                    # wait its turn; still pending next tick
            self._policy.started(key)
            try:
                self._replace(key, payload, now)
            except Exception as exc:                    # noqa: BLE001
                # Writing the mesh file, building the SDF, or call_async
                # itself can all raise before any future/callback exists to
                # ever call `finished` - without this the key would be
                # stranded in `_in_flight` forever, permanently blocking
                # this tile and shrinking the global in-flight cap.
                self.get_logger().error(
                    f"dispatching a replacement for tile {key} failed: {exc}")
                self._policy.finished(key, payload, self._clock_fn(), ok=False)

    def _replace(self, key, payload: bytes, now: float = None) -> None:
        kind, ix, iy = key
        self._version += 1
        # Mesh file prefix is not the same as the model-name prefix
        # (`tile_` predates the `terrain_`/`obst_` model-name split and stays
        # as-is for terrain; `obst_` is the new kind's own).
        prefix = 'tile' if kind == 'terrain' else 'obst'
        name = f"{prefix}_{ix}_{iy}_v{self._version:05d}.obj"
        generation = self._generation.get(key, -1) + 1
        # Committed here, at dispatch, not in `_on_spawned` on success. A
        # spawn that fails or is never answered may still have created the
        # model in Gazebo, and retrying under the same name then fails for
        # good with "already exists" - so the number is burned whatever
        # happens. Generations are free; a name that can never be spawned
        # again is not.
        self._generation[key] = generation
        model = model_name(key, generation, self._run_id)
        token = self._claim_factory_slot(
            'spawn', now, model=model, key=key, payload=payload, mesh_name=name)
        try:
            # Inside the try: an ENOSPC halfway through the write would
            # otherwise leave a truncated .obj behind for good.
            with open(os.path.join(self._mesh_dir, name), 'wb') as handle:
                handle.write(payload)
            mesh_uri = f"model://{GAZEBO_MODEL_NAME}/meshes/{name}"
            sdf = (terrain_sdf(mesh_uri, model) if kind == 'terrain'
                   else obstacle_sdf(mesh_uri, model))
            request = SpawnEntity.Request()
            request.name = model
            request.xml = sdf
            future = self._spawn.call_async(request)
        except Exception:
            self._release_factory_slot(token)
            self._unlink(name)                          # nothing will load it now
            raise
        future.add_done_callback(
            lambda done: self._on_spawned(done, token, key, payload, model, name))

    def _on_spawned(self, future, token, key, payload, model, mesh_name) -> None:
        if not self._release_factory_slot(token):
            # The watchdog already wrote this request off and handed the
            # tile back to the policy; doing any of that again here would
            # free a slot this request no longer holds.
            self.get_logger().warn(
                f"spawning {model} was answered after it had already timed "
                "out; ignoring the late answer")
            return
        # Stamped with the clock at completion time, not with whatever
        # `now` `_pump` was dispatched at: Gazebo can take a while to
        # answer, and the per-tile 1 s cap must count from when the
        # replacement actually lands, not from when it was requested -
        # otherwise a slow spawn makes the tile eligible again immediately.
        now = self._clock_fn()
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
        # `_generation[key]` was already committed at dispatch (see
        # `_replace`); only what is actually on screen is recorded here.
        self._current[key], self._mesh_file[key] = model, mesh_name
        self._policy.finished(key, payload, now, ok=True)
        if previous_model:
            self._delete_model(previous_model, previous_mesh)

    def _remove(self, key, payload) -> None:
        model, mesh = self._current.pop(key, None), self._mesh_file.pop(key, None)
        if model:
            self._delete_model(model, mesh)
        self._policy.finished(key, payload, self._clock_fn(), ok=True)

    def _delete_model(self, model: str, mesh_name, due_at: float = None) -> None:
        """Marks `model` doomed; never dispatches DeleteEntity itself.

        The instant a spawn confirms, its superseded model must be deleted -
        but firing that DeleteEntity call right here, unconditionally, is
        exactly the uncapped traffic that overwhelmed Gazebo's factory
        service under sustained tile churn (round 3's live-run finding).
        Registering it here and letting the bounded loop in `_pump` do the
        actual dispatch (and every retry) means a delete waits its turn for
        a factory-request slot exactly like a spawn does, and the combined
        spawn+delete rate stays under `_max_factory_in_flight` regardless
        of how many tiles change at once.

        `due_at` holds the first delete attempt back by one retry interval,
        for a model that probably does not exist at all (a spawn Gazebo
        never answered): the model-list poll gets to say so first, and no
        DeleteEntity call is spent on it.
        """
        self._doomed[model] = {
            'mesh_name': mesh_name,
            'attempts': 0,
            # -inf: due the moment a slot is free.
            'last_attempt': float('-inf') if due_at is None else due_at,
            'in_flight': False,
            'confirmed_at': None,
        }

    def _attempt_delete(self, model: str, now: float = None) -> None:
        entry = self._doomed.get(model)
        if entry is None:
            return
        entry['attempts'] += 1
        entry['last_attempt'] = self._clock_fn() if now is None else now
        entry['in_flight'] = True
        token = self._claim_factory_slot('delete', now, model=model)
        try:
            future = self._delete.call_async(DeleteEntity.Request(name=model))
        except Exception as exc:                        # noqa: BLE001
            entry['in_flight'] = False
            self._release_factory_slot(token)
            self.get_logger().warn(f"deleting {model} failed to dispatch: {exc}")
            self._maybe_give_up(model)
            return
        future.add_done_callback(lambda done: self._on_deleted(done, token, model))

    def _on_deleted(self, future, token: int, model: str) -> None:
        if not self._release_factory_slot(token):
            # Written off by the watchdog already - the doomed entry has been
            # freed for a retry (or given up on) without this answer.
            self.get_logger().warn(
                f"deleting {model} was answered after it had already timed "
                "out; ignoring the late answer")
            return
        entry = self._doomed.get(model)
        if entry is None:
            return                                      # already given up
        entry['in_flight'] = False
        try:
            response = future.result()
            ok = bool(response.success)
            error = response.status_message
        except Exception as exc:                        # noqa: BLE001
            ok, error = False, str(exc)
        if ok:
            # Not trusted alone - Gazebo has been observed to answer
            # success=True while the model stays in the world under heavy
            # churn. `_on_model_list` is what actually untracks and unlinks,
            # once /get_model_list agrees the model is gone; if it is still
            # listed `_delete_confirm_grace` after this, the delete is
            # re-sent (see `_on_model_list`).
            entry['confirmed_at'] = self._clock_fn()
            return
        self.get_logger().warn(f"deleting {model} failed: {error}")
        self._maybe_give_up(model)

    def _maybe_give_up(self, model: str) -> None:
        entry = self._doomed.get(model)
        if entry is not None and entry['attempts'] >= self._max_delete_attempts:
            self.get_logger().error(
                f"giving up deleting {model} after {entry['attempts']} attempts; "
                "it will stay in Gazebo until removed manually")
            del self._doomed[model]
            # The model is beyond help, but its mesh is not: this entry was
            # the last reference to that file, so leaving it would grow
            # ~/.gazebo/models without limit over a long run. Gazebo keeps
            # what it has already loaded in memory, so the stranded model
            # on screen does not need the file.
            self._unlink(entry['mesh_name'])

    def _maybe_poll_model_list(self, now: float) -> None:
        """Polls /get_model_list at most once a second while anything is
        doomed - the sole ground truth for whether a delete actually
        happened, since DeleteEntity's own response has been observed to
        lie under load (see `_on_deleted`)."""
        if not self._doomed or self._model_list_token is not None:
            return
        if (self._last_list_poll is not None
                and now - self._last_list_poll < self._model_list_interval):
            return
        self._last_list_poll = now
        self._next_model_list_token += 1
        token = self._next_model_list_token
        self._model_list_token = token
        self._model_list_dispatched_at = now
        try:
            future = self._model_list.call_async(GetModelList.Request())
        except Exception as exc:                        # noqa: BLE001
            self._model_list_token = None
            self._model_list_dispatched_at = None
            self.get_logger().warn(f"listing Gazebo's models failed to dispatch: {exc}")
            return
        future.add_done_callback(lambda done: self._on_model_list(done, token))

    def _expire_stalled_model_list_poll(self, now: float) -> None:
        """Frees the poll slot when Gazebo never answers a /get_model_list.

        The in-flight flag used to be cleared only by an answer, so one
        poll that never came back stopped every further poll for the life
        of the process. Nothing else broke - spawns and deletes carried on -
        which is what made it invisible: only the *verification* of deletes
        was gone, and that verification is the only thing that ever
        untracks a doomed model or unlinks its mesh.
        """
        if self._model_list_token is None or self._model_list_dispatched_at is None:
            return
        waited = now - self._model_list_dispatched_at
        if waited < self._factory_timeout:
            return
        self.get_logger().warn(
            f"/get_model_list was never answered by Gazebo ({waited:.1f}s); "
            "polling again - verifying deletes depends on it")
        self._model_list_token = None                   # a late answer is ignored
        self._model_list_dispatched_at = None

    def _on_model_list(self, future, token: int = None) -> None:
        if token is not None and token != self._model_list_token:
            self.get_logger().warn(
                "/get_model_list was answered after it had already timed out; "
                "ignoring the late answer")
            return
        self._model_list_token = None
        self._model_list_dispatched_at = None
        try:
            response = future.result()
            names = set(response.model_names)
        except Exception as exc:                        # noqa: BLE001
            self.get_logger().warn(f"listing Gazebo's models failed: {exc}")
            return
        now = self._clock_fn()
        for model, entry in list(self._doomed.items()):
            if model not in names:
                self._unlink(entry['mesh_name'])
                del self._doomed[model]
                continue
            confirmed_at = entry.get('confirmed_at')
            if confirmed_at is not None and now - confirmed_at >= self._delete_confirm_grace:
                self.get_logger().warn(
                    f"{model} is still in Gazebo {now - confirmed_at:.1f}s after "
                    "a successful delete response; re-sending the delete")
                entry['confirmed_at'] = None
                entry['last_attempt'] = float('-inf')   # due again on the next _pump

    def _scan_for_leftover_models(self) -> None:
        """Deletes any terrain tile model already in the world at start-up -
        left over from a previous, uncleanly stopped run - through the same
        bounded delete path as everything else.

        Leftovers carry a *different* run id (or none at all, from an older
        build), so this run's own names can never be caught by the sweep,
        and no leftover is missed for being from a build that named its
        models differently - see LEFTOVER_MODEL_RE."""
        try:
            future = self._model_list.call_async(GetModelList.Request())
        except Exception as exc:                        # noqa: BLE001
            self.get_logger().warn(f"could not list existing models at start-up: {exc}")
            return
        future.add_done_callback(self._on_startup_model_list)

    def _on_startup_model_list(self, future) -> None:
        try:
            response = future.result()
            names = list(response.model_names)
        except Exception as exc:                        # noqa: BLE001
            self.get_logger().warn(f"listing existing models at start-up failed: {exc}")
            return
        # The list is a snapshot taken some time ago; this run may already
        # have spawned a tile of its own by now, and deleting that would be
        # a self-inflicted wound. Its run id tells it apart from every
        # leftover, which is one more thing the run id buys.
        leftovers = [name for name in names
                     if LEFTOVER_MODEL_RE.match(name)
                     and f'_{self._run_id}_' not in name]
        if leftovers:
            self.get_logger().warn(
                f"{len(leftovers)} leftover terrain tile model(s) from a previous "
                "run found in Gazebo at start-up; deleting them")
        for name in leftovers:
            self._delete_model(name, None)

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
