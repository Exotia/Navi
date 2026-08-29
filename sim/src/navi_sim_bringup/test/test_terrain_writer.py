"""Reading the tile message back, and the rate/ordering policy on respawning.

  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/sim/src/navi_sim_bringup:$PYTHONPATH python3 -m pytest \
    sim/src/navi_sim_bringup/test/test_terrain_writer.py -q'
"""

import os

import numpy as np
import pytest
import rclpy
from grid_map_msgs.msg import GridMap
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from navi_sim_bringup.terrain_writer import (
    LAYER, LEFTOVER_MODEL_RE, TerrainWriter, TileRespawnPolicy,
    elevation_from_message, model_name, obstacle_centres_from_message,
    obstacle_key_from_frame_id, tile_index_of)


def message(grid_rows, grid_cols, values, resolution=0.10,
            center=(10.0, -5.0), layers=(LAYER,)):
    """A GridMap laid out the way elevation_mapper lays one out."""
    layer = Float32MultiArray()
    layer.layout.dim = [
        MultiArrayDimension(label='column_index', size=grid_cols,
                            stride=grid_rows * grid_cols),
        MultiArrayDimension(label='row_index', size=grid_rows, stride=grid_rows),
    ]
    layer.data = list(values)
    out = GridMap()
    out.header.frame_id = 'map'
    out.info.resolution = resolution
    out.info.length_x = grid_rows * resolution
    out.info.length_y = grid_cols * resolution
    out.info.pose.position.x, out.info.pose.position.y = center
    out.layers = list(layers)
    out.basic_layers = list(layers)
    out.data = [layer]
    return out


def test_the_message_reads_back_into_the_grid_it_was_made_from():
    # grid_map matrix (3 rows along -x, 2 columns along -y), column-major:
    # [ (0,0) (1,0) (2,0) (0,1) (1,1) (2,1) ]
    grid_map = message(3, 2, [10.0, 20.0, 30.0, 40.0, 50.0, 60.0])

    elevation, resolution, center_x, center_y = elevation_from_message(grid_map)

    assert elevation.shape == (2, 3)          # (rows along y, columns along x)
    assert resolution == pytest.approx(0.10)
    assert center_x == pytest.approx(10.0)
    assert center_y == pytest.approx(-5.0)
    # grid_map (0, 0) is the largest x and largest y, which is the last
    # column of the last row here.
    assert elevation[1, 2] == pytest.approx(10.0)
    assert elevation[0, 0] == pytest.approx(60.0)


def test_empty_cells_survive_as_nan():
    grid_map = message(1, 1, [float('nan')])

    assert np.isnan(elevation_from_message(grid_map)[0][0, 0])


def test_a_message_without_an_elevation_layer_is_refused():
    grid_map = message(1, 1, [1.0], layers=('traversability',))

    with pytest.raises(ValueError):
        elevation_from_message(grid_map)


def test_a_circular_buffer_message_is_refused_rather_than_read_wrongly():
    grid_map = message(2, 2, [1.0, 2.0, 3.0, 4.0])
    grid_map.outer_start_index = 1

    with pytest.raises(ValueError):
        elevation_from_message(grid_map)


def test_model_names_are_unique_per_generation():
    assert model_name(('terrain', 3, -2), 0, 'a1b2c3') == 'terrain_3_-2_a1b2c3_g0'
    assert model_name(('terrain', 3, -2), 1, 'a1b2c3') == 'terrain_3_-2_a1b2c3_g1'
    assert model_name(('terrain', 3, -2), 2, 'a1b2c3') == 'terrain_3_-2_a1b2c3_g2'


def test_obstacle_model_names_use_the_obst_kind():
    assert model_name(('obst', 3, -2), 0, 'a1b2c3') == 'obst_3_-2_a1b2c3_g0'
    assert (model_name(('obst', 3, -2), 0, 'a1b2c3')
            != model_name(('terrain', 3, -2), 0, 'a1b2c3'))


def test_model_names_from_different_runs_never_collide():
    # Generations restart at 0 on every process start, so the run id is the
    # only thing keeping a fresh spawn off a leftover of the same tile whose
    # delete has not confirmed yet.
    for generation in range(3):
        assert (model_name(('terrain', 3, -2), generation, 'a1b2c3')
                != model_name(('terrain', 3, -2), generation, 'd4e5f6'))


def test_the_leftover_sweep_matches_tile_models_from_any_run_and_any_build():
    assert LEFTOVER_MODEL_RE.match('terrain_3_-2_a1b2c3_g0')     # this build
    assert LEFTOVER_MODEL_RE.match('terrain_-4_11_ffffff_g137')
    assert LEFTOVER_MODEL_RE.match('terrain_3_-2_g0')            # before run ids
    assert LEFTOVER_MODEL_RE.match('terrain_3_-2_a')             # the a/b build
    assert LEFTOVER_MODEL_RE.match('terrain_3_-2_b')
    assert LEFTOVER_MODEL_RE.match('obst_3_-2_a1b2c3_g0')        # the obstacle kind
    assert LEFTOVER_MODEL_RE.match('obst_-4_11_ffffff_g137')
    assert not LEFTOVER_MODEL_RE.match('terrain_of_someone_else')
    assert not LEFTOVER_MODEL_RE.match('rover')
    assert not LEFTOVER_MODEL_RE.match('ground_plane')


def test_two_writers_never_name_the_same_tile_generation_alike(tmp_path):
    rclpy.init()
    first = TerrainWriter(model_dir=str(tmp_path / 'one'))
    second = TerrainWriter(model_dir=str(tmp_path / 'two'))
    try:
        assert first._run_id != second._run_id
        for generation in range(3):
            assert (model_name(('terrain', 0, 0), generation, first._run_id)
                    != model_name(('terrain', 0, 0), generation, second._run_id))
    finally:
        first.destroy_node()
        second.destroy_node()
        rclpy.shutdown()


def test_tile_index_of_matches_the_rovers_convention():
    assert tile_index_of(1.275, 1.275) == (0, 0)
    assert tile_index_of(2.5 * 4 + 1.275, 2.5 * -3 + 1.275) == (4, -3)


def test_a_new_tile_is_due_at_once():
    policy = TileRespawnPolicy()
    policy.offer((0, 0), b'a', now=0.0)
    assert policy.next_due(now=0.0) == [((0, 0), b'a')]


def test_an_unchanged_payload_is_not_due_again():
    policy = TileRespawnPolicy()
    policy.offer((0, 0), b'a', now=0.0)
    policy.started((0, 0))
    policy.finished((0, 0), b'a', now=0.0, ok=True)
    policy.offer((0, 0), b'a', now=5.0)
    assert policy.next_due(now=5.0) == []


def test_a_tile_replaces_at_most_once_per_second_and_the_newest_wins():
    policy = TileRespawnPolicy()
    policy.offer((0, 0), b'a', now=0.0)
    policy.started((0, 0))
    policy.finished((0, 0), b'a', now=0.0, ok=True)
    policy.offer((0, 0), b'b', now=0.3)
    policy.offer((0, 0), b'c', now=0.6)
    assert policy.next_due(now=0.9) == []
    assert policy.next_due(now=1.0) == [((0, 0), b'c')]


def test_at_most_four_spawns_in_flight():
    policy = TileRespawnPolicy()
    for i in range(6):
        policy.offer((i, 0), bytes([i]), now=0.0)
    due = policy.next_due(now=0.0)
    assert len(due) == 4
    for key, _ in due:
        policy.started(key)
    assert policy.next_due(now=0.0) == []
    policy.finished((0, 0), b'\x00', now=0.1, ok=True)
    assert len(policy.next_due(now=0.1)) == 1


def test_a_failed_spawn_keeps_the_tile_pending_for_a_retry():
    policy = TileRespawnPolicy()
    policy.offer((0, 0), b'a', now=0.0)
    policy.started((0, 0))
    policy.finished((0, 0), b'a', now=0.0, ok=False)
    assert policy.next_due(now=1.0) == [((0, 0), b'a')]


class FakeFuture:
    def __init__(self):
        self._callbacks = []
        self._result = None
        self._done = False

    def add_done_callback(self, callback):
        self._callbacks.append(callback)
        if self._done:                # already resolved before this was added
            callback(self)

    def result(self):
        return self._result

    def resolve(self, result):
        self._result = result
        self._done = True
        for callback in self._callbacks:
            callback(self)


class FakeService:
    def __init__(self):
        self.calls = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        future = FakeFuture()
        self.calls.append((request, future))
        return future


class Response:
    def __init__(self, success=True, status_message='', model_names=()):
        self.success = success
        self.status_message = status_message
        self.model_names = list(model_names)          # only used by /get_model_list


class FakeClock:
    """A settable stand-in for `TerrainWriter._clock_fn`.

    Lets a test control when a spawn/delete *completes* (`set`, read by
    `_on_spawned`/`_remove`) independently of when `_pump` is *dispatched*
    (the explicit `now=` passed to `_pump`) - the two are different clocks
    in production whenever Gazebo takes a while to answer.
    """

    def __init__(self, t: float = 0.0):
        self.t = t

    def __call__(self) -> float:
        return self.t

    def set(self, t: float) -> None:
        self.t = t


@pytest.fixture
def writer(tmp_path):
    rclpy.init()
    node = TerrainWriter(model_dir=str(tmp_path))
    node._spawn = FakeService()
    node._delete = FakeService()
    node._model_list = FakeService()
    node._clock_fn = FakeClock(0.0)
    # Every _pump() triggers a one-time start-up scan for leftover terrain_*
    # models (see test_leftover_terrain_models_..._below, which builds its
    # own node instead of using this fixture to control that explicitly).
    # Flush it here, as "nothing left over", so the rest of the tests below
    # don't have to deal with the noise of an extra unresolved
    # /get_model_list call sitting in node._model_list.calls forever.
    node._pump(now=0.0)
    node._model_list.calls[0][1].resolve(Response(model_names=[]))
    node._model_list.calls.clear()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def tile_name(writer, generation, key=(0, 0), kind='terrain'):
    """The model name `writer` gives `key`'s `generation`-th spawn.

    Not a literal: the name carries the writer's own random run id, which
    is what keeps a restarted node's spawns off a previous run's leftovers.
    """
    return model_name((kind,) + tuple(key), generation, writer._run_id)


def tile_message(key, value=1.0):
    from navi_sim_bringup.terrain_writer import TILE_SAMPLES, tile_center
    cx, cy = tile_center(*key)
    values = [value] * (TILE_SAMPLES * TILE_SAMPLES)
    return message(TILE_SAMPLES, TILE_SAMPLES, values, resolution=0.05, center=(cx, cy))


def obstacle_tile_message(ix, iy, centres=()):
    """A PointCloud2 laid out the way the rover's obstacle-tile publisher
    writes it: x/y/z float32 tightly packed, frame_id 'map|<ix>|<iy>' -
    the only place tile identity can travel for an empty (all-clear) tile."""
    centres = np.asarray(list(centres), dtype='<f4').reshape(-1, 3)
    out = PointCloud2()
    out.header.frame_id = f'map|{ix}|{iy}'
    out.height = 1
    out.width = centres.shape[0]
    out.is_bigendian = False
    out.is_dense = True
    out.point_step = 12
    out.row_step = out.point_step * out.width
    out.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    out.data = centres.tobytes()
    return out


def test_the_replacement_is_spawned_before_the_old_tile_is_deleted(writer):
    writer._on_tile(tile_message((0, 0), 1.0))
    writer._pump(now=0.0)
    request, future = writer._spawn.calls[0]
    assert request.name == tile_name(writer, 0)
    writer._clock_fn.set(0.0)
    future.resolve(Response())
    assert writer._delete.calls == []                    # nothing to delete yet

    writer._on_tile(tile_message((0, 0), 2.0))
    writer._pump(now=2.0)
    request, future = writer._spawn.calls[1]
    assert request.name == tile_name(writer, 1)
    assert writer._delete.calls == []                    # old one still standing
    writer._clock_fn.set(2.0)
    future.resolve(Response())
    assert writer._delete.calls == []                    # doomed, not dispatched yet
    assert tile_name(writer, 0) in writer._doomed

    writer._pump(now=2.1)                                 # dispatches the bounded delete
    assert [r.name for r, _ in writer._delete.calls] == [tile_name(writer, 0)]


def test_a_failed_spawn_leaves_the_old_tile_and_deletes_nothing(writer):
    writer._on_tile(tile_message((0, 0), 1.0))
    writer._pump(now=0.0)
    writer._clock_fn.set(0.0)
    writer._spawn.calls[0][1].resolve(Response())
    writer._on_tile(tile_message((0, 0), 2.0))
    writer._pump(now=2.0)
    writer._clock_fn.set(2.0)
    writer._spawn.calls[1][1].resolve(Response(False, 'nope'))
    assert writer._delete.calls == []
    assert writer._current[('terrain', 0, 0)] == tile_name(writer, 0)


def test_an_all_nan_tile_deletes_the_model(writer):
    writer._on_tile(tile_message((0, 0), 1.0))
    writer._pump(now=0.0)
    writer._clock_fn.set(0.0)
    writer._spawn.calls[0][1].resolve(Response())
    writer._on_tile(tile_message((0, 0), float('nan')))
    writer._clock_fn.set(2.0)
    writer._pump(now=2.0)
    assert writer._delete.calls == []                    # doomed, not dispatched yet
    assert tile_name(writer, 0) in writer._doomed
    assert len(writer._spawn.calls) == 1

    writer._pump(now=2.1)                                 # dispatches the bounded delete
    assert [r.name for r, _ in writer._delete.calls] == [tile_name(writer, 0)]
    assert len(writer._spawn.calls) == 1


def test_the_rate_cap_is_measured_from_when_gazebo_confirms_not_when_dispatched(writer):
    writer._on_tile(tile_message((0, 0), 1.0))
    writer._pump(now=0.0)                        # dispatched at t=0
    writer._clock_fn.set(2.0)                       # Gazebo was slow: confirms at t=2
    writer._spawn.calls[0][1].resolve(Response())

    writer._on_tile(tile_message((0, 0), 2.0))
    writer._pump(now=2.5)                        # only 0.5 s since completion
    assert len(writer._spawn.calls) == 1          # not due yet - eligible at t=3

    writer._pump(now=3.0)
    assert len(writer._spawn.calls) == 2
    assert writer._spawn.calls[1][0].name == tile_name(writer, 1)


def test_a_dispatch_error_does_not_strand_the_tile_in_flight(writer):
    class RaisingSpawnService(FakeService):
        def call_async(self, request):
            raise RuntimeError('boom')

    writer._spawn = RaisingSpawnService()
    writer._on_tile(tile_message((0, 0), 1.0))
    writer._pump(now=0.0)

    assert writer._policy._in_flight == set()
    due = writer._policy.next_due(now=1.0)
    assert len(due) == 1
    assert due[0][0] == ('terrain', 0, 0)


class CollidingSpawnService(FakeService):
    """Behaves like real Gazebo: refuses to spawn a name that is already
    alive. Regression guard for the a/b naming bug, where a later
    replacement could reuse an earlier generation's name while that
    generation's delete was still stuck, and Gazebo refused the spawn with
    "already exists".
    """

    def __init__(self):
        super().__init__()
        self.alive = set()

    def call_async(self, request):
        future = FakeFuture()
        self.calls.append((request, future))
        if request.name in self.alive:
            future.resolve(Response(False, f'Entity [{request.name}] already exists'))
        else:
            self.alive.add(request.name)
        return future


def test_repeated_replacements_never_collide_even_when_deletes_never_succeed(writer):
    writer._spawn = CollidingSpawnService()
    names = []
    for i, value in enumerate((1.0, 2.0, 3.0, 4.0)):
        writer._clock_fn.set(float(i))
        writer._on_tile(tile_message((0, 0), value))
        writer._pump(now=float(i))
        request, future = writer._spawn.calls[i]
        names.append(request.name)
        if future.result() is None:                  # not auto-refused
            future.resolve(Response())
        assert future.result().success                # never refused as already existing
        if writer._delete.calls:
            writer._delete.calls[-1][1].resolve(Response(False, 'nope'))  # delete never succeeds

    assert names == [tile_name(writer, 0), tile_name(writer, 1), tile_name(writer, 2), tile_name(writer, 3)]
    assert len(set(names)) == 4


def test_a_failed_delete_is_retried_after_the_interval_and_stays_tracked(writer):
    writer._on_tile(tile_message((0, 0), 1.0))
    writer._pump(now=0.0)
    writer._clock_fn.set(0.0)
    writer._spawn.calls[0][1].resolve(Response())

    writer._on_tile(tile_message((0, 0), 2.0))
    writer._pump(now=1.0)
    writer._clock_fn.set(1.0)
    writer._spawn.calls[1][1].resolve(Response())          # dooms generation 0
    assert writer._delete.calls == []                      # registered, not dispatched yet

    writer._pump(now=1.1)                                   # dispatches the first attempt
    assert len(writer._delete.calls) == 1
    writer._clock_fn.set(1.1)
    writer._delete.calls[0][1].resolve(Response(False, 'busy'))
    assert tile_name(writer, 0) in writer._doomed               # still tracked after failure

    writer._pump(now=1.6)                                    # only 0.5 s since the attempt
    assert len(writer._delete.calls) == 1                    # not retried yet

    writer._pump(now=2.1)                                     # 1.0 s since the attempt
    assert len(writer._delete.calls) == 2                     # retried
    assert writer._delete.calls[1][0].name == tile_name(writer, 0)
    assert tile_name(writer, 0) in writer._doomed


def test_a_delete_is_untracked_and_the_mesh_unlinked_once_the_model_list_confirms_absence(writer):
    writer._on_tile(tile_message((0, 0), 1.0))
    writer._pump(now=0.0)
    writer._clock_fn.set(0.0)
    writer._spawn.calls[0][1].resolve(Response())
    old_mesh = writer._mesh_file[('terrain', 0, 0)]
    old_mesh_path = os.path.join(writer._mesh_dir, old_mesh)
    assert os.path.exists(old_mesh_path)

    writer._on_tile(tile_message((0, 0), 2.0))
    writer._pump(now=1.0)
    writer._clock_fn.set(1.0)
    writer._spawn.calls[1][1].resolve(Response())          # dooms generation 0

    writer._pump(now=1.1)                                   # dispatches the delete + a poll
    assert len(writer._delete.calls) == 1
    assert len(writer._model_list.calls) == 1
    writer._clock_fn.set(1.1)
    writer._delete.calls[0][1].resolve(Response(True))       # "success" - not trusted alone
    assert tile_name(writer, 0) in writer._doomed
    assert os.path.exists(old_mesh_path)

    writer._model_list.calls[0][1].resolve(Response(model_names=[tile_name(writer, 1)]))  # g0 is gone

    assert tile_name(writer, 0) not in writer._doomed
    assert not os.path.exists(old_mesh_path)


def test_a_model_still_listed_two_seconds_after_a_successful_delete_is_re_sent(writer):
    writer._on_tile(tile_message((0, 0), 1.0))
    writer._pump(now=0.0)
    writer._clock_fn.set(0.0)
    writer._spawn.calls[0][1].resolve(Response())

    writer._on_tile(tile_message((0, 0), 2.0))
    writer._pump(now=1.0)
    writer._clock_fn.set(1.0)
    writer._spawn.calls[1][1].resolve(Response())            # dooms generation 0

    writer._pump(now=2.0)                                     # dispatches poll + first attempt
    still_there = Response(model_names=[tile_name(writer, 0), tile_name(writer, 1)])
    writer._model_list.calls[0][1].resolve(still_there)
    writer._clock_fn.set(2.0)
    writer._delete.calls[0][1].resolve(Response(True))        # "success" - not trusted alone
    assert tile_name(writer, 0) in writer._doomed
    assert writer._doomed[tile_name(writer, 0)]['confirmed_at'] == 2.0

    writer._pump(now=3.0)                                      # polls again (1 s since last poll)
    writer._clock_fn.set(3.0)
    writer._model_list.calls[1][1].resolve(still_there)
    assert tile_name(writer, 0) in writer._doomed                  # only 1 s since confirmed - not yet
    assert len(writer._delete.calls) == 1                      # no re-send yet

    writer._pump(now=4.0)                                       # polls again (2 s since confirmed)
    writer._clock_fn.set(4.0)
    writer._model_list.calls[2][1].resolve(still_there)
    assert writer._doomed[tile_name(writer, 0)]['confirmed_at'] is None    # reset, due again

    writer._pump(now=4.1)                                        # dispatches the re-sent delete
    assert len(writer._delete.calls) == 2
    assert writer._delete.calls[1][0].name == tile_name(writer, 0)
    assert tile_name(writer, 0) in writer._doomed                    # stays tracked throughout


def test_remove_with_a_failing_delete_keeps_retrying(writer):
    writer._on_tile(tile_message((0, 0), 1.0))
    writer._pump(now=0.0)
    writer._clock_fn.set(0.0)
    writer._spawn.calls[0][1].resolve(Response())

    writer._on_tile(tile_message((0, 0), float('nan')))
    writer._clock_fn.set(1.0)
    writer._pump(now=1.0)                                    # _remove registers the doomed model
    assert writer._delete.calls == []
    assert tile_name(writer, 0) in writer._doomed

    writer._pump(now=1.1)                                     # dispatches the first attempt
    assert len(writer._delete.calls) == 1
    writer._delete.calls[0][1].resolve(Response(False, 'nope'))
    assert tile_name(writer, 0) in writer._doomed

    writer._pump(now=1.6)
    assert len(writer._delete.calls) == 1                     # not due yet

    writer._pump(now=2.2)
    assert len(writer._delete.calls) == 2                     # retried
    assert writer._delete.calls[1][0].name == tile_name(writer, 0)


def test_giving_up_after_the_retry_cap_stops_tracking_and_retrying(writer):
    writer._max_delete_attempts = 2
    writer._on_tile(tile_message((0, 0), 1.0))
    writer._pump(now=0.0)
    writer._clock_fn.set(0.0)
    writer._spawn.calls[0][1].resolve(Response())

    writer._on_tile(tile_message((0, 0), 2.0))
    writer._pump(now=1.0)
    writer._clock_fn.set(1.0)
    writer._spawn.calls[1][1].resolve(Response())            # dooms generation 0

    writer._pump(now=1.1)                                     # attempt 1 dispatched
    assert tile_name(writer, 0) in writer._doomed
    writer._delete.calls[0][1].resolve(Response(False, 'nope'))   # attempt 1 fails (1 < cap 2)
    assert tile_name(writer, 0) in writer._doomed

    writer._clock_fn.set(2.2)
    writer._pump(now=2.2)                                     # attempt 2 dispatched
    writer._delete.calls[1][1].resolve(Response(False, 'nope'))   # attempt 2 fails (2 >= cap 2)

    assert tile_name(writer, 0) not in writer._doomed              # gave up

    writer._clock_fn.set(10.0)
    writer._pump(now=10.0)
    assert len(writer._delete.calls) == 2                       # no further retries


def test_no_more_than_four_factory_requests_are_outstanding_at_once_under_a_burst_of_ten_tiles(writer):
    """Round 3's regression guard: spawns and deletes now share one budget.

    Ten tiles are brought up to generation 0, then all ten are replaced at
    once - which both dispatches new spawns (bounded by TileRespawnPolicy's
    own cap) and, as each confirms, registers a delete of the superseded
    generation. Those deletes must compete for the *same* budget as spawns,
    not fire unboundedly - that unbounded firing was round 3's actual bug.
    """
    world = set()                              # models believed alive in the fake Gazebo

    def resolve_pending():
        for request, future in writer._spawn.calls:
            if future.result() is None:
                world.add(request.name)
                future.resolve(Response())
        for request, future in writer._delete.calls:
            if future.result() is None:
                world.discard(request.name)
                future.resolve(Response(True))
        for request, future in writer._model_list.calls:
            if future.result() is None:
                future.resolve(Response(model_names=sorted(world)))

    def assert_budget_ok():
        outstanding_spawns = sum(1 for _, f in writer._spawn.calls if f.result() is None)
        outstanding_deletes = sum(1 for _, f in writer._delete.calls if f.result() is None)
        assert outstanding_spawns + outstanding_deletes <= 4
        assert outstanding_spawns + outstanding_deletes == writer._factory_in_flight
        return outstanding_spawns, outstanding_deletes

    t = 0.0
    for i in range(10):
        writer._on_tile(tile_message((i, 0), float(i)))

    # Bring all ten tiles up to generation 0 first, respecting the 1 s/tile cap.
    for _ in range(30):
        writer._clock_fn.set(t)
        writer._pump(now=t)
        assert_budget_ok()
        resolve_pending()
        t += 1.1
        if len(writer._current) == 10:
            break
    assert len(writer._current) == 10
    assert writer._delete.calls == []

    # Replace all ten tiles at once - spawns and the deletes of the
    # superseded generation now compete for a single shared budget of 4.
    for i in range(10):
        writer._on_tile(tile_message((i, 0), float(i) + 100.0))

    for _ in range(80):
        writer._clock_fn.set(t)
        writer._pump(now=t)
        assert_budget_ok()
        resolve_pending()
        t += 1.1
        if all(writer._generation.get(('terrain', i, 0)) == 1 for i in range(10)) and not writer._doomed:
            break

    assert all(writer._generation.get(('terrain', i, 0)) == 1 for i in range(10))
    assert writer._doomed == {}


def test_deletes_and_spawns_share_one_factory_budget_not_one_each(writer):
    """Direct proof that a delete does not get its own separate budget on
    top of spawns' four - round 3's actual bug was exactly that: a delete
    fired the instant a spawn confirmed, uncapped, alongside spawns that
    were already independently capped at four.

    Three tiles are already doomed (as if their replacements just
    confirmed) and three more tiles are offered for the first time - six
    requests chasing one shared budget of four - all in a single `_pump`.
    """
    for i in range(3):
        writer._doomed[model_name(('terrain', i, 0), 0, writer._run_id)] = {
            'mesh_name': None, 'attempts': 0, 'last_attempt': float('-inf'),
            'in_flight': False, 'confirmed_at': None,
        }
    for i in range(3, 6):
        writer._on_tile(tile_message((i, 0), float(i)))

    writer._pump(now=0.0)

    dispatched_deletes = len(writer._delete.calls)
    dispatched_spawns = len(writer._spawn.calls)
    assert dispatched_deletes + dispatched_spawns == 4          # the shared budget, not 3 + 4
    assert dispatched_deletes == 3                               # all three doomed ones fit
    assert dispatched_spawns == 1                                 # only one slot left for spawns
    assert writer._factory_in_flight == 4


def test_leftover_terrain_models_from_a_previous_run_are_deleted_through_the_bounded_path(tmp_path):
    rclpy.init()
    node = TerrainWriter(model_dir=str(tmp_path))
    node._spawn = FakeService()
    node._delete = FakeService()
    node._model_list = FakeService()
    node._clock_fn = FakeClock(0.0)
    try:
        other_run = 'a1b2c3' if node._run_id != 'a1b2c3' else 'd4e5f6'
        leftovers = [
            f'terrain_1_2_{other_run}_g3',      # a previous run of this build
            'terrain_5_5_g0',                   # a build from before run ids
            'terrain_-2_4_b',                   # the original a/b build
        ]
        mine = model_name(('terrain', 0, 0), 0, node._run_id)

        node._pump(now=0.0)                    # triggers the start-up leftover scan
        assert len(node._model_list.calls) == 1
        node._model_list.calls[0][1].resolve(Response(
            model_names=leftovers + [mine, 'rover', 'ground_plane']))

        # Every leftover naming scheme is swept; this run's own tile is not,
        # nor is anything that is not a terrain tile.
        assert set(node._doomed) == set(leftovers)
        assert node._delete.calls == []             # registered, not dispatched yet

        node._pump(now=1.0)                          # dispatches the bounded deletes
        assert {r.name for r, _ in node._delete.calls} == set(leftovers)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_a_spawn_that_is_never_answered_frees_its_slot_and_is_retried(writer):
    """Round 2's live run saw factory requests that never came back at all.

    Their slots were only freed by a response, so four of them wedged the
    shared budget shut forever and the node silently stopped drawing.
    """
    writer._on_tile(tile_message((0, 0), 1.0))
    writer._pump(now=0.0)
    assert len(writer._spawn.calls) == 1
    assert writer._factory_in_flight == 1
    mesh_path = os.path.join(
        writer._mesh_dir, f'tile_0_0_v{writer._version:05d}.obj')
    assert os.path.exists(mesh_path)

    writer._pump(now=9.9)                       # still inside the timeout
    assert writer._factory_in_flight == 1
    assert len(writer._spawn.calls) == 1

    writer._clock_fn.set(10.0)
    writer._pump(now=10.0)                       # written off, slot freed
    assert writer._factory_in_flight == 0
    assert len(writer._spawn.calls) == 1
    assert writer._current == {}                  # nothing is on screen
    # The mesh stays until the model this spawn may or may not have created
    # is known to be gone - see
    # test_a_written_off_spawn_that_never_reached_gazebo_costs_one_poll.
    assert os.path.exists(mesh_path)
    writer._model_list.calls[-1][1].resolve(Response(model_names=[]))
    assert not os.path.exists(mesh_path)

    writer._pump(now=11.0)                        # retried, once the 1 s/tile cap allows
    assert len(writer._spawn.calls) == 2
    # A new generation, never the old name: Gazebo may have created that
    # model without answering, and then it can never be spawned again.
    assert writer._spawn.calls[1][0].name == tile_name(writer, 1)
    assert writer._factory_in_flight == 1


def test_four_stalled_spawns_do_not_wedge_the_budget_forever(writer):
    for i in range(6):
        writer._on_tile(tile_message((i, 0), float(i)))

    writer._pump(now=0.0)
    assert len(writer._spawn.calls) == 4          # the whole budget, all stalled
    writer._pump(now=1.0)
    assert len(writer._spawn.calls) == 4          # and nothing else can move

    writer._clock_fn.set(10.0)
    writer._pump(now=10.0)                         # the four stalled ones are written off
    assert len(writer._spawn.calls) == 6           # the two tiles that never got a turn
    assert writer._factory_in_flight == 2
    # Their names are doomed rather than dropped, but the poll gets the
    # first word and finds Gazebo never created them - so they cost no
    # DeleteEntity call and free the budget again straight away.
    writer._model_list.calls[-1][1].resolve(Response(model_names=[]))
    assert writer._doomed == {}

    writer._pump(now=11.0)                          # and the written-off four come back
    assert len(writer._spawn.calls) == 8
    assert writer._delete.calls == []
    assert writer._factory_in_flight == 4


def test_a_delete_that_is_never_answered_is_re_attempted_after_the_timeout(writer):
    writer._on_tile(tile_message((0, 0), 1.0))
    writer._pump(now=0.0)
    writer._clock_fn.set(0.0)
    writer._spawn.calls[0][1].resolve(Response())

    writer._on_tile(tile_message((0, 0), 2.0))
    writer._pump(now=1.0)
    writer._clock_fn.set(1.0)
    writer._spawn.calls[1][1].resolve(Response())            # dooms generation 0

    writer._pump(now=1.1)                                     # attempt 1, never answered
    assert len(writer._delete.calls) == 1
    assert writer._factory_in_flight == 1

    writer._pump(now=5.0)
    assert len(writer._delete.calls) == 1                     # still waiting on it
    assert writer._doomed[tile_name(writer, 0)]['in_flight'] is True

    writer._clock_fn.set(11.1)
    writer._pump(now=11.1)                                     # written off and re-attempted
    assert len(writer._delete.calls) == 2
    assert writer._delete.calls[1][0].name == tile_name(writer, 0)
    assert writer._doomed[tile_name(writer, 0)]['attempts'] == 2
    assert writer._factory_in_flight == 1                       # the retry, not a leak


def test_a_late_answer_after_the_timeout_does_not_release_a_second_slot(writer):
    writer._on_tile(tile_message((0, 0), 1.0))
    writer._pump(now=0.0)
    stalled = writer._spawn.calls[0][1]

    writer._clock_fn.set(10.0)
    writer._pump(now=10.0)                        # written off
    writer._model_list.calls[-1][1].resolve(Response(model_names=[]))  # never existed
    writer._clock_fn.set(11.0)
    writer._pump(now=11.0)                        # and retried, taking a fresh slot
    assert writer._factory_in_flight == 1
    assert len(writer._spawn.calls) == 2

    stalled.resolve(Response())                    # the original answer, far too late

    assert writer._factory_in_flight == 1          # the retry still holds its slot
    assert writer._current == {}                   # and the late answer changed nothing
    assert writer._delete.calls == []

    writer._clock_fn.set(11.1)
    writer._spawn.calls[1][1].resolve(Response())   # the retry lands normally
    assert writer._factory_in_flight == 0
    assert writer._current[('terrain', 0, 0)] == tile_name(writer, 1)


def test_a_late_delete_answer_after_the_timeout_does_not_double_release(writer):
    writer._doomed[tile_name(writer, 0)] = {
        'mesh_name': None, 'attempts': 0, 'last_attempt': float('-inf'),
        'in_flight': False, 'confirmed_at': None,
    }
    writer._pump(now=0.0)
    stalled = writer._delete.calls[0][1]
    assert writer._factory_in_flight == 1

    writer._clock_fn.set(10.0)
    writer._pump(now=10.0)                         # written off, re-attempted at once
    assert len(writer._delete.calls) == 2
    assert writer._factory_in_flight == 1

    stalled.resolve(Response(True))                 # the first answer, far too late

    assert writer._factory_in_flight == 1           # the re-attempt keeps its slot
    assert writer._doomed[tile_name(writer, 0)]['confirmed_at'] is None


def test_a_model_list_poll_that_is_never_answered_is_re_issued(writer):
    """The poll is the only ground truth for whether a delete happened.

    Its in-flight flag was cleared by the answer alone, so one poll Gazebo
    never came back from disabled delete verification for the rest of the
    run - silently, since every other part of the node kept working.
    """
    writer._doomed[tile_name(writer, 0)] = {
        'mesh_name': None, 'attempts': 0, 'last_attempt': float('-inf'),
        'in_flight': False, 'confirmed_at': None,
    }
    writer._pump(now=0.0)
    assert len(writer._model_list.calls) == 1
    stalled = writer._model_list.calls[0][1]

    writer._pump(now=5.0)
    assert len(writer._model_list.calls) == 1              # still waiting on it

    writer._clock_fn.set(10.0)
    writer._pump(now=10.0)                                  # written off, polled again
    assert len(writer._model_list.calls) == 2

    stalled.resolve(Response(model_names=[]))               # far too late
    assert tile_name(writer, 0) in writer._doomed           # the late answer decided nothing

    writer._model_list.calls[1][1].resolve(Response(model_names=[]))
    assert tile_name(writer, 0) not in writer._doomed       # the live poll still works


def test_a_written_off_spawn_is_retried_under_a_new_name(writer):
    """Gazebo may have spawned the model and simply not answered.

    Retrying the same name then fails forever with "already exists", so the
    generation is burned at dispatch and the unanswered name is doomed
    rather than forgotten - if it never existed, the model-list poll
    untracks it for free.
    """
    writer._on_tile(tile_message((0, 0), 1.0))
    writer._pump(now=0.0)
    first = writer._spawn.calls[0][0].name
    assert first == tile_name(writer, 0)

    writer._clock_fn.set(10.0)
    writer._pump(now=10.0)                                  # written off
    assert first in writer._doomed
    assert writer._current == {}

    writer._clock_fn.set(11.0)
    writer._pump(now=11.0)                                   # retried
    assert len(writer._spawn.calls) == 2
    assert writer._spawn.calls[1][0].name == tile_name(writer, 1)
    assert writer._spawn.calls[1][0].name != first


def test_a_written_off_spawn_that_never_reached_gazebo_costs_one_poll(writer):
    writer._on_tile(tile_message((0, 0), 1.0))
    writer._pump(now=0.0)
    mesh_path = os.path.join(
        writer._mesh_dir, f'tile_0_0_v{writer._version:05d}.obj')

    writer._clock_fn.set(10.0)
    writer._pump(now=10.0)
    assert os.path.exists(mesh_path)          # kept until the model is known gone

    writer._model_list.calls[-1][1].resolve(Response(model_names=[]))
    assert writer._doomed == {}                # never existed, nothing to delete
    assert writer._delete.calls == []
    assert not os.path.exists(mesh_path)


def test_giving_up_on_a_delete_still_unlinks_its_mesh(writer):
    writer._on_tile(tile_message((0, 0), 1.0))
    writer._pump(now=0.0)
    writer._clock_fn.set(0.0)
    writer._spawn.calls[0][1].resolve(Response())
    mesh_path = os.path.join(writer._mesh_dir, writer._mesh_file[('terrain', 0, 0)])
    assert os.path.exists(mesh_path)

    writer._on_tile(tile_message((0, 0), float('nan')))     # the tile goes away
    writer._clock_fn.set(2.0)
    writer._pump(now=2.0)
    doomed = tile_name(writer, 0)
    assert doomed in writer._doomed

    now = 2.0
    while doomed in writer._doomed:
        now += 1.0
        writer._clock_fn.set(now)
        writer._pump(now=now)
        for _, future in writer._delete.calls:
            if not future._done:
                future.resolve(Response(False, 'nope'))
    # Given up on: the model stays in Gazebo, but its mesh file must not
    # stay on disk forever - nothing will ever load it again.
    assert not os.path.exists(mesh_path)


def test_a_tile_message_with_no_layout_is_logged_not_raised(writer):
    broken = GridMap()
    broken.layers = [LAYER]
    broken.basic_layers = [LAYER]
    broken.data = [Float32MultiArray()]                     # no dimensions at all

    writer._on_tile(broken)                                  # IndexError, not ValueError

    assert writer._policy._pending == {}


def test_obstacle_key_from_frame_id_parses_map_ix_iy():
    assert obstacle_key_from_frame_id('map|3|-2') == ('obst', 3, -2)


def test_obstacle_key_from_frame_id_refuses_anything_else():
    with pytest.raises(ValueError):
        obstacle_key_from_frame_id('map')
    with pytest.raises(ValueError):
        obstacle_key_from_frame_id('odom|3|-2')
    with pytest.raises(ValueError):
        obstacle_key_from_frame_id('map|three|-2')


def test_obstacle_centres_from_message_reads_xyz_back():
    centres = obstacle_centres_from_message(
        obstacle_tile_message(0, 0, [(0.025, 0.075, 0.125), (1.0, 2.0, 3.0)]))
    assert centres.shape == (2, 3)
    assert centres[0] == pytest.approx([0.025, 0.075, 0.125])
    assert centres[1] == pytest.approx([1.0, 2.0, 3.0])


def test_obstacle_centres_from_message_is_empty_for_an_empty_tile():
    centres = obstacle_centres_from_message(obstacle_tile_message(0, 0, []))
    assert centres.shape == (0, 3)


def test_obstacle_centres_from_message_refuses_the_wrong_point_step():
    broken = obstacle_tile_message(0, 0, [(0.025, 0.025, 0.025)])
    broken.point_step = 16
    with pytest.raises(ValueError):
        obstacle_centres_from_message(broken)


def test_obstacle_centres_from_message_refuses_the_wrong_fields():
    broken = obstacle_tile_message(0, 0, [(0.025, 0.025, 0.025)])
    broken.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT64, count=1)]
    with pytest.raises(ValueError):
        obstacle_centres_from_message(broken)


def test_an_obstacle_tile_spawns_a_grey_obst_model_with_its_mesh_file(writer):
    writer._on_obstacle_tile(obstacle_tile_message(0, 0, [(0.025, 0.025, 0.025)]))
    writer._pump(now=0.0)

    assert len(writer._spawn.calls) == 1
    request, future = writer._spawn.calls[0]
    assert request.name == tile_name(writer, 0, kind='obst')
    assert '0.82 0.80 0.70' in request.xml          # obstacle_sdf's grey material

    writer._clock_fn.set(0.0)
    future.resolve(Response())

    assert writer._current[('obst', 0, 0)] == tile_name(writer, 0, kind='obst')
    mesh_name = writer._mesh_file[('obst', 0, 0)]
    assert mesh_name.startswith('obst_0_0_v')
    assert os.path.exists(os.path.join(writer._mesh_dir, mesh_name))


def test_an_empty_obstacle_tile_removes_the_model(writer):
    writer._on_obstacle_tile(obstacle_tile_message(0, 0, [(0.025, 0.025, 0.025)]))
    writer._pump(now=0.0)
    writer._clock_fn.set(0.0)
    writer._spawn.calls[0][1].resolve(Response())
    assert ('obst', 0, 0) in writer._current

    writer._on_obstacle_tile(obstacle_tile_message(0, 0, []))
    writer._clock_fn.set(1.0)
    writer._pump(now=1.0)

    assert ('obst', 0, 0) not in writer._current
    assert tile_name(writer, 0, kind='obst') in writer._doomed


def test_terrain_and_obstacle_tiles_at_the_same_index_are_independent_models(writer):
    writer._on_tile(tile_message((0, 0), 1.0))
    writer._on_obstacle_tile(obstacle_tile_message(0, 0, [(0.025, 0.025, 0.025)]))
    writer._pump(now=0.0)

    names = {request.name for request, _ in writer._spawn.calls}
    assert names == {tile_name(writer, 0, kind='terrain'), tile_name(writer, 0, kind='obst')}

    writer._clock_fn.set(0.0)
    for _, future in writer._spawn.calls:
        future.resolve(Response())

    assert writer._current[('terrain', 0, 0)] == tile_name(writer, 0, kind='terrain')
    assert writer._current[('obst', 0, 0)] == tile_name(writer, 0, kind='obst')


def test_the_startup_sweep_dooms_leftover_obst_models_from_another_run(tmp_path):
    rclpy.init()
    node = TerrainWriter(model_dir=str(tmp_path))
    node._spawn = FakeService()
    node._delete = FakeService()
    node._model_list = FakeService()
    node._clock_fn = FakeClock(0.0)
    try:
        other_run = 'a1b2c3' if node._run_id != 'a1b2c3' else 'd4e5f6'
        leftover = f'obst_2_3_{other_run}_g0'

        node._pump(now=0.0)                    # triggers the start-up leftover scan
        node._model_list.calls[0][1].resolve(
            Response(model_names=[leftover, 'rover', 'ground_plane']))

        assert leftover in node._doomed
        assert node._delete.calls == []             # registered, not dispatched yet

        node._pump(now=1.0)                          # dispatches the bounded delete
        assert {r.name for r, _ in node._delete.calls} == {leftover}
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_a_malformed_obstacle_cloud_is_logged_not_raised(writer):
    broken = obstacle_tile_message(0, 0, [(0.025, 0.025, 0.025)])
    broken.point_step = 16                      # wrong layout, refused

    writer._on_obstacle_tile(broken)             # must not raise

    assert writer._policy._pending == {}
    assert writer._spawn.calls == []


def test_an_obstacle_cloud_with_a_bad_frame_id_is_logged_not_raised(writer):
    broken = obstacle_tile_message(0, 0, [])
    broken.header.frame_id = 'not_the_right_format'

    writer._on_obstacle_tile(broken)             # must not raise

    assert writer._policy._pending == {}


def test_the_shared_factory_budget_counts_both_kinds(writer):
    for i in range(3):
        writer._on_tile(tile_message((i, 0), float(i)))
    for i in range(3):
        writer._on_obstacle_tile(obstacle_tile_message(i, 1, [(0.025, 0.025, 0.025)]))

    writer._pump(now=0.0)

    assert writer._factory_in_flight == 4
    assert len(writer._spawn.calls) == 4
