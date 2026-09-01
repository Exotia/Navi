"""The GridMap contract, both ways. The one test that stops the aggregator and
the rover's mapper drifting apart - the same job test_grid_map_round_trip.py
does for the simulation.

  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/rover/src/navi_autonomy:$PWD/rover/src/navi_localization \
    python3 -m pytest rover/src/navi_autonomy/test/test_grid_map_io.py -q'
"""
import numpy as np
import pytest
from builtin_interfaces.msg import Time

from navi_autonomy.grid_map_io import (
    AGE_LAYER, ELEVATION_LAYER, build_grid_map, build_occupancy_grid, layer_from_message,
    tile_from_message)
from navi_localization.elevation_mapper import build_tile_message


def a_tile():
    tile = np.full((51, 51), np.nan, dtype=np.float32)
    tile[0, 0] = 1.0
    tile[0, 1] = 2.0
    tile[1, 0] = 4.0
    tile[25, 25] = 3.0
    tile[50, 50] = 6.0
    return tile


def test_what_the_rover_publishes_is_what_the_aggregator_reads():
    tile = a_tile()
    message = build_tile_message((3, -2), tile, 'map', Time())
    got, age, ix, iy = tile_from_message(message)
    assert np.array_equal(got, tile, equal_nan=True)
    assert age is None                    # no age_s layer was ever given
    assert (ix, iy) == (3, -2)


def test_tile_from_message_reads_back_the_age_layer_when_present():
    elevation = a_tile()
    age = np.full((51, 51), np.nan, dtype=np.float32)
    age[25, 25] = 12.5
    message = build_tile_message((0, 0), elevation, 'map', Time(), age=age)

    got_elevation, got_age, ix, iy = tile_from_message(message)
    assert np.array_equal(got_elevation, elevation, equal_nan=True)
    assert np.array_equal(got_age, age, equal_nan=True)
    assert (ix, iy) == (0, 0)


def test_tile_from_message_refuses_an_age_layer_of_the_wrong_shape():
    # build_tile_message does not itself require age to match the tile's
    # shape - it just serialises whatever array it is given - so a message
    # with the two layers at different sizes is a real, well-formed
    # message this function still has to catch.
    message = build_tile_message((0, 0), a_tile(), 'map', Time(),
                                 age=np.zeros((3, 3), dtype=np.float32))
    with pytest.raises(ValueError, match='age_s'):
        tile_from_message(message)


def test_a_tile_at_a_foreign_resolution_is_refused_not_resampled():
    message = build_tile_message((0, 0), a_tile(), 'map', Time())
    message.info.resolution = 0.10
    with pytest.raises(ValueError, match='resolution'):
        tile_from_message(message)


def test_a_message_without_an_elevation_layer_is_refused():
    message = build_tile_message((0, 0), a_tile(), 'map', Time())
    message.layers = ['colour']
    with pytest.raises(ValueError, match='elevation'):
        tile_from_message(message)


def test_a_circular_buffer_message_is_refused():
    message = build_tile_message((0, 0), a_tile(), 'map', Time())
    message.outer_start_index = 3
    with pytest.raises(ValueError, match='circular'):
        tile_from_message(message)


def test_a_built_grid_map_reads_back_as_what_went_in():
    rows, cols = 6, 4                       # deliberately not square
    elevation = np.arange(rows * cols, dtype=np.float32).reshape(rows, cols)
    valid = np.ones((rows, cols), dtype=np.float32)
    message = build_grid_map({'elevation': elevation, 'valid': valid},
                             origin_ix=-2, origin_iy=5, resolution=0.05,
                             frame_id='map', stamp=Time())
    assert list(message.layers) == ['elevation', 'valid']
    assert message.header.frame_id == 'map'
    assert message.info.resolution == pytest.approx(0.05)
    assert message.info.length_x == pytest.approx(cols * 0.05)
    assert message.info.length_y == pytest.approx(rows * 0.05)
    assert message.info.pose.position.x == pytest.approx((-2 + cols / 2.0) * 0.05)
    assert message.info.pose.position.y == pytest.approx((5 + rows / 2.0) * 0.05)
    assert message.info.pose.orientation.w == pytest.approx(1.0)
    assert message.outer_start_index == 0 and message.inner_start_index == 0
    assert np.array_equal(layer_from_message(message, 'elevation'), elevation)
    assert np.array_equal(layer_from_message(message, 'valid'), valid)


def test_a_built_grid_map_carries_nan_through():
    elevation = np.full((4, 4), np.nan, dtype=np.float32)
    elevation[1, 2] = 7.5
    message = build_grid_map({'elevation': elevation}, 0, 0, 0.05, 'map', Time())
    assert np.array_equal(layer_from_message(message, 'elevation'), elevation,
                          equal_nan=True)


def test_the_occupancy_grid_origin_is_the_corner_and_x_runs_fastest():
    cost = np.array([[0, 1, 2], [3, 4, 5]], dtype=np.int8)   # 2 rows (y), 3 cols (x)
    message = build_occupancy_grid(cost, origin_ix=-10, origin_iy=4,
                                   resolution=0.05, frame_id='map', stamp=Time())
    assert (message.info.width, message.info.height) == (3, 2)
    assert message.info.resolution == pytest.approx(0.05)
    assert message.info.origin.position.x == pytest.approx(-0.5)
    assert message.info.origin.position.y == pytest.approx(0.2)
    assert message.info.origin.orientation.w == pytest.approx(1.0)
    assert list(message.data) == [0, 1, 2, 3, 4, 5]


def test_the_occupancy_grid_keeps_lethal_and_unknown_intact():
    cost = np.array([[-1, 100], [0, 99]], dtype=np.int8)
    message = build_occupancy_grid(cost, 0, 0, 0.05, 'map', Time())
    assert list(message.data) == [-1, 100, 0, 99]


def test_the_layer_name_asked_for_is_the_one_returned():
    message = build_grid_map({'a': np.zeros((2, 2), dtype=np.float32),
                              'b': np.ones((2, 2), dtype=np.float32)},
                             0, 0, 0.05, 'map', Time())
    assert layer_from_message(message, 'b')[0, 0] == pytest.approx(1.0)
    with pytest.raises(ValueError, match='c'):
        layer_from_message(message, 'c')


def test_the_elevation_layer_name_matches_the_mapper():
    assert ELEVATION_LAYER == 'elevation'
