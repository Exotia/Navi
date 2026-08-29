"""The grid-to-Gazebo encoding.

Run with the system python3 - this laptop's .venv has neither numpy nor
Pillow, and a ROS node runs under the system interpreter anyway:
  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/sim/src/navi_sim_bringup python3 -m pytest \
    sim/src/navi_sim_bringup/test/test_heightmap.py -q'
"""

import struct

import numpy as np
import pytest

from navi_sim_bringup.heightmap import (
    MAX_VALUE, MIN_SPAN, UNSEEN_DROP, heightmap_from_grid, model_config_xml,
    png_bytes, square_side, terrain_sdf)


@pytest.mark.parametrize("cells, expected", [(1, 129), (129, 129), (130, 257),
                                             (600, 1025)])
def test_the_side_is_the_next_power_of_two_plus_one(cells, expected):
    # Gazebo Classic refuses a heightmap image that is not square and
    # (2^n + 1) on a side, and 129 is the smallest size worth loading.
    assert square_side(cells) == expected


def test_the_image_is_square_two_to_the_n_plus_one_and_sixteen_bit():
    grid = np.array([[0.0, 1.0, 2.0]], dtype=np.float32)

    heightmap = heightmap_from_grid(grid, 0.10, 10.0, -5.0)

    side = heightmap.side
    assert heightmap.image.shape == (side, side)
    assert heightmap.image.dtype == np.uint16
    assert (side - 1) & (side - 2) == 0        # side - 1 is a power of two


def test_the_extent_follows_from_the_sample_spacing():
    heightmap = heightmap_from_grid(np.array([[0.0, 1.0, 2.0]]), 0.10, 10.0, -5.0)

    # 129 samples 0.10 m apart span 12.8 m, not 12.9.
    assert heightmap.size_x == pytest.approx(12.8)
    assert heightmap.size_y == pytest.approx(12.8)


def test_the_padding_is_centred_so_the_mapped_ground_lands_where_it_was_mapped():
    # Two columns in a 129-sample square: the padding cannot be symmetric,
    # and the offset has to come out in <pos>, not be rounded away.
    heightmap = heightmap_from_grid(np.array([[1.0, 2.0]]), 0.10, 10.0, -5.0)

    assert heightmap.pos_x == pytest.approx(10.05)
    assert heightmap.pos_y == pytest.approx(-5.0)


def test_row_zero_of_the_image_is_the_largest_y():
    # Gazebo's ImageHeightmap::FillHeightMap reads the image mirrored along
    # y, so the top row of the picture is the far side of the map. Get this
    # backwards and the terrain is a mirror image of the ground.
    grid = np.array([[0.0], [0.5], [1.0]])      # row 0 = smallest y

    image = heightmap_from_grid(grid, 0.10, 0.0, 0.0).image

    assert image[63, 64] == MAX_VALUE           # the y = max cell, at the top
    assert image[65, 64] < image[64, 64] < image[63, 64]


def test_unseen_cells_sit_below_the_lowest_ground_that_was_seen():
    grid = np.array([[1.0, np.nan]])

    heightmap = heightmap_from_grid(grid, 0.10, 0.0, 0.0)

    assert heightmap.image[64, 63] == MAX_VALUE  # the cell that was seen
    assert heightmap.image[64, 64] == 0          # the NaN cell beside it
    assert heightmap.image[0, 0] == 0            # and the padding around both
    # Everything unseen is UNSEEN_DROP below the lowest measurement, so it
    # disappears under the world's ground plane instead of z-fighting it.
    assert heightmap.pos_z == pytest.approx(1.0 - UNSEEN_DROP)


def test_a_perfectly_flat_map_still_has_a_positive_height_scale():
    heightmap = heightmap_from_grid(np.array([[2.0, 2.0]]), 0.10, 0.0, 0.0)

    assert heightmap.size_z == pytest.approx(MIN_SPAN)
    assert heightmap.pos_z == pytest.approx(2.0 - UNSEEN_DROP)


def test_a_map_with_nothing_in_it_produces_no_heightmap():
    assert heightmap_from_grid(np.full((3, 3), np.nan), 0.10, 0.0, 0.0) is None


def test_the_png_is_a_sixteen_bit_greyscale_image():
    heightmap = heightmap_from_grid(np.array([[0.0, 1.0]]), 0.10, 0.0, 0.0)

    data = png_bytes(heightmap.image)

    assert data[:8] == b'\x89PNG\r\n\x1a\n'
    width, height, depth, colour = struct.unpack('>IIBB', data[16:26])
    assert (width, height) == (heightmap.side, heightmap.side)
    assert depth == 16
    assert colour == 0          # greyscale, no palette, no alpha


def test_the_sdf_is_a_static_visual_only_heightmap():
    heightmap = heightmap_from_grid(np.array([[0.0, 1.0]]), 0.10, 0.0, 0.0)

    sdf = terrain_sdf('model://navi_terrain/materials/textures/heightmap_0007.png',
                      heightmap)

    assert '<model name="terrain">' in sdf
    assert '<static>true</static>' in sdf
    assert 'heightmap_0007.png' in sdf
    assert f'<size>{heightmap.size_x:.4f}' in sdf
    assert f'<pos>{heightmap.pos_x:.4f}' in sdf
    # No collision: the rover drives on the world's ground plane and its
    # pose comes from the real rover anyway. A 1025 x 1025 collision
    # heightmap would cost physics time for nothing.
    assert '<collision' not in sdf


def test_the_model_config_names_the_model_gazebo_will_look_up():
    assert '<name>navi_terrain</name>' in model_config_xml()
