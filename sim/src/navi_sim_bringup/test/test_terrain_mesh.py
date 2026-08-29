"""The grid-to-Gazebo encoding.

Run with the system python3 - this laptop's .venv has no numpy, and a ROS
node runs under the system interpreter anyway:
  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/sim/src/navi_sim_bringup:$PYTHONPATH python3 -m pytest \
    sim/src/navi_sim_bringup/test/test_terrain_mesh.py -q'
"""

import numpy as np
import pytest

from navi_sim_bringup.terrain_mesh import (
    MAX_SIDE, model_config_xml, obj_bytes, terrain_mesh_from_grid, terrain_sdf)


def faces_of(obj: bytes):
    return [line.split()[1:] for line in obj.decode().splitlines()
            if line.startswith('f ')]


def vertices_of(obj: bytes):
    return [tuple(float(v) for v in line.split()[1:])
            for line in obj.decode().splitlines() if line.startswith('v ')]


def test_every_cell_becomes_a_vertex_at_its_mapped_position():
    grid = np.array([[0.0, 1.0, 2.0],       # two rows (y), three columns (x)
                     [0.0, 1.0, 2.0]])

    mesh = terrain_mesh_from_grid(grid, 0.10, 10.0, -5.0)

    assert mesh.vertices.shape == (6, 3)
    # Column 0 is the smallest x, the lattice is centred on (center_x, center_y).
    assert mesh.vertices[:3, 0] == pytest.approx([9.9, 10.0, 10.1])
    assert mesh.vertices[:3, 1] == pytest.approx([-5.05, -5.05, -5.05])
    assert mesh.vertices[:3, 2] == pytest.approx([0.0, 1.0, 2.0])


def test_row_zero_is_the_smallest_y():
    grid = np.array([[0.0, 0.0], [0.5, 0.5], [1.0, 1.0]])  # row 0 = smallest y

    mesh = terrain_mesh_from_grid(grid, 0.10, 0.0, 0.0)

    assert mesh.vertices[::2, 1] == pytest.approx([-0.1, 0.0, 0.1])
    assert mesh.vertices[::2, 2] == pytest.approx([0.0, 0.5, 1.0])


def test_only_ground_that_was_seen_gets_triangles():
    # A 2 x 3 grid has two cells of four corners each. The right cell has a
    # NaN corner, so it must not be drawn: unseen ground is not a slope down
    # to nowhere, it is simply absent.
    grid = np.array([[0.0, 0.0, 0.0],
                     [0.0, 0.0, np.nan]])

    mesh = terrain_mesh_from_grid(grid, 0.10, 0.0, 0.0)

    assert len(mesh.faces) == 2                 # one cell, two triangles
    assert len(faces_of(obj_bytes(mesh))) == 2


def test_triangles_wind_counter_clockwise_seen_from_above():
    # Gazebo culls back faces, so a clockwise terrain is invisible from the
    # chase camera above it.
    mesh = terrain_mesh_from_grid(np.zeros((2, 2)), 0.10, 0.0, 0.0)

    for face in mesh.faces:
        a, b, c = (mesh.vertices[i] for i in face)
        normal_z = np.cross(b - a, c - a)[2]
        assert normal_z > 0


def test_a_map_with_no_complete_cell_produces_no_mesh():
    assert terrain_mesh_from_grid(np.full((3, 3), np.nan), 0.10, 0.0, 0.0) is None
    # A single seen cell has no neighbours to make a triangle with either.
    lonely = np.full((3, 3), np.nan)
    lonely[1, 1] = 1.0
    assert terrain_mesh_from_grid(lonely, 0.10, 0.0, 0.0) is None


def test_a_large_map_is_subsampled_to_the_side_gazebo_can_afford():
    # Long and thin: the stride comes from the long side and thins both.
    grid = np.zeros((3 * MAX_SIDE, 4))

    mesh = terrain_mesh_from_grid(grid, 0.10, 0.0, 0.0)

    rows = len({round(v[1], 6) for v in vertices_of(obj_bytes(mesh))})
    assert rows <= MAX_SIDE
    assert mesh.cols == 2 and len(mesh.faces) == 2 * (mesh.rows - 1)
    # Subsampling keeps the extent: the first and last rows are still the
    # ends of the mapped ground, not MAX_SIDE cells from one end.
    assert mesh.vertices[:, 1].min() == pytest.approx(-(3 * MAX_SIDE - 1) / 2 * 0.10)
    assert mesh.vertices[:, 1].max() == pytest.approx((3 * MAX_SIDE - 1) / 2 * 0.10, abs=0.3)


def test_the_obj_has_one_normal_per_vertex_and_indexes_them_from_one():
    mesh = terrain_mesh_from_grid(np.array([[0.0, 1.0], [0.0, 1.0]]), 0.10, 0.0, 0.0)

    obj = obj_bytes(mesh).decode()

    lines = obj.splitlines()
    assert sum(line.startswith('v ') for line in lines) == 4
    assert sum(line.startswith('vn ') for line in lines) == 4
    assert 'mtllib' not in obj                    # the SDF supplies the material
    for face in faces_of(obj_bytes(mesh)):
        for corner in face:
            index, normal = corner.split('//')
            assert 1 <= int(index) <= 4 and index == normal


def test_the_same_grid_always_encodes_to_the_same_bytes():
    # terrain_writer compares payloads to decide whether the map changed.
    grid = np.array([[0.0, 0.25], [0.5, 0.75]])
    first = obj_bytes(terrain_mesh_from_grid(grid, 0.10, 1.0, 2.0))
    second = obj_bytes(terrain_mesh_from_grid(grid.copy(), 0.10, 1.0, 2.0))
    assert first == second


def test_the_sdf_is_a_static_visual_only_mesh():
    sdf = terrain_sdf('model://navi_terrain/meshes/terrain_0007.obj')

    assert '<model name="terrain">' in sdf
    assert '<static>true</static>' in sdf
    assert '<mesh>' in sdf and 'terrain_0007.obj' in sdf
    # Never a heightmap: Gazebo Classic's Ogre terrain cannot survive being
    # deleted and re-spawned while a camera renders it (gzserver dies with
    # "Zero sized texture surface on texture TerrBlend3"). Reproduced.
    assert '<heightmap>' not in sdf
    # No collision: the rover drives on the world's ground plane and its
    # pose comes from the real rover anyway.
    assert '<collision' not in sdf


def test_the_model_config_names_the_model_gazebo_will_look_up():
    assert '<name>navi_terrain</name>' in model_config_xml()


def test_an_isolated_needle_is_flattened_to_its_neighbourhood():
    # A single misread cell (a flying pixel) 1 m above flat ground: its 3x3
    # neighbourhood is 8 finite cells, all at ground level, so its median
    # is 0.0 and it gets pulled down.
    grid = np.zeros((5, 5))
    grid[2, 2] = 1.0

    mesh = terrain_mesh_from_grid(grid, 0.10, 0.0, 0.0)

    assert mesh.vertices[:, 2].max() == pytest.approx(0.0, abs=1e-9)


def test_a_wide_dome_is_not_flattened():
    # A 7x7 raised block in the middle of flat ground: interior cells of
    # the block have neighbours that are also part of the block, so their
    # median agrees with them and they are left alone. This is a real
    # feature (a rock, a mound), not a flying pixel.
    grid = np.zeros((15, 15))
    grid[4:11, 4:11] = 0.20

    mesh = terrain_mesh_from_grid(grid, 0.10, 0.0, 0.0)

    assert mesh.vertices[:, 2].max() == pytest.approx(0.20, abs=0.01)


def test_peak_threshold_m_controls_how_much_disagreement_is_tolerated():
    grid = np.zeros((5, 5))
    grid[2, 2] = 0.05                 # under the default 0.10 m threshold

    default = terrain_mesh_from_grid(grid, 0.10, 0.0, 0.0)
    assert default.vertices[:, 2].max() == pytest.approx(0.05)

    strict = terrain_mesh_from_grid(grid, 0.10, 0.0, 0.0, peak_threshold_m=0.01)
    assert strict.vertices[:, 2].max() == pytest.approx(0.0, abs=1e-9)


def test_a_nan_neighbourhood_does_not_create_fake_values():
    # Near a hole (fewer than 5 finite neighbours), the cell is left alone
    # rather than replaced with a median trusted on too little data.
    grid = np.full((5, 5), np.nan)
    grid[0, 0] = 1.0
    grid[0, 1] = 0.0
    grid[1, 0] = 0.0
    grid[1, 1] = 0.0

    mesh = terrain_mesh_from_grid(grid, 0.10, 0.0, 0.0)

    # The lone high corner (only 3 finite neighbours) survives unflattened,
    # and no NaN turned into a fabricated number.
    assert mesh.vertices[:, 2].max() == pytest.approx(1.0)
    assert np.isfinite(mesh.vertices[:, 2]).all()


def _stepped_grid(step_m: float, resolution: float, run_before: int = 4,
                   run_after: int = 4) -> np.ndarray:
    # A flat run, a sharp step of step_m, then another flat run - both
    # rows and columns repeated so every triangle on either side of the
    # step is a real, complete cell.
    rows = 4
    cols = run_before + run_after
    row = np.array([0.0] * run_before + [step_m] * run_after)
    return np.tile(row, (rows, 1))


def test_a_vertical_wall_face_is_dropped_but_the_ground_beside_it_stays():
    resolution = 0.10
    grid = _stepped_grid(step_m=1.2, resolution=resolution)   # a near-vertical step

    before = terrain_mesh_from_grid(grid, resolution, 0.0, 0.0, max_slope_deg=90.0)
    after = terrain_mesh_from_grid(grid, resolution, 0.0, 0.0)

    assert len(after.faces) < len(before.faces)
    assert len(after.faces) > 0
    # Ground away from the step - flat cells fully on one side - still
    # produces faces; only the near-vertical step triangles are gone.
    flat_vertex_indices = np.flatnonzero(after.vertices[:, 2] == 0.0)
    assert any(np.isin(face, flat_vertex_indices).all() for face in after.faces)


def test_a_45_degree_slope_keeps_every_face():
    resolution = 0.10
    rows, cols = 4, 6
    # Each column step rises by exactly `resolution`, a 45 degree slope.
    grid = np.tile(np.arange(cols) * resolution, (rows, 1))

    mesh = terrain_mesh_from_grid(grid, resolution, 0.0, 0.0)
    unfiltered = terrain_mesh_from_grid(grid, resolution, 0.0, 0.0, max_slope_deg=89.9)

    assert len(mesh.faces) == len(unfiltered.faces)


def test_max_slope_deg_controls_the_cutoff():
    resolution = 0.10
    grid = _stepped_grid(step_m=1.2, resolution=resolution)

    lenient = terrain_mesh_from_grid(grid, resolution, 0.0, 0.0, max_slope_deg=90.0)
    strict = terrain_mesh_from_grid(grid, resolution, 0.0, 0.0, max_slope_deg=10.0)

    assert len(strict.faces) < len(lenient.faces)
