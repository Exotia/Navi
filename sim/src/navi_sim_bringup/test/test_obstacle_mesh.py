"""Cube meshes with hidden faces removed, for the obstacle voxel tiles.

Run with the system python3, no rover source needed:
  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/sim/src/navi_sim_bringup:$PWD/rover/src/navi_localization:$PYTHONPATH \
    python3 -m pytest sim/src/navi_sim_bringup/test/test_obstacle_mesh.py -q'

Face counts below are triangles (`mesh.faces` rows), 2 per exposed cube
face, matching the `(F, 3)` triangle-index contract: a lone cube has 6
exposed faces (12 triangles); a solid 2x2x2 block of 8 voxels exposes
exactly 3 of each voxel's 6 faces (24 exposed faces, 48 triangles) since
every voxel in a 2-wide block has exactly 3 neighbours present and 3
absent; two separate cubes hide nothing between them (2 x 12 = 24
triangles); a 10x1x1 row exposes both y-faces and both z-faces of every
voxel (40 faces) plus the two x-end-caps (2 faces) = 42 exposed faces, 84
triangles.
"""

import time

import numpy as np
import pytest

from navi_sim_bringup.obstacle_mesh import (
    DEFAULT_SIZE, obj_bytes, obstacle_mesh_from_voxels, obstacle_sdf)


def centre_of(ix, iy, iz, size=DEFAULT_SIZE):
    return ((ix + 0.5) * size, (iy + 0.5) * size, (iz + 0.5) * size)


def block(nx, ny, nz, size=DEFAULT_SIZE):
    return np.array([centre_of(ix, iy, iz, size)
                      for ix in range(nx) for iy in range(ny) for iz in range(nz)])


def test_empty_input_is_none():
    assert obstacle_mesh_from_voxels(np.zeros((0, 3))) is None


def test_one_cube_has_twelve_triangles_over_six_faces():
    mesh = obstacle_mesh_from_voxels(block(1, 1, 1))

    assert mesh.faces.shape == (12, 3)
    assert mesh.vertices.shape == (24, 3)   # 6 faces x 4 vertices, per-face
    assert mesh.normals.shape == (24, 3)
    assert mesh.voxel_count == 1


def test_two_by_two_by_two_block_hides_the_shared_internal_faces():
    mesh = obstacle_mesh_from_voxels(block(2, 2, 2))

    # 8 voxels x 3 exposed faces each = 24 exposed faces = 48 triangles,
    # not the 48 exposed faces (96 triangles) eight independent cubes
    # would draw.
    assert mesh.faces.shape == (48, 3)
    assert mesh.voxel_count == 8


def test_two_separate_cubes_hide_nothing_between_them():
    centres = np.array([centre_of(0, 0, 0), centre_of(5, 5, 5)])

    mesh = obstacle_mesh_from_voxels(centres)

    assert mesh.faces.shape == (24, 3)   # 2 x 12, fully independent
    assert mesh.voxel_count == 2


def test_ten_by_one_by_one_wall_segment():
    mesh = obstacle_mesh_from_voxels(block(10, 1, 1))

    # Every voxel exposes both y-faces and both z-faces (40); only the two
    # end voxels expose an x-face (2). 42 exposed faces = 84 triangles.
    assert mesh.faces.shape == (84, 3)
    assert mesh.voxel_count == 10


def test_normals_point_outward_from_the_block_centre():
    mesh = obstacle_mesh_from_voxels(block(2, 2, 2))
    block_centre = mesh.vertices.mean(axis=0)

    for a, b, c in mesh.faces:
        centroid = mesh.vertices[[a, b, c]].mean(axis=0)
        normal = mesh.normals[a]
        outward = centroid - block_centre
        assert np.dot(normal, outward) > 0


def test_deterministic_for_the_same_sorted_input():
    centres = block(2, 2, 2)
    mesh_a = obstacle_mesh_from_voxels(centres)
    mesh_b = obstacle_mesh_from_voxels(centres[::-1].copy())

    assert obj_bytes(mesh_a) == obj_bytes(mesh_b)


def test_obj_bytes_has_no_material_reference():
    mesh = obstacle_mesh_from_voxels(block(1, 1, 1))
    obj = obj_bytes(mesh).decode()

    assert 'mtllib' not in obj
    assert 'usemtl' not in obj


def test_performance_five_thousand_voxels_under_50ms():
    centres = block(50, 20, 5)   # 5000 voxels
    assert centres.shape[0] == 5000

    start = time.perf_counter()
    mesh = obstacle_mesh_from_voxels(centres)
    elapsed = time.perf_counter() - start

    assert mesh.voxel_count == 5000
    assert elapsed < 0.05, f'{elapsed * 1000:.1f} ms'


def test_sdf_is_grey_static_and_visual_only():
    sdf = obstacle_sdf('model://obst_0_0_run1_g3/mesh.obj', 'obst_0_0_run1_g3')

    assert '<static>true</static>' in sdf
    assert '0.82 0.80 0.70' in sdf
    assert '0.92 0.90 0.78' in sdf
    assert '<collision' not in sdf
    assert '<heightmap' not in sdf
    assert 'model://obst_0_0_run1_g3/mesh.obj' in sdf
    assert 'name="obst_0_0_run1_g3"' in sdf
