import numpy as np

from navi_sim_bringup.nav_path_mesh import (MODEL_NAME, nav_path_sdf, obj_bytes,
                                            path_mesh_from_points)


def test_a_two_point_path_is_one_quad_of_two_triangles():
    mesh = path_mesh_from_points([(0.0, 0.0), (1.0, 0.0)], waypoints=[])
    assert mesh.faces.shape == (2, 3)
    assert mesh.vertices.shape == (4, 3)


def test_the_ribbon_is_as_wide_as_asked_and_lifted_clear_of_the_terrain():
    mesh = path_mesh_from_points([(0.0, 0.0), (1.0, 0.0)], waypoints=[],
                                 width=0.08, z=0.15)
    assert abs(mesh.vertices[:, 1].max() - mesh.vertices[:, 1].min() - 0.08) < 1e-6
    assert np.allclose(mesh.vertices[:, 2], 0.15)


def test_every_waypoint_gets_a_pad():
    mesh = path_mesh_from_points([(0.0, 0.0), (1.0, 0.0)],
                                 waypoints=[(1.0, 0.0), (2.0, 0.0)])
    # One quad for the segment, one for each pad.
    assert mesh.faces.shape == (6, 3)


def test_a_path_of_fewer_than_two_points_and_no_waypoints_is_no_mesh():
    assert path_mesh_from_points([], waypoints=[]) is None
    assert path_mesh_from_points([(0.0, 0.0)], waypoints=[]) is None


def test_a_repeated_point_does_not_produce_a_zero_length_segment():
    mesh = path_mesh_from_points([(0.0, 0.0), (0.0, 0.0), (1.0, 0.0)],
                                 waypoints=[])
    assert mesh.faces.shape == (2, 3)
    assert np.isfinite(mesh.vertices).all()


def test_obj_bytes_round_trips_through_a_vertex_and_face_count():
    mesh = path_mesh_from_points([(0.0, 0.0), (1.0, 0.0)], waypoints=[])
    text = obj_bytes(mesh).decode()
    assert text.count("\nv ") == 4 and text.count("\nf ") == 2


def test_the_sdf_is_static_and_visual_only_like_every_other_drawn_model():
    # navi_terrain, not terrain_tiles: terrain_mesh.GAZEBO_MODEL_NAME is
    # 'navi_terrain', and that is the model dir every mesh URI here lives
    # under. The assertions never read the URI, but a fixture naming a
    # directory that does not exist misleads the next reader.
    sdf = nav_path_sdf("model://navi_terrain/meshes/plan_0_0_v00001.obj")
    assert "<static>true</static>" in sdf
    assert "<collision" not in sdf
    assert f'<model name="{MODEL_NAME}"' in sdf
