import numpy as np

from navi_sim_bringup.nav_path_mesh import (DEFAULT_Z, MODEL_NAME, WAYPOINT_BEAM_HEIGHT,
                                            WAYPOINT_BEAM_WIDTH, nav_path_sdf, obj_bytes,
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


def test_the_ribbon_still_lies_flat_with_every_normal_pointing_straight_up():
    mesh = path_mesh_from_points([(0.0, 0.0), (1.0, 0.0)], waypoints=[])
    assert np.allclose(mesh.vertices[:, 2], DEFAULT_Z)
    assert np.allclose(mesh.normals, np.array([0.0, 0.0, 1.0]))


def test_default_z_is_above_the_traversability_layers_lethal_step_of_0_25m():
    # traversability.STEP_LETHAL_M == 0.25: a ribbon at or below that is a
    # ribbon that can be drawn buried inside ground the map still calls
    # drivable, which is exactly what DEFAULT_Z exists to rule out.
    assert DEFAULT_Z > 0.25


def test_every_waypoint_gets_a_beam():
    mesh = path_mesh_from_points([(0.0, 0.0), (1.0, 0.0)],
                                 waypoints=[(1.0, 0.0), (2.0, 0.0)])
    # One quad (2 triangles) for the segment, 4 side faces (8 triangles
    # each) for each of the two beams.
    assert mesh.faces.shape == (2 + 2 * 4 * 2, 3)


def test_two_waypoints_produce_two_beams_and_the_vertex_count_scales():
    one = path_mesh_from_points([], waypoints=[(1.0, 0.0)])
    two = path_mesh_from_points([], waypoints=[(1.0, 0.0), (2.0, 0.0)])
    assert one.vertices.shape[0] == 16
    assert two.vertices.shape[0] == 32
    assert two.faces.shape[0] == 2 * one.faces.shape[0]


def test_a_beams_vertices_span_exactly_the_ground_to_the_beam_height():
    mesh = path_mesh_from_points([], waypoints=[(3.0, -2.0)])
    assert np.isclose(mesh.vertices[:, 2].min(), 0.0)
    assert np.isclose(mesh.vertices[:, 2].max(), WAYPOINT_BEAM_HEIGHT)


def test_a_beam_has_no_horizontal_face_to_be_mistaken_for_ground():
    mesh = path_mesh_from_points([], waypoints=[(0.0, 0.0)])
    # A horizontal face has a normal along z; a vertical wall's normal has
    # no z component at all. Checking every triangle's own normal (via its
    # vertices, since each face's 3 corners share one normal) is the direct
    # geometric statement that a beam can never look like ground.
    for a, b, c in mesh.faces:
        normal = mesh.normals[a]
        assert np.allclose(normal, mesh.normals[b])
        assert np.allclose(normal, mesh.normals[c])
        assert abs(normal[2]) < 1e-9


def test_every_beam_face_normal_points_away_from_the_beams_axis():
    x, y = 5.0, -1.0
    mesh = path_mesh_from_points([], waypoints=[(x, y)])
    axis = np.array([x, y])
    # A beam's own 4 side faces are laid down 4 vertices at a time (see
    # _vertical_beam), so grouping by fours gives each face's own corners -
    # a triangle's centroid would skew toward whichever of its 3 corners
    # dominate, which is not the same thing as "outward from the beam".
    for face_start in range(0, mesh.vertices.shape[0], 4):
        face_vertices = mesh.vertices[face_start:face_start + 4]
        normal = mesh.normals[face_start]
        centroid_xy = face_vertices[:, :2].mean(axis=0)
        outward = centroid_xy - axis
        outward /= np.linalg.norm(outward)
        # The face's own normal and the direction from the beam's axis to
        # its centroid should agree: that is what "outward" means, and
        # what Gazebo's back-face culling needs to render a wall at all.
        assert np.dot(normal[:2], outward) > 0.99


def test_a_beam_is_centred_on_its_waypoint_and_its_cross_section_is_beam_width_square():
    x, y = 4.0, 7.0
    mesh = path_mesh_from_points([], waypoints=[(x, y)])
    xy = mesh.vertices[:, :2]
    assert np.isclose((xy[:, 0].max() + xy[:, 0].min()) / 2.0, x)
    assert np.isclose((xy[:, 1].max() + xy[:, 1].min()) / 2.0, y)
    assert np.isclose(xy[:, 0].max() - xy[:, 0].min(), WAYPOINT_BEAM_WIDTH)
    assert np.isclose(xy[:, 1].max() - xy[:, 1].min(), WAYPOINT_BEAM_WIDTH)


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
