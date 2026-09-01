"""Turns the plan NaVi is driving into a cyan mesh for Gazebo: a ribbon
tracing the path plus one vertical beam standing at each waypoint.

Pure numpy, no ROS - runs under the system python3 like terrain_mesh.py and
obstacle_mesh.py, for the same reason: this is the arithmetic a screenshot
does not check.

Geometry: each consecutive pair of *distinct* points in the path becomes a
flat quad `width` metres wide, centred on the segment, at constant `z`; a
repeated point (two consecutive path points that are the same, or close
enough that the segment's length is effectively zero) contributes no
segment rather than a degenerate, zero-length or NaN-normal one.

Each waypoint becomes a vertical beam, not a horizontal pad: a thin
`WAYPOINT_BEAM_WIDTH` metre square column standing on the ground and
rising to `WAYPOINT_BEAM_HEIGHT`, built from its four vertical side faces
only, with no top or bottom cap. A beam of light has no lid, and a
horizontal face is exactly the shape that used to read as a ground step
the rover climbed onto as it arrived - the whole point of the beam is that
the rover drives straight through it instead.

Convention - per-face vertices, not per-quad-shared: each quad's or beam
face's own 4 vertices are appended fresh (as obstacle_mesh.py does per cube
face), so every vertex carries an unambiguous face normal. The ribbon's
quads are flat and horizontal, so their normal is simply (0, 0, 1) - a
flat-shaded top face, lifted clear of the terrain. A beam's four faces are
vertical planes instead, so each carries its own horizontal normal,
pointing straight out from the beam's axis; that is what makes a waypoint
and the ground look nothing alike in the render.

Winding: for a ribbon quad from `p0` to `p1` with half-width offset `hw`
(perpendicular to the segment, in the xy-plane), the emitted corner order
is `[p0 + hw, p0 - hw, p1 - hw, p1 + hw]`, triangulated as
`(0, 1, 2), (0, 2, 3)`. That order is counter-clockwise as seen from above
(+z out of the page) - matching Gazebo's back-face culling - which is why
the ribbon's own width test (a segment along +x) comes out with all of its
width on the y-axis and every normal pointing straight up. A beam face
uses the same `(0, 1, 2), (0, 2, 3)` triangulation on its own 4 corners
`[a, b, b_top, a_top]`, where `a` and `b` are adjacent cross-section
corners taken counter-clockwise from above; that ordering comes out
counter-clockwise as seen from outside the beam (looking in along the
outward normal), which is what back-face culling needs on a wall instead
of a roof.

`z = 0.30` by default: `traversability.STEP_LETHAL_M` (currently 0.25 m,
see sub-project 3) is the tallest step the traversability layer still
calls drivable, so the ribbon has to float above whatever that constant is
set to, or it can be drawn buried inside ground the map calls drivable -
exactly what this default exists to prevent. If that threshold moves
again, this default has to move with it. This module deliberately does
not import traversability.py to check that value at runtime: sim/ must
not depend on rover/, so the tie is enforced by a human reading both
comments, not by code.
"""

from dataclasses import dataclass

import numpy as np

DEFAULT_WIDTH = 0.08
DEFAULT_Z = 0.30
WAYPOINT_BEAM_WIDTH = 0.06   # metres, the beam's square cross-section edge
WAYPOINT_BEAM_HEIGHT = 1.5   # metres, ground to tip: taller than the 0.409 m
                             # rover so it is visible over it, short enough
                             # to stay in frame
MODEL_NAME = 'nav_plan'

# Below this segment length (metres) a pair of consecutive path points is
# treated as a repeat: not just an exact duplicate, but anything close
# enough that normalising its direction vector would divide by ~0 and hand
# back NaN/inf vertices.
_MIN_SEGMENT_M = 1e-9


# eq=False for the same reason as ObstacleMesh/TerrainMesh: a generated
# __eq__ would compare numpy arrays and raise on the result's truth value.
@dataclass(frozen=True, eq=False)
class Mesh:
    vertices: np.ndarray    # (V, 3) float64, world coordinates
    normals: np.ndarray     # (V, 3) float64, unit length, one per vertex
    faces: np.ndarray       # (F, 3) int64, indices into vertices, CCW from
                            # outside the solid (above, for the flat ribbon)


def _flat_quad(p0, p1, width: float, z: float):
    """The 4 vertices and 2 CCW-from-above triangles of a flat quad
    `width` metres wide, centred on the segment `p0` -> `p1`, at height
    `z`. `p0`/`p1` are 2-tuples (x, y). Returns (vertices, faces) with
    faces indexed from 0, or None if the segment is too short to give a
    direction (see _MIN_SEGMENT_M)."""
    p0 = np.asarray(p0, dtype=np.float64)
    p1 = np.asarray(p1, dtype=np.float64)
    direction = p1 - p0
    length = float(np.hypot(direction[0], direction[1]))
    if length < _MIN_SEGMENT_M:
        return None
    unit = direction / length
    half = np.array([-unit[1], unit[0]]) * (width / 2.0)
    corners_2d = (p0 + half, p0 - half, p1 - half, p1 + half)
    vertices = np.array([[x, y, z] for x, y in corners_2d], dtype=np.float64)
    faces = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int64)
    return vertices, faces


def _vertical_beam(x: float, y: float, width: float, height: float):
    """The 16 vertices, 16 per-vertex normals and 8 triangles of a vertical
    beam standing at `(x, y)`: a `width` metre square column with four
    vertical side faces and no top or bottom cap, from z = 0 to
    z = `height`.

    Unlike a flat quad, a beam has no single normal - each of its four
    faces is its own vertical plane, so the normal is computed once per
    face and repeated across that face's own 4 vertices. Returns
    (vertices, normals, faces) with faces indexed from 0.
    """
    half = width / 2.0
    # Counter-clockwise as seen from above, the same sense the ribbon's
    # own corners use.
    corners = [
        (x - half, y - half),
        (x + half, y - half),
        (x + half, y + half),
        (x - half, y + half),
    ]

    vertex_chunks = []
    normal_chunks = []
    face_chunks = []
    vcount = 0
    for i in range(4):
        ax, ay = corners[i]
        bx, by = corners[(i + 1) % 4]
        dx, dy = bx - ax, by - ay
        # The corners are wound CCW from above, so rotating the edge
        # direction -90 degrees (clockwise) in the xy-plane turns "along
        # the wall" into "away from the beam's axis" - the outward normal.
        normal = np.array([dy, -dx, 0.0])
        normal /= np.linalg.norm(normal)
        face_vertices = np.array([
            [ax, ay, 0.0],
            [bx, by, 0.0],
            [bx, by, height],
            [ax, ay, height],
        ], dtype=np.float64)
        vertex_chunks.append(face_vertices)
        normal_chunks.append(np.tile(normal, (4, 1)))
        face_chunks.append(np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int64) + vcount)
        vcount += 4

    vertices = np.concatenate(vertex_chunks)
    normals = np.concatenate(normal_chunks)
    faces = np.concatenate(face_chunks)
    return vertices, normals, faces


def path_mesh_from_points(points, waypoints, width: float = DEFAULT_WIDTH,
                          z: float = DEFAULT_Z):
    """A Mesh of the path ribbon plus one vertical beam per waypoint, or
    None if neither produces any geometry (fewer than two usable path
    points and no waypoints).

    `points`/`waypoints` are iterables of (x, y) pairs, world frame, metres.
    """
    vertex_chunks = []
    normal_chunks = []
    face_chunks = []
    vcount = 0

    points = [(float(x), float(y)) for x, y in points]
    for p0, p1 in zip(points, points[1:]):
        quad = _flat_quad(p0, p1, width, z)
        if quad is None:
            continue
        vertices, faces = quad
        vertex_chunks.append(vertices)
        normal_chunks.append(np.tile(np.array([0.0, 0.0, 1.0]), (vertices.shape[0], 1)))
        face_chunks.append(faces + vcount)
        vcount += vertices.shape[0]

    for x, y in waypoints:
        x, y = float(x), float(y)
        vertices, normals, faces = _vertical_beam(
            x, y, WAYPOINT_BEAM_WIDTH, WAYPOINT_BEAM_HEIGHT)
        vertex_chunks.append(vertices)
        normal_chunks.append(normals)
        face_chunks.append(faces + vcount)
        vcount += vertices.shape[0]

    if not face_chunks:
        return None

    vertices = np.concatenate(vertex_chunks)
    normals = np.concatenate(normal_chunks)
    faces = np.concatenate(face_chunks)
    return Mesh(vertices=vertices, normals=normals, faces=faces)


def obj_bytes(mesh: Mesh) -> bytes:
    """A Wavefront OBJ of `mesh`: vertices, normals, faces, and no material.

    The SDF supplies the material (see nav_path_sdf), so no mtllib - Gazebo
    would only warn that it cannot find one. Fixed-format numbers, so the
    same path always encodes to the same bytes and terrain_writer can tell
    a changed plan from a repeated one by comparing payloads.
    """
    lines = ['# navi nav path: the plan, drawn as a ribbon with waypoint beams']
    lines += [f'v {x:.4f} {y:.4f} {z:.4f}' for x, y, z in mesh.vertices]
    lines += [f'vn {x:.4f} {y:.4f} {z:.4f}' for x, y, z in mesh.normals]
    # OBJ indices count from one; each vertex uses its own (face) normal.
    lines += [f'f {a + 1}//{a + 1} {b + 1}//{b + 1} {c + 1}//{c + 1}'
              for a, b, c in mesh.faces]
    return ('\n'.join(lines) + '\n').encode()


def nav_path_sdf(mesh_uri: str, model_name: str = MODEL_NAME) -> str:
    """The SDF for one nav-path model.

    Visual only and static, exactly like terrain_sdf/obstacle_sdf: no
    collision, so the real or simulated rover's own physics/localisation is
    unaffected. Cyan, not the terrain's orange or the obstacle mesh's bone
    grey - a third colour is what makes the plan readable against both at
    a glance.
    """
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>{mesh_uri}</uri>
          </mesh>
        </geometry>
        <material>
          <ambient>0.10 0.55 0.70 1</ambient>
          <diffuse>0.20 0.80 0.95 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
