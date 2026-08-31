"""Turns the plan NaVi is driving into a cyan ribbon-and-pads mesh for Gazebo.

Pure numpy, no ROS - runs under the system python3 like terrain_mesh.py and
obstacle_mesh.py, for the same reason: this is the arithmetic a screenshot
does not check.

Geometry: each consecutive pair of *distinct* points in the path becomes a
flat quad `width` metres wide, centred on the segment, at constant `z`; a
repeated point (two consecutive path points that are the same, or close
enough that the segment's length is effectively zero) contributes no
segment rather than a degenerate, zero-length or NaN-normal one. Each
waypoint becomes its own `0.25 m` square pad at the same `z` - built as the
very same flat quad, just centred on a dummy horizontal segment of that
length, so a pad's winding and normal come from one piece of code instead
of a second, independently-verified one.

Convention - per-face vertices, not per-quad-shared: each quad's own 4
vertices are appended fresh (as obstacle_mesh.py does per cube face), so
every vertex carries an unambiguous face normal. The whole mesh is flat and
horizontal, so every normal is simply (0, 0, 1) - a flat-shaded top face,
lifted clear of the terrain, is the entire visual.

Winding: for a quad from `p0` to `p1` with half-width offset `hw`
(perpendicular to the segment, in the xy-plane), the emitted corner order
is `[p0 + hw, p0 - hw, p1 - hw, p1 + hw]`, triangulated as
`(0, 1, 2), (0, 2, 3)`. That order is counter-clockwise as seen from above
(+z out of the page) - matching Gazebo's back-face culling - which is why
the ribbon's own width test (a segment along +x) comes out with all of its
width on the y-axis and every normal pointing straight up.

`z = 0.15` by default: above the 0.14 m step the traversability layer (see
sub-project 3) calls lethal, so the ribbon can never be drawn buried inside
ground the map calls drivable.
"""

from dataclasses import dataclass

import numpy as np

DEFAULT_WIDTH = 0.08
DEFAULT_Z = 0.15
PAD_SIZE = 0.25
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
    faces: np.ndarray       # (F, 3) int64, indices into vertices, CCW from above


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


def path_mesh_from_points(points, waypoints, width: float = DEFAULT_WIDTH,
                          z: float = DEFAULT_Z):
    """A Mesh of the path ribbon plus one pad per waypoint, or None if
    neither produces any geometry (fewer than two usable path points and no
    waypoints).

    `points`/`waypoints` are iterables of (x, y) pairs, world frame, metres.
    """
    vertex_chunks = []
    face_chunks = []
    vcount = 0

    points = [(float(x), float(y)) for x, y in points]
    for p0, p1 in zip(points, points[1:]):
        quad = _flat_quad(p0, p1, width, z)
        if quad is None:
            continue
        vertices, faces = quad
        vertex_chunks.append(vertices)
        face_chunks.append(faces + vcount)
        vcount += vertices.shape[0]

    half_pad = PAD_SIZE / 2.0
    for x, y in waypoints:
        x, y = float(x), float(y)
        # A pad is a quad on a dummy horizontal segment of its own length,
        # centred on the waypoint - same corner order, same winding, same
        # code path as a ribbon segment.
        quad = _flat_quad((x - half_pad, y), (x + half_pad, y), PAD_SIZE, z)
        vertices, faces = quad          # length is PAD_SIZE > _MIN_SEGMENT_M always
        vertex_chunks.append(vertices)
        face_chunks.append(faces + vcount)
        vcount += vertices.shape[0]

    if not face_chunks:
        return None

    vertices = np.concatenate(vertex_chunks)
    faces = np.concatenate(face_chunks)
    normals = np.tile(np.array([0.0, 0.0, 1.0]), (vertices.shape[0], 1))
    return Mesh(vertices=vertices, normals=normals, faces=faces)


def obj_bytes(mesh: Mesh) -> bytes:
    """A Wavefront OBJ of `mesh`: vertices, normals, faces, and no material.

    The SDF supplies the material (see nav_path_sdf), so no mtllib - Gazebo
    would only warn that it cannot find one. Fixed-format numbers, so the
    same path always encodes to the same bytes and terrain_writer can tell
    a changed plan from a repeated one by comparing payloads.
    """
    lines = ['# navi nav path: the plan, drawn as a ribbon with waypoint pads']
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
