"""Turns occupied obstacle voxels into a grey blocky mesh model for Gazebo.

Pure numpy, no ROS - runs under the system python3 like terrain_mesh.py, and
for the same reason: this is the arithmetic a screenshot does not check.

Each occupied voxel is a `size`-metre cube. A cube face is drawn only when
the neighbouring voxel in that direction is *absent* ("hidden-face
removal") - two touching voxels never draw the shared wall between them,
so a solid block of voxels renders as one hollow-shell blocky solid rather
than a pile of independent, mutually-occluding cubes. Faces are visual
only (see obstacle_sdf) and static, exactly like terrain_mesh's mesh:
Gazebo's mesh loader tolerates being spawned and deleted all day, unlike
its heightmap/terrain subsystem.

Convention - per-face vertices, not per-cube-shared: each emitted face
gets its own 4 vertices (repeated positions where two faces of the same
cube meet at an edge) so that every vertex carries exactly one, unambiguous
face normal - `mesh.normals` is one flat outward normal per vertex, matched
1:1 to `mesh.vertices` the same way terrain_mesh pairs its per-vertex
gradient normals to its vertices. A single isolated cube therefore has 12
triangles (6 faces x 2) over 24 vertices (6 faces x 4), not the 8 a
fully-shared-corner cube would use; sharing corners would force each
corner to average three different face normals into one direction, which
is wrong for a blocky, flat-shaded look. This is the documented
alternative the task allows.

Voxel indices are recovered from centres as `round(centre / size - 0.5)`
(the inverse of the rover's `(index + 0.5) * VOXEL` centre convention),
and membership is a vectorised lookup of bit-packed int64 keys (np.isin) - fast enough
for the <= 5k voxels a single obstacle tile carries.
"""

from dataclasses import dataclass

import numpy as np

DEFAULT_SIZE = 0.05
MODEL_NAME = 'obstacle'

# One flat outward normal and 4 corners (as +/-0.5 offsets of a unit cube)
# per face direction, in an order chosen so that
# cross(corners[1] - corners[0], corners[2] - corners[0]) == normal - i.e.
# winding is counter-clockwise as seen from outside the cube, matching
# Gazebo's back-face culling.
_FACES = {
    (1, 0, 0): (
        (1.0, 0.0, 0.0),
        ((0.5, -0.5, -0.5), (0.5, 0.5, -0.5), (0.5, 0.5, 0.5), (0.5, -0.5, 0.5)),
    ),
    (-1, 0, 0): (
        (-1.0, 0.0, 0.0),
        ((-0.5, -0.5, -0.5), (-0.5, -0.5, 0.5), (-0.5, 0.5, 0.5), (-0.5, 0.5, -0.5)),
    ),
    (0, 1, 0): (
        (0.0, 1.0, 0.0),
        ((-0.5, 0.5, -0.5), (-0.5, 0.5, 0.5), (0.5, 0.5, 0.5), (0.5, 0.5, -0.5)),
    ),
    (0, -1, 0): (
        (0.0, -1.0, 0.0),
        ((-0.5, -0.5, -0.5), (0.5, -0.5, -0.5), (0.5, -0.5, 0.5), (-0.5, -0.5, 0.5)),
    ),
    (0, 0, 1): (
        (0.0, 0.0, 1.0),
        ((-0.5, -0.5, 0.5), (0.5, -0.5, 0.5), (0.5, 0.5, 0.5), (-0.5, 0.5, 0.5)),
    ),
    (0, 0, -1): (
        (0.0, 0.0, -1.0),
        ((-0.5, -0.5, -0.5), (-0.5, 0.5, -0.5), (0.5, 0.5, -0.5), (0.5, -0.5, -0.5)),
    ),
}


# eq=False for the same reason as terrain_mesh.TerrainMesh: a generated
# __eq__ would compare numpy arrays and raise on the result's truth value.
@dataclass(frozen=True, eq=False)
class ObstacleMesh:
    vertices: np.ndarray    # (V, 3) float64, world coordinates
    normals: np.ndarray     # (V, 3) float64, unit length, one per vertex
    faces: np.ndarray       # (F, 3) int64, indices into vertices, CCW from outside
    voxel_count: int        # number of occupied voxels meshed


# Bit-packing for _pack: each axis gets _KEY_BITS bits of a plain int64,
# biased so a negative index still packs to a non-negative field. 20 bits
# covers indices of +/-500,000 - 25 km of 5 cm voxels either side of the
# origin, far past any tile this map will ever hold.
_KEY_BITS = 20
_KEY_BIAS = 1 << (_KEY_BITS - 1)


def _pack(indices: np.ndarray) -> np.ndarray:
    """A (..., 3) int array packed to one plain int64 key per row.

    Used for `np.isin` set-membership tests. A structured/void-record view
    would also give one opaque value per row, but numpy's `isin` sorts
    such records with a slow, comparison-based fallback; packing into a
    plain int64 keeps it on numpy's fast integer sort, which is what keeps
    5,000 voxels well under the 50 ms budget (a void-record version measured
    ~60 ms just in the six `np.isin` calls; this one is sub-millisecond).
    """
    biased = (indices + _KEY_BIAS).astype(np.int64)
    x, y, z = biased[..., 0], biased[..., 1], biased[..., 2]
    return (x << (2 * _KEY_BITS)) | (y << _KEY_BITS) | z


def obstacle_mesh_from_voxels(centres, size: float = DEFAULT_SIZE):
    """An ObstacleMesh of `size`-metre cubes at `centres`, or None if empty.

    `centres` is an (N, 3) array of voxel centre positions (world frame,
    metres) - the same convention `(index + 0.5) * VOXEL` the rover
    publishes. Faces between two occupied voxels are dropped (hidden-face
    removal); everything else is emitted with an outward normal.

    Vectorised per face direction (6 iterations, not one per voxel): for
    each direction, every voxel's neighbour key in that direction is tested
    for membership in the occupied set at once (`_pack` + `np.isin`), and
    the exposed voxels' 4 corner vertices are built by broadcasting rather
    than a per-voxel Python loop.
    """
    centres = np.asarray(centres, dtype=np.float64).reshape(-1, 3)
    if centres.shape[0] == 0:
        return None

    indices = np.round(centres / size - 0.5).astype(np.int64)
    voxels = np.unique(indices, axis=0)                    # (M, 3) sorted
    voxel_keys = _pack(voxels)
    voxel_centres = (voxels.astype(np.float64) + 0.5) * size  # (M, 3)

    vertices = []
    normals = []
    faces = []
    vcount = 0
    for (dx, dy, dz), (normal, corners) in _FACES.items():
        neighbour_keys = _pack(voxels + np.array([dx, dy, dz], dtype=np.int64))
        exposed = ~np.isin(neighbour_keys, voxel_keys)
        if not exposed.any():
            continue

        exposed_centres = voxel_centres[exposed]           # (K, 3)
        k = exposed_centres.shape[0]
        corners_arr = np.array(corners, dtype=np.float64) * size  # (4, 3)
        quad = exposed_centres[:, np.newaxis, :] + corners_arr[np.newaxis, :, :]
        vertices.append(quad.reshape(-1, 3))
        normals.append(np.tile(normal, (k * 4, 1)))

        start = vcount + np.arange(k) * 4
        faces.append(np.stack([start, start + 1, start + 2], axis=1))
        faces.append(np.stack([start, start + 2, start + 3], axis=1))
        vcount += k * 4

    if not faces:
        # Every voxel fully enclosed by its neighbours - no wall to show.
        return None

    return ObstacleMesh(
        vertices=np.concatenate(vertices),
        normals=np.concatenate(normals),
        faces=np.concatenate(faces).astype(np.int64),
        voxel_count=voxels.shape[0],
    )


def obj_bytes(mesh: ObstacleMesh) -> bytes:
    """A Wavefront OBJ of `mesh`: vertices, normals, faces, and no material.

    The SDF supplies the material (see obstacle_sdf), so no mtllib - Gazebo
    would only warn that it cannot find one. Fixed-format numbers, so the
    same sorted voxel set always encodes to the same bytes and
    terrain_writer can tell a changed tile from a repeated one by comparing
    payloads.
    """
    lines = ['# navi obstacle voxels: everything the ZED sees that is not ground']
    lines += [f'v {x:.4f} {y:.4f} {z:.4f}' for x, y, z in mesh.vertices]
    lines += [f'vn {x:.4f} {y:.4f} {z:.4f}' for x, y, z in mesh.normals]
    # OBJ indices count from one; each vertex uses its own (face) normal.
    lines += [f'f {a + 1}//{a + 1} {b + 1}//{b + 1} {c + 1}//{c + 1}'
              for a, b, c in mesh.faces]
    return ('\n'.join(lines) + '\n').encode()


def obstacle_sdf(mesh_uri: str, model_name: str = MODEL_NAME) -> str:
    """The SDF for one obstacle-voxel model.

    Visual only and static, exactly like terrain_sdf: no collision, so the
    real or simulated rover's own physics/localisation is unaffected. Grey,
    not orange - obstacles are not the mapped ground, and grey keeps them
    visually distinct from both the orange terrain and Gazebo's own grey
    background at a glance (the blocky geometry and drop shadow separate
    it from the background; the colour separates it from the terrain).
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
          <ambient>0.82 0.80 0.70 1</ambient>
          <diffuse>0.92 0.90 0.78 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
