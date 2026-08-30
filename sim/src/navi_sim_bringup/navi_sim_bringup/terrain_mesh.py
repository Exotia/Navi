"""Turns an elevation grid into a mesh model Gazebo Classic can show as terrain.

A mesh, deliberately not a heightmap. This started as a heightmap - the
obvious SDF geometry for a height grid - and it killed the simulation:
Gazebo Classic renders a heightmap through Ogre's terrain subsystem, whose
blend textures do not survive the model being deleted and re-spawned while
a camera is rendering it. On the third respawn gzserver died with

    Ogre Error: OGRE EXCEPTION(3:RenderingAPIException): Zero sized texture
    surface on texture TerrBlend3 face 0 mipmap 0
    gzserver: OgreSharedPtr.h:253: Assertion `pRep' failed.

taking the chase camera and so the ground station's picture with it.
Reproduced on demand by changing the map every few seconds; a map that
never changes survives, which is why it looked fine in a static test. A
mesh visual goes through Gazebo's ordinary mesh loader, which is spawned
and deleted all day long by every model in every world, and carries no
such state. terrain_writer.py respawns that model when the map changes.

Pure numpy, no ROS - the arithmetic here is the part that is wrong in ways
a screenshot does not show. Runs under the system python3; the
repository's .venv has no numpy and never sees this file.

Conventions:

  * The grid comes in the way navi_localization stores it - row 0 at the
    smallest y, column 0 at the smallest x - centred on (center_x,
    center_y). Vertices are written in world coordinates, so the model
    sits at the world origin and needs no <pose>.
  * Only cells whose four corners were all seen are drawn. Unseen ground is
    absent, not a slope down to nowhere: what the operator sees is exactly
    the ground the rover has seen.
  * Triangles wind counter-clockwise seen from +z. Gazebo culls back faces,
    and the chase camera looks down.

Two cleanups run before the mesh reaches Gazebo, because the rover's
elevation grid is not clean ground:

  * Isolated-peak median: a flying pixel (a ZED depth outlier), a person or
    a rover wheel caught mid-frame, or a single misregistered cell all show
    up as a one-cell needle - height wildly different from its
    surroundings, with no support from its neighbours. A cell is pulled
    down to the median of its 3x3 neighbourhood only when it is truly
    isolated - not one neighbour (of the 8) agrees with it within
    `peak_threshold_m` - and that neighbourhood had enough finite cells to
    trust (>= 5 of 8). A genuine rock or ridge, even a 2-cell-wide one, has
    at least one neighbour that agrees with it, so it is left untouched.
    This runs on each tile at draw resolution (terrain_writer's
    `_draw_resolution` stride), not on the raw per-frame cloud, and per
    tile rather than per frame: a tile's outermost ring of samples (its
    shared halo boundary with the +x/+y neighbour tile, see
    navi_localization.tiles) is never replaced, so two tiles filtering the
    same physical cell from their own, differently-cropped neighbourhoods
    always agree - the median filter cannot re-open the seam the halo
    exists to close.
  * Slope cutoff: a wall, doorway or other near-vertical surface the ZED
    caught face-on turns into a near-vertical spike of triangles in the
    mesh - correct height data, wrong thing to draw as ground. Triangles
    steeper than `max_slope_deg` from horizontal are dropped after meshing;
    the flat ground beside the wall is unaffected because it is a separate
    triangle.

Together these are what "spikes in the terrain" turned out to be: not a
mesh bug, but real leaks from the rover's grid (flying pixels, dynamic
objects, vertical surfaces) that the rover's grid legitimately reports and
this module must not draw as ground.
"""

import warnings
from dataclasses import dataclass

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

from navi_sim_bringup import height_palette

# Vertices per side, at most. 257 x 257 is 66k vertices and 131k triangles,
# which Gazebo loads in well under a second; the elevation grid clips at
# 600 cells a side, which would be five times that. Subsampled by stride,
# so the extent is kept and only the detail is thinned.
MAX_SIDE = 257
MODEL_NAME = 'terrain'
GAZEBO_MODEL_NAME = 'navi_terrain'


# eq=False for the same reason as GridSnapshot: a generated __eq__ would
# compare numpy arrays and raise on the result's truth value.
@dataclass(frozen=True, eq=False)
class TerrainMesh:
    vertices: np.ndarray    # (n, 3) float64, world coordinates
    normals: np.ndarray     # (n, 3) float64, unit length
    faces: np.ndarray       # (m, 3) int64, indices into vertices, CCW from +z
    rows: int               # samples along y after subsampling
    cols: int               # samples along x after subsampling
    size_x: float           # extent of the sample lattice, metres
    size_y: float
    # (m,) int64, one per face: height_palette.band_index of the face's mean
    # z (the mean of its three vertices). None only for a TerrainMesh built
    # some other way than terrain_mesh_from_grid; that function always fills
    # it in.
    face_bands: np.ndarray = None


def _stride(cells: int) -> int:
    return max(1, -(-cells // MAX_SIDE))


def _flatten_isolated_peaks(elevation: np.ndarray, peak_threshold_m: float) -> np.ndarray:
    """Pulls truly isolated height outliers down to their neighbourhood median.

    NaN-aware median over each cell's 3x3 neighbourhood (the eight cells
    around it, not the cell itself), computed with a padded sliding window -
    no scipy. A cell is replaced only when it is *isolated*: not one of its
    up to 8 neighbours is within `peak_threshold_m` of the cell's own
    height, and at least 5 of those 8 neighbours are finite (so a cell near
    the edge of the mapped area, or next to a hole, is left alone rather
    than judged from a handful of neighbours). A genuine multi-cell
    feature - even a 2-cell-wide ridge - has at least one neighbour that
    agrees with it and so is never touched; only a needle, whose entire
    neighbourhood disagrees, gets pulled down.

    This is called once per tile (see terrain_mesh_from_grid), and a tile's
    outermost ring of samples - row/column 0 and the last row/column, the
    halo it shares with its +x/+y neighbour tile (navi_localization.tiles)
    - is never replaced. Two tiles filtering the same physical boundary
    cell see different, tile-cropped neighbourhoods (one has real data
    where the other has only the tile's own edge), so without this the
    same cell could come out flattened to two different heights depending
    on which tile computed it - reopening exactly the seam the halo exists
    to hide.
    """
    rows, cols = elevation.shape
    padded = np.pad(elevation, 1, constant_values=np.nan)
    windows = sliding_window_view(padded, (3, 3)).reshape(rows, cols, 9)
    neighbours = np.delete(windows, 4, axis=-1)          # drop the centre cell
    finite = np.isfinite(neighbours)
    count = finite.sum(axis=-1)
    own = elevation[..., np.newaxis]
    with np.errstate(invalid='ignore'), warnings.catch_warnings():
        # A window with zero finite neighbours (deep inside an unseen
        # region) makes nanmedian warn "All-NaN slice"; its NaN result is
        # exactly right and `replace` below never selects it. A NaN
        # neighbour's comparison against `own` is simply False, so it
        # never counts as agreement - no separate finite mask needed there.
        warnings.filterwarnings('ignore', message='All-NaN slice encountered')
        median = np.nanmedian(neighbours, axis=-1)
        agrees = np.abs(neighbours - own) <= peak_threshold_m
    isolated = ~agrees.any(axis=-1)

    interior = np.ones((rows, cols), dtype=bool)
    interior[[0, -1], :] = False
    interior[:, [0, -1]] = False

    replace = (np.isfinite(elevation) & np.isfinite(median)
               & (count >= 5) & isolated & interior)
    return np.where(replace, median, elevation)


def terrain_mesh_from_grid(elevation, resolution: float, center_x: float,
                           center_y: float, *, max_slope_deg: float = 60.0,
                           peak_threshold_m: float = 0.10):
    """A TerrainMesh for one elevation grid, or None if no cell is complete."""
    elevation = np.asarray(elevation, dtype=np.float64)
    elevation = _flatten_isolated_peaks(elevation, peak_threshold_m)
    full_rows, full_cols = elevation.shape
    stride = max(_stride(full_rows), _stride(full_cols))
    sampled = elevation[::stride, ::stride]
    rows, cols = sampled.shape

    # The sample (r, c) of the full grid sits at
    # center + (index - (n - 1) / 2) * resolution; the subsampled index
    # r * stride keeps that formula with the full grid's centre.
    xs = center_x + (np.arange(cols) * stride - (full_cols - 1) / 2.0) * resolution
    ys = center_y + (np.arange(rows) * stride - (full_rows - 1) / 2.0) * resolution
    grid_x, grid_y = np.meshgrid(xs, ys)          # (rows, cols), row = y
    seen = np.isfinite(sampled)
    heights = np.where(seen, sampled, 0.0)

    # A cell is drawn when all four corners were seen.
    complete = seen[:-1, :-1] & seen[:-1, 1:] & seen[1:, :-1] & seen[1:, 1:]
    if not complete.any():
        return None

    index = np.arange(rows * cols).reshape(rows, cols)
    v00 = index[:-1, :-1][complete]
    v10 = index[:-1, 1:][complete]
    v01 = index[1:, :-1][complete]
    v11 = index[1:, 1:][complete]
    # (x right, y up) with row = y: v00 -> v10 -> v11 is counter-clockwise
    # from above, as is v00 -> v11 -> v01.
    faces = np.concatenate([np.stack([v00, v10, v11], axis=1),
                            np.stack([v00, v11, v01], axis=1)])

    vertices = np.stack([grid_x, grid_y, heights], axis=-1).reshape(-1, 3)

    # Slope cutoff: a wall or other near-vertical surface the rover mapped
    # face-on comes out of the loop above as real triangles with real
    # height data - just not something to draw as ground. Drop any
    # triangle whose plane leans more than max_slope_deg from horizontal;
    # the ground beside it is a separate triangle and keeps its face.
    p0 = vertices[faces[:, 0]]
    p1 = vertices[faces[:, 1]]
    p2 = vertices[faces[:, 2]]
    face_normal = np.cross(p1 - p0, p2 - p0)
    face_normal_len = np.linalg.norm(face_normal, axis=1)
    with np.errstate(invalid='ignore', divide='ignore'):
        cos_from_vertical = face_normal[:, 2] / face_normal_len
    min_cos = np.cos(np.deg2rad(max_slope_deg))
    faces = faces[cos_from_vertical >= min_cos]
    faces = faces.astype(np.int64)

    # Height-band colouring: each face's own colour is the band of its mean
    # z (the mean of its three vertices, all at the same drawn resolution -
    # see height_palette for why banding is absolute-z and cyclic rather
    # than keyed to this tile's own min/max).
    face_z_mean = vertices[faces[:, 0], 2] + vertices[faces[:, 1], 2] + vertices[faces[:, 2], 2]
    face_z_mean /= 3.0
    face_bands = height_palette.band_index(face_z_mean)

    # Normals from the height gradient: n = (-dz/dx, -dz/dy, 1), unit
    # length. Unseen samples take height 0, which only affects normals of
    # vertices no drawn triangle uses.
    spacing = resolution * stride
    dz_dy, dz_dx = np.gradient(heights, spacing)
    normals = np.stack([-dz_dx, -dz_dy, np.ones_like(heights)], axis=-1)
    normals /= np.linalg.norm(normals, axis=-1, keepdims=True)

    return TerrainMesh(vertices=vertices, normals=normals.reshape(-1, 3),
                       faces=faces, rows=rows, cols=cols,
                       size_x=(cols - 1) * spacing, size_y=(rows - 1) * spacing,
                       face_bands=face_bands)


def obj_bytes(mesh: TerrainMesh, coloured: bool = True) -> bytes:
    """A Wavefront OBJ of `mesh`: vertices, normals, faces.

    `coloured=False` (or a mesh with no face_bands) reproduces the old,
    material-free OBJ byte-for-byte - the SDF then supplies one flat
    material, and no mtllib - Gazebo would only warn that it cannot find
    one. `coloured=True` instead emits `mtllib navi_height.mtl` and groups
    faces under `usemtl` per height band, so Gazebo draws each face in its
    own band's material and the SDF must leave its own material out (see
    terrain_sdf) or it would override every one of them.

    Either way, fixed-format numbers mean an unchanged grid encodes to the
    same bytes and terrain_writer can tell a changed map from a repeated
    one by comparing payloads.
    """
    lines = ['# navi terrain: the ground the rover has mapped']
    grouped = coloured and mesh.face_bands is not None
    if grouped:
        lines.append(f'mtllib {height_palette.MTL_NAME}')
    lines += [f'v {x:.4f} {y:.4f} {z:.4f}' for x, y, z in mesh.vertices]
    lines += [f'vn {x:.4f} {y:.4f} {z:.4f}' for x, y, z in mesh.normals]
    # OBJ indices count from one; each vertex uses its own normal.
    if grouped:
        for material, index in height_palette.faces_by_material(mesh.face_bands):
            lines.append(f'usemtl {material}')
            lines += [f'f {a + 1}//{a + 1} {b + 1}//{b + 1} {c + 1}//{c + 1}'
                      for a, b, c in mesh.faces[index]]
    else:
        lines += [f'f {a + 1}//{a + 1} {b + 1}//{b + 1} {c + 1}//{c + 1}'
                  for a, b, c in mesh.faces]
    return ('\n'.join(lines) + '\n').encode()


TERRAIN_FLAT_MATERIAL = """
        <material>
          <!-- Orange-brown, not grey: the operator must tell the mapped
               ground from Gazebo's grey default background at a glance. -->
          <ambient>0.72 0.42 0.16 1</ambient>
          <diffuse>0.80 0.47 0.18 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
        </material>"""


def terrain_sdf(mesh_uri: str, model_name: str = MODEL_NAME, coloured: bool = True) -> str:
    """The SDF for one terrain model.

    Visual only and static: the rover drives on the world's ground plane and
    in semi-autonomous mode its pose comes from the real rover, so terrain
    collision would cost physics time and buy nothing.

    `coloured=True` (the default) omits the `<material>` block: the mesh's
    own OBJ/MTL height-band materials (see obj_bytes) supply the colour,
    and an SDF `<material>` here would override every one of them with one
    flat colour. `coloured=False` keeps the old flat orange-brown material -
    relief was the only information available then, and a texture would
    have suggested the rover had seen colour it had not.
    """
    material = '' if coloured else TERRAIN_FLAT_MATERIAL
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
        </geometry>{material}
      </visual>
    </link>
  </model>
</sdf>
"""


def model_config_xml(name: str = GAZEBO_MODEL_NAME) -> str:
    """What makes ~/.gazebo/models/<name> resolvable as model://<name>."""
    return f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description>Terrain the rover has mapped. Rewritten by terrain_writer.</description>
</model>
"""
