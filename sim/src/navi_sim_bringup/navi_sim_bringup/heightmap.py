"""Turns an elevation grid into the two things Gazebo Classic needs for terrain.

Gazebo Classic cannot change a heightmap in place, and it will only load one
from a square greyscale image whose side is 2^n + 1 samples. So: pad the
grid into such a square, scale it into 16-bit greyscale, and wrap it in an
SDF model. terrain_writer.py respawns that model when the map changes.

Pure numpy and Pillow, no ROS - the arithmetic here is the part that is
wrong in ways a screenshot does not show. Runs under the system python3
(numpy 1.21.5, Pillow 9.0.1 on this laptop, verified to write a bit-depth-16
greyscale PNG that reads back byte for byte); the repository's .venv has
neither and never sees this file.

Two conventions to keep straight:

  * The grid comes in the way navi_localization stores it - row 0 at the
    smallest y, column 0 at the smallest x.
  * Gazebo reads a heightmap image mirrored along y
    (common::ImageHeightmap::FillHeightMap indexes the picture with
    `vertSize - y - 1`), so the *top* row of the image is the largest y.

Height scale: unseen cells go to pixel 0, which sits UNSEEN_DROP below the
lowest ground actually measured. That puts unmapped area just under the
world's ground plane at z = 0 rather than z-fighting with it, so what the
operator sees is exactly the ground the rover has seen, standing slightly
proud of a flat plate.
"""

import io
from dataclasses import dataclass

import numpy as np
from PIL import Image

MIN_SIDE = 129
MAX_VALUE = 65535
UNSEEN_DROP = 0.05
MIN_SPAN = 0.05
MODEL_NAME = 'terrain'
GAZEBO_MODEL_NAME = 'navi_terrain'


def square_side(cells: int) -> int:
    """The smallest 2^n + 1 that holds `cells` samples, never below 129."""
    side = MIN_SIDE
    while side < cells:
        side = (side - 1) * 2 + 1
    return side


# eq=False for the same reason as GridSnapshot: a generated __eq__ would
# compare numpy arrays and raise on the result's truth value.
@dataclass(frozen=True, eq=False)
class Heightmap:
    image: np.ndarray      # (side, side) uint16; row 0 = largest y, col 0 = smallest x
    size_x: float
    size_y: float
    size_z: float
    pos_x: float
    pos_y: float
    pos_z: float

    @property
    def side(self) -> int:
        return int(self.image.shape[0])


def heightmap_from_grid(elevation, resolution: float, center_x: float,
                        center_y: float):
    """A Heightmap for one elevation grid, or None if nothing was seen."""
    elevation = np.asarray(elevation, dtype=np.float64)
    rows, cols = elevation.shape
    if not np.isfinite(elevation).any():
        return None

    side = square_side(max(rows, cols))
    pad_row = (side - rows) // 2
    pad_col = (side - cols) // 2
    padded = np.full((side, side), np.nan)
    padded[pad_row:pad_row + rows, pad_col:pad_col + cols] = elevation

    z_min = float(np.nanmin(elevation))
    z_max = float(np.nanmax(elevation))
    base_z = z_min - UNSEEN_DROP
    size_z = max(z_max - base_z, MIN_SPAN)

    scaled = np.nan_to_num((padded - base_z) / size_z * MAX_VALUE, nan=0.0)
    image = np.flipud(np.clip(np.rint(scaled), 0, MAX_VALUE).astype(np.uint16))

    # <size> is the extent the samples span, and `side` samples 'resolution'
    # apart span (side - 1) * resolution, not side * resolution.
    extent = (side - 1) * resolution
    # <pos> is the centre of the sample lattice. The padding is only
    # symmetric when the difference is even, so the offset is computed
    # rather than assumed away.
    pos_x = center_x + ((side - 1) / 2.0 - pad_col - (cols - 1) / 2.0) * resolution
    pos_y = center_y + ((side - 1) / 2.0 - pad_row - (rows - 1) / 2.0) * resolution

    return Heightmap(image=image, size_x=extent, size_y=extent, size_z=size_z,
                     pos_x=pos_x, pos_y=pos_y, pos_z=base_z)


def png_bytes(image: np.ndarray) -> bytes:
    """A 16-bit greyscale PNG of `image`."""
    buffer = io.BytesIO()
    Image.fromarray(np.ascontiguousarray(image, dtype=np.uint16)).save(
        buffer, format='PNG')
    return buffer.getvalue()


def terrain_sdf(image_uri: str, heightmap: Heightmap,
                model_name: str = MODEL_NAME) -> str:
    """The SDF for one terrain model.

    Visual only and static: the rover drives on the world's ground plane and
    in semi-autonomous mode its pose comes from the real rover, so terrain
    collision would cost physics time (up to 1025 x 1025 samples) and buy
    nothing. The material is one flat grey on purpose - relief is the
    information, and a texture would suggest the rover has seen colour it
    has not.
    """
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <heightmap>
            <uri>{image_uri}</uri>
            <size>{heightmap.size_x:.4f} {heightmap.size_y:.4f} {heightmap.size_z:.4f}</size>
            <pos>{heightmap.pos_x:.4f} {heightmap.pos_y:.4f} {heightmap.pos_z:.4f}</pos>
          </heightmap>
        </geometry>
        <material>
          <!-- Orange-brown, not grey: the operator must tell the mapped
               ground from Gazebo's grey default background at a glance. -->
          <ambient>0.72 0.42 0.16 1</ambient>
          <diffuse>0.80 0.47 0.18 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
        </material>
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
