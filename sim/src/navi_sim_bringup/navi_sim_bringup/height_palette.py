"""The colour palette the terrain mesh is drawn with: one material per
10 cm of absolute elevation, so a dip or a hole reads as a colour change
in Gazebo rather than disappearing into one flat orange-brown mesh.

Gazebo Classic loads OBJ meshes through Assimp/Ogre, which honours a
material per face group (`usemtl`) but not per-vertex colours - the same
mechanism terrain_mesh.py's reverted colour-experiment branch used for
per-face ZED colour (see palette.py there). Here the source of colour is
height, not the camera.

Absolute map-frame z, not tile-relative or rover-relative: two adjacent
terrain tiles are meshed independently, and only an absolute band boundary
guarantees they agree at the shared seam - a tile-relative band would put
the same physical ground in different bands depending on which tile drew
it. The banding is cyclic (`band % len(RAMP)`) rather than keyed to a
scene-wide min/max, so it needs no second pass over the map and, more
importantly, is deterministic: identical elevation in always produces
identical material assignment out, whatever else is on screen.
terrain_writer compares OBJ payload bytes to decide whether a tile
actually changed, so a palette that depended on scene state would make it
respawn every tile forever even when nothing moved.
"""

import numpy as np

BAND_M = 0.10                                # metres of elevation per band
MTL_NAME = 'navi_height.mtl'

# 30 materials, dark (low ground) to light (high ground): deep blue-black,
# through blue, teal, green, olive, orange, tan, to near-white. Each rung's
# luminance (ITU-R BT.601: 0.299 R + 0.587 G + 0.114 B) is strictly greater
# than the one before, so the ramp reads as monotonically brightening; hue
# does most of the work of keeping neighbours apart (minimum adjacent RGB
# distance ~15/441) since 30 steps packed into one smooth gradient would
# otherwise make some neighbours nearly indistinguishable.
RAMP = [
    (6, 4, 36),
    (9, 10, 90),
    (11, 21, 111),
    (12, 33, 124),
    (13, 46, 133),
    (14, 63, 123),
    (14, 78, 117),
    (15, 93, 112),
    (16, 108, 110),
    (20, 121, 103),
    (23, 135, 92),
    (28, 150, 77),
    (33, 165, 59),
    (42, 178, 37),
    (75, 174, 39),
    (107, 172, 40),
    (137, 169, 42),
    (161, 171, 41),
    (184, 172, 42),
    (212, 170, 46),
    (246, 165, 50),
    (255, 169, 68),
    (254, 179, 91),
    (253, 189, 114),
    (251, 198, 137),
    (249, 208, 157),
    (248, 218, 176),
    (248, 227, 195),
    (249, 236, 215),
    (250, 245, 235),
]


def band_index(z) -> np.ndarray:
    """floor(z / BAND_M) as int64, vectorised over an array of elevations."""
    z = np.asarray(z, dtype=np.float64)
    return np.floor(z / BAND_M).astype(np.int64)


def material_name(band) -> str:
    """The cyclic material name for `band` - the ramp repeats every
    len(RAMP) * BAND_M metres. Python's `%` floors toward -inf for a
    positive modulus, exactly matching band_index's own floor, so a
    negative band cycles the same way a positive one does."""
    return f"h{int(band) % len(RAMP)}"


def mtl_bytes() -> bytes:
    """The whole ramp as a Wavefront MTL. Ambient a little under diffuse
    so relief still reads under Gazebo's flat lighting; a small specular
    highlight, matte ground."""
    lines = ['# navi height bands: one material per 10 cm of elevation, cyclic']
    for band, (r, g, b) in enumerate(RAMP):
        kd = (r / 255.0, g / 255.0, b / 255.0)
        lines.append(f"newmtl {material_name(band)}")
        lines.append(f"Ka {kd[0] * 0.8:.3f} {kd[1] * 0.8:.3f} {kd[2] * 0.8:.3f}")
        lines.append(f"Kd {kd[0]:.3f} {kd[1]:.3f} {kd[2]:.3f}")
        lines.append("Ks 0.020 0.020 0.020")
        lines.append("illum 2")
    return ('\n'.join(lines) + '\n').encode()


def faces_by_material(face_bands) -> list:
    """[(material_name, face_index_array), ...] grouped by band modulo the
    ramp length, in a deterministic (sorted-key) order. The index arrays
    partition the whole face set exactly once - every face index appears
    in exactly one group."""
    face_bands = np.asarray(face_bands)
    if face_bands.size == 0:
        return []
    keys = np.mod(face_bands, len(RAMP))
    order = np.argsort(keys, kind='stable')
    sorted_keys = keys[order]
    change = np.flatnonzero(np.diff(sorted_keys)) + 1
    starts = np.concatenate([[0], change])
    ends = np.concatenate([change, [len(order)]])
    out = []
    for s, e in zip(starts, ends):
        out.append((material_name(int(sorted_keys[s])), order[s:e]))
    return out
