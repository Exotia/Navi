"""The height-band palette: absolute-z banding, cyclic material names, and
the MTL/face-grouping machinery terrain_mesh.py builds a coloured OBJ from.

  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/sim/src/navi_sim_bringup:$PYTHONPATH python3 -m pytest \
    sim/src/navi_sim_bringup/test/test_height_palette.py -q'
"""

import re

import numpy as np
import pytest

from navi_sim_bringup.height_palette import (
    BAND_M, MTL_NAME, RAMP, band_index, faces_by_material, material_name,
    mtl_bytes)


def test_band_index_at_exact_boundaries():
    assert band_index(np.array([0.0]))[0] == 0
    assert band_index(np.array([0.099]))[0] == 0
    assert band_index(np.array([0.10]))[0] == 1
    assert band_index(np.array([-0.01]))[0] == -1     # floor, not truncation


def test_band_index_is_vectorised():
    z = np.array([0.0, 0.099, 0.10, 0.25, -0.01, -0.15])
    bands = band_index(z)
    assert bands.dtype == np.int64
    assert list(bands) == [0, 0, 1, 2, -1, -2]


def test_band_index_negative_values():
    assert band_index(np.array([-0.001]))[0] == -1
    assert band_index(np.array([-0.10]))[0] == -1
    assert band_index(np.array([-0.101]))[0] == -2


def test_material_name_cycles_at_the_ramp_length():
    assert material_name(0) == material_name(len(RAMP))
    assert material_name(1) == material_name(len(RAMP) + 1)
    assert material_name(0) == 'h0'
    assert material_name(len(RAMP) - 1) == f'h{len(RAMP) - 1}'


def test_material_name_matches_band_index_for_negatives():
    # band -1 must land on the same material as the last rung of the ramp,
    # the way band_index's floor and material_name's modulo agree.
    assert material_name(-1) == material_name(len(RAMP) - 1)
    assert material_name(-len(RAMP)) == material_name(0)


def test_mtl_bytes_has_one_newmtl_per_ramp_entry():
    mtl = mtl_bytes().decode()
    newmtl_lines = [line for line in mtl.splitlines() if line.startswith('newmtl ')]
    assert len(newmtl_lines) == len(RAMP) == 30
    names = {line.split()[1] for line in newmtl_lines}
    assert names == {material_name(band) for band in range(len(RAMP))}


def test_mtl_bytes_is_parseable():
    mtl = mtl_bytes().decode()
    # Every newmtl is followed by Ka, Kd, Ks and illum before the next
    # newmtl (or end of file) - a minimal Wavefront MTL parser's contract.
    blocks = re.split(r'(?=^newmtl )', mtl, flags=re.MULTILINE)
    blocks = [b for b in blocks if b.startswith('newmtl')]
    assert len(blocks) == len(RAMP)
    for block in blocks:
        assert re.search(r'^Ka [\d.]+ [\d.]+ [\d.]+$', block, re.MULTILINE)
        assert re.search(r'^Kd [\d.]+ [\d.]+ [\d.]+$', block, re.MULTILINE)
        assert re.search(r'^Ks [\d.]+ [\d.]+ [\d.]+$', block, re.MULTILINE)
        assert re.search(r'^illum \d+$', block, re.MULTILINE)
        # Ka at ~0.8 of Kd.
        ka = [float(v) for v in re.search(r'^Ka (.+)$', block, re.MULTILINE).group(1).split()]
        kd = [float(v) for v in re.search(r'^Kd (.+)$', block, re.MULTILINE).group(1).split()]
        for a, d in zip(ka, kd):
            assert a == pytest.approx(d * 0.8, abs=1e-3)


def test_mtl_bytes_is_deterministic():
    assert mtl_bytes() == mtl_bytes()


def test_mtl_name_is_the_expected_filename():
    assert MTL_NAME == 'navi_height.mtl'


def test_faces_by_material_groups_correctly():
    # Bands 0, 1, 0, len(RAMP)+1 (which cycles to the same material as 1).
    face_bands = np.array([0, 1, 0, len(RAMP) + 1])

    groups = dict(faces_by_material(face_bands))

    assert set(groups) == {material_name(0), material_name(1)}
    assert sorted(groups[material_name(0)].tolist()) == [0, 2]
    assert sorted(groups[material_name(1)].tolist()) == [1, 3]


def test_faces_by_material_partitions_every_face_exactly_once():
    rng = np.random.default_rng(0)
    n_faces = 500
    face_bands = rng.integers(-30, 30, size=n_faces)

    groups = faces_by_material(face_bands)

    all_indices = np.concatenate([idx for _, idx in groups])
    assert sorted(all_indices.tolist()) == list(range(n_faces))


def test_faces_by_material_is_deterministic_order():
    face_bands = np.array([5, 2, 8, 2, 5, 0])

    first = [name for name, _ in faces_by_material(face_bands)]
    second = [name for name, _ in faces_by_material(face_bands)]

    assert first == second
    assert first == sorted(first)   # sorted-key order


def test_faces_by_material_on_no_faces_is_empty():
    assert faces_by_material(np.array([], dtype=np.int64)) == []


def test_ramp_has_thirty_distinguishable_rgb_triples():
    assert len(RAMP) == 30
    assert len(set(RAMP)) == 30
    for triple in RAMP:
        assert len(triple) == 3
        assert all(0 <= channel <= 255 for channel in triple)


def _luminance(rgb):
    r, g, b = rgb
    return 0.299 * r + 0.587 * g + 0.114 * b


def test_ramp_luminance_increases_monotonically():
    luminances = [_luminance(rgb) for rgb in RAMP]
    assert luminances == sorted(luminances)
    assert len(set(luminances)) == len(luminances)   # strictly increasing


def test_band_m_is_ten_centimetres():
    assert BAND_M == pytest.approx(0.10)


def test_ramp_cycles_every_three_metres():
    assert len(RAMP) * BAND_M == pytest.approx(3.0)
