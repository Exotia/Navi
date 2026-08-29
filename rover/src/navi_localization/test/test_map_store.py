"""Saving and loading maps on disk. Same run command as test_tiles.py."""
import numpy as np
import pytest

from navi_localization.elevation_grid import GridState
from navi_localization.map_store import MapStore, MapStoreError


def state(value=1.0):
    elevation = np.full((4, 6), value, dtype=np.float32)
    elevation[0, 0] = np.nan
    return GridState(elevation=elevation, count=np.ones((4, 6), dtype=np.int32),
                     origin_ix=-3, origin_iy=7, resolution=0.05)


def test_save_then_list_then_load_round_trips(tmp_path):
    store = MapStore(str(tmp_path))
    assert store.list_names() == []
    path = store.save('yard-day1', state())
    assert path.endswith('yard-day1.npz')
    assert store.list_names() == ['yard-day1']

    loaded = store.load('yard-day1')
    assert np.array_equal(loaded.elevation, state().elevation, equal_nan=True)
    assert loaded.count.dtype == np.int32 and loaded.count.sum() == 24
    assert (loaded.origin_ix, loaded.origin_iy, loaded.resolution) == (-3, 7, 0.05)


def test_saving_over_an_existing_name_is_refused_unless_asked(tmp_path):
    store = MapStore(str(tmp_path))
    store.save('a', state(1.0))
    with pytest.raises(MapStoreError, match='exists'):
        store.save('a', state(2.0))
    store.save('a', state(2.0), overwrite=True)
    assert store.load('a').elevation[1, 1] == 2.0


@pytest.mark.parametrize('bad', ['', 'has space', 'a/b', '../x', 'ü', 'x' * 65])
def test_bad_names_are_refused(tmp_path, bad):
    with pytest.raises(MapStoreError):
        MapStore(str(tmp_path)).validate_name(bad)


def test_loading_a_missing_map_is_an_error_not_a_crash(tmp_path):
    with pytest.raises(MapStoreError, match='no map'):
        MapStore(str(tmp_path)).load('nope')


def test_the_directory_is_created_on_first_save(tmp_path):
    store = MapStore(str(tmp_path / 'deeper' / 'maps'))
    store.save('a', state())
    assert store.list_names() == ['a']


def test_the_file_records_when_it_was_saved(tmp_path):
    store = MapStore(str(tmp_path))
    store.save('a', state())
    with np.load(str(tmp_path / 'a.npz')) as data:
        assert str(data['saved_at']).startswith('20')


def test_a_save_that_blows_up_leaves_no_tmp_or_npz_behind(tmp_path, monkeypatch):
    import navi_localization.map_store as map_store_module

    def explode(*args, **kwargs):
        raise OSError('disk full')

    monkeypatch.setattr(map_store_module.np, 'savez_compressed', explode)
    store = MapStore(str(tmp_path))
    with pytest.raises(OSError, match='disk full'):
        store.save('a', state())
    assert list(tmp_path.iterdir()) == []


def test_a_truncated_file_is_a_map_store_error_not_a_zipfile_crash(tmp_path):
    store = MapStore(str(tmp_path))
    store.save('a', state())
    path = tmp_path / 'a.npz'
    whole = path.read_bytes()
    path.write_bytes(whole[:len(whole) // 2])
    with pytest.raises(MapStoreError, match='not a readable map file'):
        store.load('a')


def test_a_file_that_is_not_an_npz_at_all_is_a_map_store_error(tmp_path):
    store = MapStore(str(tmp_path))
    (tmp_path / 'a.npz').write_bytes(b'this is not a numpy archive')
    with pytest.raises(MapStoreError, match='not a readable map file'):
        store.load('a')


def test_an_archive_missing_a_key_is_named_not_a_key_error(tmp_path):
    store = MapStore(str(tmp_path))
    np.savez_compressed(str(tmp_path / 'a.npz'),
                        count=np.ones((4, 6), dtype=np.int32),
                        origin_ix=np.int64(0), origin_iy=np.int64(0),
                        resolution=np.float64(0.05))
    with pytest.raises(MapStoreError, match="elevation"):
        store.load('a')


def test_an_archive_whose_count_does_not_match_the_elevation_is_refused(tmp_path):
    store = MapStore(str(tmp_path))
    np.savez_compressed(str(tmp_path / 'a.npz'),
                        elevation=np.zeros((4, 6), dtype=np.float32),
                        count=np.ones((2, 3), dtype=np.int32),
                        origin_ix=np.int64(0), origin_iy=np.int64(0),
                        resolution=np.float64(0.05))
    with pytest.raises(MapStoreError, match='count'):
        store.load('a')
