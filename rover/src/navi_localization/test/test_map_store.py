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
