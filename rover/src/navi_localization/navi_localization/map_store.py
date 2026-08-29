"""Saved maps on the rover: ~/navi_maps/<name>.npz.

Opt-in and one-shot: nothing is written unless the operator asks for a
save. A map file is the grid whole (mean, count, lattice origin,
resolution), compressed - the yard at 5 cm is under a megabyte.
"""

import os
import re
import zipfile
from datetime import datetime, timezone

import numpy as np

from navi_localization.elevation_grid import GridState

NAME_PATTERN = re.compile(r'^[A-Za-z0-9_-]{1,64}$')
DEFAULT_DIRECTORY = os.path.join(os.path.expanduser('~'), 'navi_maps')


class MapStoreError(ValueError):
    pass


class MapStore:

    def __init__(self, directory: str = DEFAULT_DIRECTORY):
        self.directory = directory

    def validate_name(self, name) -> None:
        if not isinstance(name, str) or not NAME_PATTERN.match(name):
            raise MapStoreError(
                f"map name {name!r} is not allowed: 1-64 of A-Z a-z 0-9 _ -")

    def _path(self, name: str) -> str:
        return os.path.join(self.directory, f"{name}.npz")

    def list_names(self) -> list:
        if not os.path.isdir(self.directory):
            return []
        return sorted(entry[:-4] for entry in os.listdir(self.directory)
                      if entry.endswith('.npz') and NAME_PATTERN.match(entry[:-4]))

    def save(self, name: str, state: GridState, overwrite: bool = False) -> str:
        self.validate_name(name)
        path = self._path(name)
        if os.path.exists(path) and not overwrite:
            raise MapStoreError(f"a map named {name!r} already exists")
        os.makedirs(self.directory, exist_ok=True)
        # Write beside, then rename: a save interrupted half way must not
        # leave a truncated file under the real name.
        temporary = path + '.tmp'
        written = False
        try:
            with open(temporary, 'wb') as handle:
                np.savez_compressed(
                    handle, elevation=state.elevation.astype(np.float32),
                    count=state.count.astype(np.int32),
                    origin_ix=np.int64(state.origin_ix), origin_iy=np.int64(state.origin_iy),
                    resolution=np.float64(state.resolution),
                    saved_at=np.str_(datetime.now(timezone.utc).isoformat(timespec='seconds')))
            written = True
        finally:
            # A save that blew up partway (disk full, bad data) must not
            # leave a half-written .tmp behind for the next save or list to
            # trip over.
            if not written and os.path.exists(temporary):
                os.remove(temporary)
        os.replace(temporary, path)
        return path

    def load(self, name: str) -> GridState:
        """The saved grid, or MapStoreError - never a raw numpy/zip error.

        A map file on the rover can be truncated (the Orin lost power
        mid-save on an older build), be something else entirely that got
        the .npz name, or come from a future format. np.load answers all
        of those with BadZipFile/OSError/KeyError/ValueError, and the
        caller is a ROS callback: anything that is not a MapStoreError
        escapes rclpy.spin and ends mapping for the whole run. So every
        failure to read is named and re-raised as one.
        """
        self.validate_name(name)
        path = self._path(name)
        if not os.path.exists(path):
            raise MapStoreError(f"no map named {name!r} in {self.directory}")
        try:
            with np.load(path) as data:
                missing = [key for key in
                           ('elevation', 'count', 'origin_ix', 'origin_iy', 'resolution')
                           if key not in data.files]
                if missing:
                    raise MapStoreError(
                        f"map {name!r} is missing {', '.join(missing)}; it was not "
                        "written by this node")
                elevation = data['elevation'].astype(np.float32)
                count = data['count'].astype(np.int32)
                origin_ix, origin_iy = int(data['origin_ix']), int(data['origin_iy'])
                resolution = float(data['resolution'])
        except MapStoreError:
            raise
        except (zipfile.BadZipFile, OSError, KeyError, ValueError, TypeError) as error:
            raise MapStoreError(
                f"map {name!r} is not a readable map file "
                f"({type(error).__name__}: {error})") from error
        if count.shape != elevation.shape:
            raise MapStoreError(
                f"map {name!r} is inconsistent: count {count.shape} does not "
                f"match elevation {elevation.shape}")
        return GridState(elevation=elevation, count=count, origin_ix=origin_ix,
                         origin_iy=origin_iy, resolution=resolution)
