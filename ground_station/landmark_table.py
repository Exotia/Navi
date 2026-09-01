"""The judges' site map, as a file the operator edits by hand.

The ERC site map lists a handful of ArUco-tagged landmarks by their pole-axis
position in the *site* frame — the judges' grid, not the rover's. This module
loads that list from a small JSON file (`docs/site/landmarks.example.json` is
a worked example) so that `site_frame.py` has correspondences to fit against,
and so the operator can update it in a text editor between runs without
touching code.

The file is JSON, not YAML: `pyyaml` is not a declared dependency of the
ground station, and this is a file an operator edits twice a year — stdlib
`json` is enough. Every field this module requires is validated, and a
missing or malformed one raises `LandmarkTableError` with a message that
names the offending entry, written for a human reading it off a laptop
screen under a tent: "landmark 3 ('52') has no 'y'", not a stack trace.
Unknown extra keys anywhere in the file are ignored on purpose — the
operator will paste rows in from the judges' printed sheet, and this module
should not stand in the way of a copy-paste job that carries along fields it
does not use.

A landmark's `x`/`y` are its **pole axis**, not the face an operator or a
camera actually sees — the 125 mm offset from a detected marker face to the
pole is applied elsewhere (D4), never here.
"""

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple, Union


@dataclass(frozen=True)
class Landmark:
    id: str
    x: float
    y: float
    note: str = ""


@dataclass(frozen=True)
class MarkerSpec:
    dictionary: str = "DICT_5X5_100"
    edge_m: float = 0.150
    face_offset_m: float = 0.125
    centre_height_m: float = 0.417


@dataclass(frozen=True)
class LandmarkTable:
    site_name: str
    marker: MarkerSpec
    landmarks: Tuple[Landmark, ...]

    def by_id(self, id: str) -> Optional[Landmark]:
        for landmark in self.landmarks:
            if landmark.id == id:
                return landmark
        return None

    def __len__(self) -> int:
        return len(self.landmarks)


class LandmarkTableError(ValueError):
    """Raised when the table file cannot be parsed or fails validation.

    The message names the offending entry; it is shown to the operator
    verbatim.
    """


def _safe_float(value) -> Optional[float]:
    """A finite float from `value`, or None if it isn't one.

    Booleans are excluded even though `bool` is an `int` subclass in
    Python — a stray `true`/`false` in an `x`/`y` field is a malformed row,
    not a landmark at 1.0 or 0.0.
    """
    if isinstance(value, bool):
        return None
    if not isinstance(value, (int, float)):
        return None
    value = float(value)
    if not math.isfinite(value):
        return None
    return value


def _parse_marker(raw) -> MarkerSpec:
    default = MarkerSpec()
    if not isinstance(raw, dict):
        return default

    dictionary = raw.get("dictionary", default.dictionary)
    if not isinstance(dictionary, str):
        dictionary = default.dictionary

    edge_m = _safe_float(raw.get("edge_m", default.edge_m))
    if edge_m is None:
        edge_m = default.edge_m

    face_offset_m = _safe_float(raw.get("face_offset_m", default.face_offset_m))
    if face_offset_m is None:
        face_offset_m = default.face_offset_m

    centre_height_m = _safe_float(raw.get("centre_height_m", default.centre_height_m))
    if centre_height_m is None:
        centre_height_m = default.centre_height_m

    return MarkerSpec(dictionary=dictionary, edge_m=edge_m,
                       face_offset_m=face_offset_m,
                       centre_height_m=centre_height_m)


def parse_landmark_table(text: str) -> LandmarkTable:
    """Parse a landmark table from its JSON text.

    Every field this module cares about is required unless it has a
    documented default; a missing or wrong-typed one raises
    `LandmarkTableError` for the whole file rather than silently defaulting
    a coordinate to 0.0 and poisoning a rigid fit later. Unknown keys,
    anywhere in the document, are ignored.
    """
    try:
        data = json.loads(text)
    except (json.JSONDecodeError, TypeError):
        raise LandmarkTableError("the landmark table is not valid JSON")

    if not isinstance(data, dict):
        raise LandmarkTableError(
            "the landmark table is not a JSON object at the top level")

    schema = data.get("schema")
    if not isinstance(schema, str) or not schema.startswith("navi.site_landmarks/"):
        raise LandmarkTableError(
            f"unknown or missing \"schema\" ({schema!r}) — expected it to "
            f"start with \"navi.site_landmarks/\"")

    frame = data.get("frame", "site")
    if frame != "site":
        raise LandmarkTableError(
            f"\"frame\" must be \"site\", found {frame!r}")

    units = data.get("units", "m")
    if units != "m":
        raise LandmarkTableError(
            f"\"units\" must be \"m\", found {units!r}")

    raw_landmarks = data.get("landmarks")
    if not isinstance(raw_landmarks, list):
        raise LandmarkTableError(
            "\"landmarks\" is missing or is not a list")
    if len(raw_landmarks) == 0:
        raise LandmarkTableError(
            "\"landmarks\" is empty — at least one landmark is required")

    landmarks = []
    seen_ids = set()
    for index, entry in enumerate(raw_landmarks):
        if not isinstance(entry, dict):
            raise LandmarkTableError(f"landmark {index} is not an object")

        raw_id = entry.get("id")
        if raw_id is None:
            raise LandmarkTableError(f"landmark {index} has no \"id\"")
        if not isinstance(raw_id, str):
            raise LandmarkTableError(
                f"landmark {index} has a non-string \"id\" ({raw_id!r}) — "
                f"landmark ids are always strings")
        if raw_id in seen_ids:
            raise LandmarkTableError(f"landmark '{raw_id}' appears twice")
        seen_ids.add(raw_id)

        x = _safe_float(entry.get("x"))
        if x is None:
            raise LandmarkTableError(
                f"landmark '{raw_id}' has no finite \"x\"")
        y = _safe_float(entry.get("y"))
        if y is None:
            raise LandmarkTableError(
                f"landmark '{raw_id}' has no finite \"y\"")

        note = entry.get("note", "")
        if not isinstance(note, str):
            note = ""

        landmarks.append(Landmark(id=raw_id, x=x, y=y, note=note))

    site_name = data.get("site_name", "")
    if not isinstance(site_name, str):
        site_name = ""

    return LandmarkTable(
        site_name=site_name,
        marker=_parse_marker(data.get("marker")),
        landmarks=tuple(landmarks))


def load_landmark_table(path: Union[str, Path]) -> LandmarkTable:
    """Load and parse a landmark table file, utf-8."""
    return parse_landmark_table(Path(path).read_text(encoding="utf-8"))


def landmark_table_json(table: LandmarkTable) -> str:
    """Serialise a `LandmarkTable` back to the file format. Round-trips."""
    return json.dumps({
        "schema": "navi.site_landmarks/1",
        "site_name": table.site_name,
        "frame": "site",
        "units": "m",
        "marker": {
            "dictionary": table.marker.dictionary,
            "edge_m": table.marker.edge_m,
            "face_offset_m": table.marker.face_offset_m,
            "centre_height_m": table.marker.centre_height_m,
        },
        "landmarks": [
            {"id": landmark.id, "x": landmark.x, "y": landmark.y,
             "note": landmark.note}
            for landmark in table.landmarks
        ],
    })
