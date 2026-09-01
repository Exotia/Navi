import json
import math
from pathlib import Path

import pytest

from ground_station.landmark_table import (
    Landmark, LandmarkTable, LandmarkTableError, MarkerSpec,
    landmark_table_json, load_landmark_table, parse_landmark_table)


EXAMPLE_PATH = Path(__file__).resolve().parent.parent / "docs" / "site" / "landmarks.example.json"


def _table_json(**overrides):
    """A minimal, valid table payload as a dict, so tests can tweak one field."""
    payload = {
        "schema": "navi.site_landmarks/1",
        "site_name": "test site",
        "frame": "site",
        "units": "m",
        "landmarks": [
            {"id": "51", "x": 1.0, "y": 2.0},
            {"id": "52", "x": 3.0, "y": -4.0},
        ],
    }
    payload.update(overrides)
    return payload


# --- the shipped example never rots -----------------------------------------

def test_example_file_loads_and_has_three_landmarks():
    table = load_landmark_table(EXAMPLE_PATH)

    assert len(table) == 3
    assert isinstance(table.marker, MarkerSpec)
    assert table.marker.dictionary == "DICT_5X5_100"
    assert table.marker.edge_m == 0.150
    assert table.marker.face_offset_m == 0.125
    assert table.marker.centre_height_m == 0.417
    for landmark in table.landmarks:
        assert isinstance(landmark.id, str)
        assert math.isfinite(landmark.x)
        assert math.isfinite(landmark.y)


# --- round trip ---------------------------------------------------------

def test_round_trips_through_landmark_table_json():
    original = parse_landmark_table(json.dumps(_table_json()))

    again = parse_landmark_table(landmark_table_json(original))

    assert again.site_name == original.site_name
    assert again.marker == original.marker
    assert again.landmarks == original.landmarks


# --- by_id / __len__ -----------------------------------------------------

def test_by_id_finds_by_string_id_and_returns_none_for_unknown():
    table = parse_landmark_table(json.dumps(_table_json()))

    found = table.by_id("52")
    assert found is not None
    assert found.x == 3.0
    assert found.y == -4.0

    assert table.by_id("does-not-exist") is None
    assert len(table) == 2


# --- marker defaults -------------------------------------------------------

def test_absent_marker_block_yields_every_default():
    table = parse_landmark_table(json.dumps(_table_json()))

    assert table.marker == MarkerSpec()


def test_partial_marker_block_keeps_other_defaults():
    payload = _table_json(marker={"dictionary": "DICT_4X4_50"})

    table = parse_landmark_table(json.dumps(payload))

    assert table.marker.dictionary == "DICT_4X4_50"
    assert table.marker.edge_m == MarkerSpec().edge_m
    assert table.marker.face_offset_m == MarkerSpec().face_offset_m
    assert table.marker.centre_height_m == MarkerSpec().centre_height_m


# --- unknown extra keys are ignored, not an error --------------------------

def test_unknown_extra_keys_are_ignored():
    payload = _table_json(unexpected_top_level_key="whatever the judges' sheet had")
    payload["landmarks"][0]["colour"] = "orange"

    table = parse_landmark_table(json.dumps(payload))

    assert len(table) == 2
    assert table.by_id("51").x == 1.0


# --- note survives verbatim -------------------------------------------------

def test_note_survives_verbatim_including_markup_characters():
    payload = _table_json()
    payload["landmarks"][0]["note"] = "<b>north</b> berm"

    table = parse_landmark_table(json.dumps(payload))

    assert table.by_id("51").note == "<b>north</b> berm"


def test_note_defaults_to_empty_string():
    table = parse_landmark_table(json.dumps(_table_json()))

    assert table.by_id("51").note == ""


# --- error cases, one per row of §3.3's table ------------------------------

def test_not_json_raises():
    with pytest.raises(LandmarkTableError):
        parse_landmark_table("not json at all {{{")


def test_not_an_object_raises():
    with pytest.raises(LandmarkTableError):
        parse_landmark_table(json.dumps([1, 2, 3]))


def test_absent_schema_raises():
    payload = _table_json()
    del payload["schema"]
    with pytest.raises(LandmarkTableError, match="schema"):
        parse_landmark_table(json.dumps(payload))


def test_wrong_schema_raises():
    payload = _table_json(schema="something.else/1")
    with pytest.raises(LandmarkTableError, match="schema"):
        parse_landmark_table(json.dumps(payload))


def test_absent_landmarks_raises():
    payload = _table_json()
    del payload["landmarks"]
    with pytest.raises(LandmarkTableError, match="landmarks"):
        parse_landmark_table(json.dumps(payload))


def test_landmarks_not_a_list_raises():
    payload = _table_json(landmarks={"51": {"x": 1.0, "y": 2.0}})
    with pytest.raises(LandmarkTableError, match="landmarks"):
        parse_landmark_table(json.dumps(payload))


def test_empty_landmarks_raises():
    payload = _table_json(landmarks=[])
    with pytest.raises(LandmarkTableError, match="landmarks"):
        parse_landmark_table(json.dumps(payload))


def test_entry_with_no_id_raises():
    payload = _table_json(landmarks=[{"x": 1.0, "y": 2.0}])
    with pytest.raises(LandmarkTableError, match="id"):
        parse_landmark_table(json.dumps(payload))


def test_non_string_id_raises():
    payload = _table_json(landmarks=[{"id": 51, "x": 1.0, "y": 2.0}])
    with pytest.raises(LandmarkTableError, match="id"):
        parse_landmark_table(json.dumps(payload))


def test_duplicate_id_raises():
    payload = _table_json(landmarks=[
        {"id": "51", "x": 1.0, "y": 2.0},
        {"id": "51", "x": 3.0, "y": 4.0},
    ])
    with pytest.raises(LandmarkTableError, match="51"):
        parse_landmark_table(json.dumps(payload))


def test_missing_x_raises():
    payload = _table_json(landmarks=[{"id": "51", "y": 2.0}])
    with pytest.raises(LandmarkTableError, match="51"):
        parse_landmark_table(json.dumps(payload))


def test_missing_y_raises():
    payload = _table_json(landmarks=[{"id": "51", "x": 1.0}])
    with pytest.raises(LandmarkTableError, match="51"):
        parse_landmark_table(json.dumps(payload))


def test_non_finite_x_raises():
    payload = _table_json(landmarks=[{"id": "51", "x": float("nan"), "y": 2.0}])
    with pytest.raises(LandmarkTableError, match="51"):
        parse_landmark_table(json.dumps(payload))


def test_non_finite_y_raises():
    payload = _table_json(landmarks=[{"id": "51", "x": 1.0, "y": float("inf")}])
    with pytest.raises(LandmarkTableError, match="51"):
        parse_landmark_table(json.dumps(payload))


def test_wrong_frame_raises():
    payload = _table_json(frame="map")
    with pytest.raises(LandmarkTableError, match="frame"):
        parse_landmark_table(json.dumps(payload))


def test_wrong_units_raises():
    payload = _table_json(units="ft")
    with pytest.raises(LandmarkTableError, match="units"):
        parse_landmark_table(json.dumps(payload))


# --- loader reads UTF-8 from a path ----------------------------------------

def test_load_landmark_table_accepts_str_or_path(tmp_path):
    path = tmp_path / "table.json"
    path.write_text(json.dumps(_table_json()), encoding="utf-8")

    from_path_obj = load_landmark_table(path)
    from_str = load_landmark_table(str(path))

    assert from_path_obj == from_str
    assert len(from_path_obj) == 2
