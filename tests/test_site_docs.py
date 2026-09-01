"""Task 10: the anchor-mode ZED config and the operator's page.

Neither file is read by any code in this repo — the config is data for a
manual wrapper restart (§9 step 5/15) and the README is for a human under a
tent in the sun. What can rot without a test catching it:

1. the README staying in sync with the shipped example table,
2. the anchor config actually being 1280x720 instead of a copy-paste of the
   drive config,
3. the anchor config's ROI staying a *relaxed* mask (y = 0.35) rather than
   an emptied one (R9) — the failure mode this test exists to refuse.

pyyaml 6.0.3 is installed in .venv today but is NOT a declared dependency
(pyproject.toml lists PySide6, roslibpy, pygame only), so a bare module-level
`import yaml` would break the GS suite on a clean checkout that has the repo's
declared deps but not pyyaml. We guard it with pytest.importorskip so this
test degrades to "skipped" rather than "erroring the whole run" on such a
checkout, and parse the YAML properly when it is available rather than
regex-scraping it.
"""

import re
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parent.parent
DRIVE_CONFIG = REPO_ROOT / "rover" / "src" / "navi_localization" / "config" / "zed_front.yaml"
ANCHOR_CONFIG = REPO_ROOT / "rover" / "src" / "navi_localization" / "config" / "zed_front_anchor.yaml"
README = REPO_ROOT / "docs" / "site" / "README.md"
EXAMPLE_TABLE = REPO_ROOT / "docs" / "site" / "landmarks.example.json"


def _load_yaml(path):
    yaml = pytest.importorskip("yaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _params(doc):
    return doc["/**"]["ros__parameters"]


# --- the README stays in sync with the shipped example table ----------------

def test_readme_exists_and_mentions_the_example_site_name():
    assert README.exists(), "docs/site/README.md must exist"
    text = README.read_text(encoding="utf-8")

    import json
    example = json.loads(EXAMPLE_TABLE.read_text(encoding="utf-8"))
    assert example["site_name"] in text, (
        "README should quote the shipped example's site_name, so a changed "
        "example forces the doc to be looked at"
    )


def test_readme_documents_the_pole_axis_convention():
    text = README.read_text(encoding="utf-8")
    assert "0.417" in text
    assert "pole" in text.lower()


def test_readme_contains_the_full_competition_day_procedure():
    text = README.read_text(encoding="utf-8")
    # Spot-check enough of §9's numbered steps, in order, that a truncated or
    # paraphrased copy fails this test. These are the exact operator actions,
    # not a summary of them.
    must_contain_in_order = [
        "landmarks.example.json",
        "Start anchor",
        "fewer than two landmarks",
        "Stop anchor",
        "Solve",
        "RMS",
        "Lock",
        "do not move the rover",
        "Camera restarted",
        "LOCKED (re-expressed)",
        "Do not unlock during a run",
    ]
    positions = []
    for phrase in must_contain_in_order:
        idx = text.find(phrase)
        assert idx != -1, f"README is missing operator step text: {phrase!r}"
        positions.append(idx)
    assert positions == sorted(positions), (
        "the procedure's steps appear out of order in the README"
    )


def test_readme_marks_measured_vs_expected_range_numbers():
    text = README.read_text(encoding="utf-8")
    # R10: a bracket from the plan must never reach the operator's page
    # looertled as if it were a measurement. The measured numbers (review
    # round 3, live rover) must appear and be distinguishable from the
    # expected/bracket numbers.
    assert "measured" in text.lower()
    assert "530" in text or "265.06" in text, (
        "the measured fx (530 px @ 1280x720, 265.06 @ 640x360) must be on the page"
    )
    assert "5.5" in text or "~5.5" in text, (
        "the measured HD2K decode range (~5.5 m) must be on the page"
    )
    assert "expected" in text.lower() or "bracket" in text.lower()


# --- the anchor config is 1280x720, not a re-skinned drive config ------------

def test_anchor_config_exists_and_is_valid_yaml():
    assert ANCHOR_CONFIG.exists()
    doc = _load_yaml(ANCHOR_CONFIG)
    assert doc is not None


def test_anchor_config_pub_downscale_factor_differs_from_drive_config():
    anchor = _params(_load_yaml(ANCHOR_CONFIG))
    drive = _params(_load_yaml(DRIVE_CONFIG))

    assert drive["general"]["pub_downscale_factor"] == 2.0, (
        "drive config changed underneath this test — re-check the anchor diff"
    )
    assert anchor["general"]["pub_downscale_factor"] == 1.0, (
        "anchor config must publish at 1280x720 (pub_downscale_factor: 1.0), "
        "not the drive config's 640x360"
    )


def test_anchor_config_grab_resolution_is_hd2k():
    # §0's live measurement (review round 3): fx = 530 @ 1280x720 puts decode
    # range at ~3.2 m against 3-8 m landmarks - HD2K is what the plan settles
    # on as the anchor-config assumption, not merely a commented-out option.
    anchor = _params(_load_yaml(ANCHOR_CONFIG))
    assert anchor["general"]["grab_resolution"] == "HD2K"


# --- R9: the ROI is relaxed, never emptied -----------------------------------

def test_anchor_config_roi_polygon_is_not_empty():
    anchor = _params(_load_yaml(ANCHOR_CONFIG))
    polygon_raw = anchor["region_of_interest"]["manual_polygon"]

    assert polygon_raw.strip() != "[]", (
        "R9: manual_polygon: '[]' removes the ROI mask entirely, which lets "
        "VIO track the sun shade as a 'stationary' object that moves with "
        "the camera - it must be RELAXED (a lower y boundary), never emptied"
    )

    import json
    polygon = json.loads(polygon_raw)
    assert len(polygon) >= 3, "a real polygon, not an empty or degenerate one"


def test_anchor_config_roi_top_boundary_is_above_zero_and_below_drive_configs():
    anchor = _params(_load_yaml(ANCHOR_CONFIG))
    drive = _params(_load_yaml(DRIVE_CONFIG))

    import json
    anchor_polygon = json.loads(anchor["region_of_interest"]["manual_polygon"])
    drive_polygon = json.loads(drive["region_of_interest"]["manual_polygon"])

    anchor_top_y = min(pt[1] for pt in anchor_polygon)
    drive_top_y = min(pt[1] for pt in drive_polygon)

    assert drive_top_y == pytest.approx(0.5), "drive config's own ROI moved — re-check"
    assert 0.0 < anchor_top_y < drive_top_y, (
        "the anchor ROI must be strictly between 'wide open' (y=0) and the "
        "drive config's y=0.5 — lowering the boundary, not removing the mask"
    )
    assert anchor_top_y == pytest.approx(0.35)


def test_anchor_config_still_applies_to_depth():
    anchor = _params(_load_yaml(ANCHOR_CONFIG))
    assert anchor["region_of_interest"]["apply_to_depth"] is True


# --- R6: nothing in this repo's code ever references the anchor config ------

def test_no_code_path_references_the_anchor_config():
    search_roots = [
        REPO_ROOT / "rover" / "src" / "navi_localization" / "navi_localization",
        REPO_ROOT / "rover" / "src" / "navi_localization" / "launch",
    ]
    hits = []
    for root in search_roots:
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if not path.is_file():
                continue
            try:
                text = path.read_text(encoding="utf-8")
            except (UnicodeDecodeError, OSError):
                continue
            if "zed_front_anchor" in text:
                hits.append(str(path))
    assert hits == [], (
        f"no code path may reference the anchor config; found it in: {hits}"
    )
