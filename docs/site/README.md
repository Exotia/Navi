# Site anchor — operator guide

This page is for the operator at the competition, not for a developer. It has two parts:
the landmark table you edit by hand, and the step-by-step procedure for anchoring the
rover to the judges' grid before a run. Read it once on the warm-up day; on the day of the
run you should only need §2.

If a number on this page is marked **measured**, it came off the real rover's camera. If
it is marked **expected**, it is an arithmetic bracket from the plan and has not been
checked against hardware — treat it as a guess, not a fact.

---

## 1. The landmark table

The judges hand out a site map with the ERC landmarks' positions in their own "site" grid.
You copy that into a small JSON file the ground station loads. Here is the shipped example,
`docs/site/landmarks.example.json` — **EXAMPLE — replace before the run**:

```json
{
  "schema": "navi.site_landmarks/1",
  "site_name": "EXAMPLE — replace before the run",
  "frame": "site",
  "units": "m",
  "marker": {
    "dictionary": "DICT_5X5_100",
    "edge_m": 0.150,
    "face_offset_m": 0.125,
    "centre_height_m": 0.417
  },
  "landmarks": [
    {"id": "51", "x": 12.40, "y": 3.10, "note": "pole by the north berm"},
    {"id": "52", "x": 18.05, "y": -2.20},
    {"id": "53", "x": 9.70, "y": -6.45}
  ]
}
```

What each field means:

- `schema` — always `"navi.site_landmarks/1"`. Do not change it.
- `site_name` — free text, shown in the SITE drawer so you can tell at a glance you loaded
  the right file. Rename it away from `EXAMPLE — replace before the run` when you make your
  own copy, so nobody drives against the sample data by accident.
- `frame` — always `"site"`.
- `units` — always `"m"` (metres).
- `marker` — optional as a whole; every key inside it has a default, so you only need to
  set the ones the rules or the range experiment (checklist item 3) actually change:
  - `dictionary` — the ArUco dictionary name, e.g. `DICT_5X5_100`. This is **data, not
    code** — whichever dictionary the printed tags actually decode under (found on hardware,
    checklist item 3) goes here, unedited by anyone but you.
  - `edge_m` — the marker's printed edge length in metres (0.150 m for the ERC tag).
  - `face_offset_m` — how far the marker's face sits in front of the landmark's pole axis
    (0.125 m — half of the 250 mm box).
  - `centre_height_m` — the tag centre's height above the ground (0.417 m).
- `landmarks` — a list of the landmarks you can use. Each one:
  - `id` — the string printed on the marker (`"51"`, not `51`). A hand-measured landmark
    that has no ArUco tag can use any id you like — `"corner-post"` is fine. Ids are always
    strings, everywhere in this system, on the wire and in this file.
  - `x`, `y` — **the pole axis position**, in site metres — **not the tag face**. The face
    sits `face_offset_m` (0.125 m) in front of the pole axis; the ground station and the
    rover both correct for this internally, so what you type here is the axis, matching what
    the judges' map actually gives you.
  - `note` — optional, free text. Shown verbatim in the ground station (it is never treated
    as markup, so write whatever helps you find the landmark by eye: `"pole by the north
    berm"`).

**Before you rely on it:** load the file in the ground station's **Site** drawer once and
confirm every landmark appears with the coordinates you typed. A typo found here, sitting
in a tent with time to spare, is free. A typo found during the anchor phase is not.

---

## 2. Competition-day procedure

This is copied in full from the site-anchor plan (§9) and is the exact sequence the
ground station and the rover expect. Do not improvise a shortened version of it — steps 14
and 15 in particular exist because of a failure mode that only shows up if they are
skipped.

### The evening before / warm-up day

1. Take the judges' site map. Type the landmark coordinates into a copy of
   `docs/site/landmarks.example.json` — **pole axis positions**, in site metres, ids as the
   strings printed on the markers. Save it somewhere you will find it.
2. Put the confirmed dictionary name and marker edge length in the `marker` block.
3. Load the file in the ground station's **Site** drawer once and confirm every landmark
   appears with the coordinates you typed. A typo found here is free.

### On the start point, before anything moves

4. Rover placed, powered, `/localization/status` OK, mode **manual**.
5. If using stage 3: stop the ZED wrapper and relaunch it with `zed_front_anchor.yaml`. Wait
   for `/localization/status` to come back OK.
6. Open **Site**. Press **Start anchor**. Watch the landmark list fill.
7. **If fewer than two landmarks are seen:** the card says so. Turn the rover slowly on the
   spot with the gamepad, in manual, and keep turning until two or more appear. The anchor
   node keeps accumulating throughout; you do not have to restart it. *Nothing in the ground
   station turns the rover for you — that is on purpose.*
8. Wait until every landmark you want reads **good** — `n` above 50, `spread_m` small. Ten to
   twenty seconds per marker is normal.
9. **If a landmark will not detect:** click it in the camera view and press **Probe** instead.
   A hand-measured landmark counts exactly the same. If you tape-measure a landmark, type it
   into the table with any id you like — ids are strings.
10. Press **Stop anchor**, then **Solve**.

### Judging the fit

11. Read the RMS. Under ~0.10 m is good; over ~0.5 m the card warns and you should look at
    the worst landmark it names.
12. If one landmark is clearly wrong, untick it and solve again. If the scale warning
    appears, you have probably matched two measurements to the wrong ids — check them.
13. **With only two landmarks, the RMS proves almost nothing.** Get a third if you can, even
    by tape measure.

### Locking and going

14. Press **Lock**. The header shows `SITE: LOCKED` and its RMS. From here the waypoint list
    is read as **site** coordinates and the NAV row labels say so. **From this moment until
    step 15 is finished, do not move the rover — not a centimetre, not a degree.** The
    restart in step 15 bears a fresh map frame at wherever the rover then is, and the lock
    is re-expressed assuming that is exactly where you locked it.
15. If you used the anchor config: stop the wrapper, relaunch with `zed_front.yaml`, wait for
    `/localization/status` OK — then press **Camera restarted** in the Site drawer. The card
    shows `LOCKED (re-expressed)`: the lock has been moved into the new map frame born at
    the parked rover. (There is no "confirm a landmark still reads the same" check here —
    at drive-config resolution nothing decodes past ~1.6 m, so the re-expression is the
    guarantee, and the rover staying parked is its one assumption.)
16. Type the judges' waypoints in site coordinates. Press **Autonomous**, then **Go**. The
    conversion happens at the wire; the rover receives ordinary map coordinates and never
    knows the difference.
17. **Do not unlock during a run.** The ground station refuses, and it is right to.

---

## 3. Range numbers — measured vs. expected

Whether a landmark decodes at all, and whether the depth patch under it returns a range,
depends on the published image size — a 150 mm 5×5 ArUco tag needs roughly 25 px across to
decode, so decode range works out to `0.150 · fx / 25` metres, i.e. `fx / 167`. `fx` itself
depends on which ZED 2i lens is fitted and at what published resolution.

**Measured on the live rover, 2026-09-01 (review round 3):** `depth/camera_info` reports
**fx = 265.06 px at 640×360**, i.e. **fx = 530 px at 1280×720** — confirming the **2.1 mm /
110° lens** is fitted (not the 4 mm / 72° variant). From that measured fx:

| published size | fx | decode range | status |
| --- | --- | --- | --- |
| 640×360 (today's drive config, `pub_downscale_factor: 2.0`) | 265 | **~1.6 m** | measured |
| 1280×720 (`pub_downscale_factor: 1.0`) | 530 | **~3.2 m** | measured |
| 2208×1242 (`grab_resolution: 'HD2K'`) | ~914 | **~5.5 m** | measured |

Landmarks are 3–8 m away in the normal case. 640×360 (the drive config) is short of that at
every range; 1280×720 barely reaches the nearest landmarks; **HD2K is what the anchor
config (`rover/src/navi_localization/config/zed_front_anchor.yaml`) actually uses**, because
it is the only one of the three that reaches into the working range with the lens this
rover has. This is why that file enables `grab_resolution: 'HD2K'` rather than leaving it as
a commented-out option.

Before this measurement, the plan carried only an **expected** bracket, because the two ZED
2i lens variants differ in `fx` by roughly a factor of two and neither this repo nor the sim
holds a calibration for the real one:

| published size | fx (expected, either lens) | decode range (expected) |
| --- | --- | --- |
| 640×360 | 225 – 440 | 1.3 – 2.6 m |
| 1280×720 | 450 – 880 | 2.7 – 5.3 m |
| HD2K | 775 – 1520 | 4.6 – 9.1 m |

The measured row above supersedes this bracket for this rover — it is left here only so
that if the camera or lens is ever swapped, whoever reads this page knows a bracket, not a
fact, is all that is left until the measurement is redone (checklist item 4 in the
site-anchor plan, `docs/superpowers/plans/2026-09-01-site-anchor.md` §8).

Depth itself is clamped to `[0.3, 10.0]` m regardless of what decodes (`depth.min_depth` /
`depth.max_depth` in both configs) — a marker that decodes beyond 10 m still returns no
range.

---

## 4. Why an anchor-mode camera config exists at all

The rover's normal ("drive") ZED config, `rover/src/navi_localization/config/zed_front.yaml`,
publishes 640×360 images (`pub_downscale_factor: 2.0`) to keep the video sender's bandwidth
down during a run, and masks the top half of the frame (`region_of_interest`) because the
planned sun shade above the lens would otherwise be tracked by VIO as a "stationary" object
that moves with the camera.

Both of those are wrong for the few minutes of the anchor phase, when the rover is
stationary and nothing else needs the bandwidth: the drive config's resolution puts every
landmark decode range short of where landmarks actually are (§3 above), and its ROI mask
sits close enough to the landmark's expected image row that a couple of degrees of upward
pitch — or a landmark on rising ground — pushes it into the masked region, silently removing
its depth while the ArUco detector keeps finding the marker.

`rover/src/navi_localization/config/zed_front_anchor.yaml` is the fix: full 1280×720
publishing, `grab_resolution: 'HD2K'` for the reach into 3–8 m range, and the ROI's top
boundary lowered to `y = 0.35` (not removed — an empty polygon (`manual_polygon: '[]'`)
would let VIO track the sun shade again, and stage 3 measures landmarks in the very map
frame that would corrupt, silently, with no depth-clamp or detector-failure symptom to
catch it).

**This file is data only.** No code in this repository loads it, launches it or switches to
it — applying it is exactly steps 5 and 15 above: stop the wrapper, relaunch it by hand with
this file in place of the drive config, and relaunch again with the drive config before
driving. A node that could restart the camera itself would be a node that could take the
camera away during a run, which is not acceptable here.
