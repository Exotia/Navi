# Site Anchor: putting the judges' grid on the rover's map

**Goal sentence.** The rover is placed somewhere on the marsyard with a heading nobody wrote
down. Before it drives, the operator measures two or more of the ERC landmarks, the ground
station solves the site→map transform, shows its RMS residual, and locks it — and from that
moment waypoints typed in the judges' grid coordinates arrive at `goal_relay` as ordinary
`map` coordinates. Nothing downstream of `/nav_request` learns that a site frame exists.

---

## 0. Read this first (state of the tree, verified 2026-09-01)

Every claim below was checked against the code in this repo today. Where the briefing that
produced this plan disagreed with reality, **reality wins** and the difference is called out
with ⚠.

| Fact | Where |
| --- | --- |
| GS suite: **354 passed** in 8.7 s | `.venv/bin/python3 -m pytest tests/ -q` |
| Nothing under `ground_station/` imports `rclpy` | `ground_station/ros_client.py` uses `roslibpy` only |
| `ground_station/models.py` imports `json`, `math`, `dataclasses` — no Qt, no ROS | `ground_station/models.py` |
| Camera in base_footprint = `Transform(0.345, 0.0, 0.548, 0,0,0,1)` | `rover/src/navi_localization/navi_localization/pose_composition.py:36` |
| Rover pose for the GS: `/localization/pose` (`nav_msgs/Odometry`, `map` frame) | published by `localization_status.py:52` |
| `/localization/status` states are `off` / `searching` / `ok` | `tracker.py`, `status_json` |
| ZED depth is clamped to `[0.3, 10.0]` m by `depth.min_depth` / `depth.max_depth` | `zed_front.yaml` |
| ERC landmark geometry — 250×250×310 mm box, 150×150 mm 5×5 tag, coordinate = pole axis at tag-centre height, ids 51–64 provisional, ≥ 2 visible from the start point | `[ERC 2026] RULES Rev.3.pdf` §7.3.2.1 |
| ZED grabs **HD720**, publishes with `pub_resolution: CUSTOM`, `pub_downscale_factor: 2.0` | `rover/src/navi_localization/config/zed_front.yaml:10-13` |
| ZED region of interest masks the **top half** of the frame, `apply_to_depth: true` | `zed_front.yaml:16-27` |
| Deploy is `./deploy_rover.sh` (rsync + colcon on the Orin), not `scp` | `deploy_rover.sh` |
| `pyyaml` is present in the venv but is **not** a declared dependency | `pyproject.toml` `dependencies = [PySide6, roslibpy, pygame]` |

### ⚠ Four corrections to the brief, binding for this plan

1. **There is no full-resolution image topic to subscribe to.** `pub_downscale_factor` is a
   global wrapper setting: *every* published image (rgb, left, depth) comes out at 640×360.
   The brief's "subscribe to the full-resolution topic instead of the downscaled one" has no
   topic to point at.

   The arithmetic that matters is one line — a marker of edge `e` at range `R` subtends

   ```
   px = e · fx / R
   ```

   — and a 5×5 ArUco tag is 7 modules wide with its border, so it needs **≳ 25 px** to
   decode at all and comfortably more to decode reliably. Rearranged, the decode range is
   `R_max = e · fx / 25`, i.e. `0.150 · fx / 25 = fx / 167` metres for the ERC tag.

   **`fx` is the one number this plan cannot look up offline, and it is not the same on both
   ZED 2i lens variants.** The 2.1 mm (110° H) variant gives fx ≈ 450 px at 1280×720; the
   4 mm (72° H) variant gives fx ≈ 880 px. `sim/src/navi_sim_bringup/urdf/asterope_sim.urdf.xacro`
   models the front camera at `horizontal_fov: 1.2` rad (69°), which points at the 4 mm
   variant, but a simulation FOV is not a calibration. **The real number is one command away
   on the rover and it is bring-up checklist item 2:**
   `ros2 topic echo /zed_front/zed_node/depth/camera_info --once`. Write it down there.

   What follows regardless of which lens is fitted, because the ratio is all that is needed:

   | published size | fx | decode range `fx/167` |
   | --- | --- | --- |
   | 640×360 (today, `pub_downscale_factor: 2.0`) | 225 – 440 | **1.3 – 2.6 m** |
   | 1280×720 (`pub_downscale_factor: 1.0`) | 450 – 880 | **2.7 – 5.3 m** |
   | 2208×1242 (`grab_resolution: 'HD2K'`) | 775 – 1520 | 4.6 – 9.1 m |

   Landmarks that are 3–8 m away are the normal case, and **every** value in the 640×360 row
   is short of that. **Stage 3 therefore requires an anchor-mode ZED config with
   `pub_downscale_factor: 1.0`, applied by restarting the wrapper.** That is exactly the
   operator procedure the brief already accepts. Task 10 ships the config file as *data*;
   **no code in this sub-project ever changes a resolution parameter.** Whether 1280×720 is
   enough, or HD2K is needed as well, is measured in checklist item 4 — the table above says
   which answer to expect, it does not settle it.

   **Measured on the live rover, 2026-09-01 (review round 3):** `depth/camera_info` reports
   **fx = 265.06 at 640×360**, i.e. fx = 530 at 1280×720 — the **2.1 mm / 110° lens**. The
   bracket collapses: decode range ≈ **1.6 m today, 3.2 m at 1280×720, ~5.5 m at HD2K**
   (fx ≈ 914). With landmarks at 3–8 m, 1280×720 barely reaches the nearest: **plan on HD2K
   for the anchor config.** Topic names confirmed live:
   `/zed_front/zed_node/left/image_rect_color`, `/zed_front/zed_node/depth/depth_registered`,
   `/zed_front/zed_node/depth/camera_info`.
2. **The ZED region of interest masks the top half of the frame, and depth with it.** The
   polygon is `[[0.0,0.5],[1.0,0.5],[1.0,1.0],[0.0,1.0]]` with `apply_to_depth: true`, so
   everything above normalized y = 0.5 — the horizontal plane through the optical axis — is
   invisible to tracking, mapping *and* depth. Camera optical centre 0.548 m, marker centre
   0.417 m → the marker sits 0.131 m below the camera, and its normalized image row is

   ```
   y = 0.5 + (0.131 / R) · fy / height
   ```

   At R = 4 m that is `0.5 + 0.0328·fy/720`, i.e. **y ≈ 0.52 with the 2.1 mm lens and
   y ≈ 0.54 with the 4 mm lens** — inside the unmasked half by two to four percent of the
   frame height, with the marker's *top* edge nearer still. It never crosses the boundary
   on level ground at any range, but the margin shrinks as 1/R, so **any upward pitch of a
   couple of degrees, or a landmark on rising ground, puts it into the masked region and
   there will be no depth there at all**. The anchor config in Task 10 therefore also opens
   the ROI upward. This is the single most likely reason stage 3 returns nothing on the
   marsyard; it is R2 and it is in the bring-up checklist.

   ⚠ **Opening the ROI is not free, and `manual_polygon: '[]'` is the wrong way to do it.**
   `zed_front.yaml`'s own comment says why the top half is masked: sky and sun glare are
   phantom inputs, and the planned sun shade sits above the lens — *"the shade MUST be masked
   out or VIO would track a 'stationary' object that moves with the camera."* The anchor phase
   measures landmarks **in the ZED's map frame**, so VIO going bad during anchoring does not
   degrade the anchor, it silently invalidates it. Task 10's config therefore lowers the
   boundary to `y = 0.35` — enough headroom for several degrees of pitch at any working range
   — rather than removing the mask. See R9.
3. **The anchor node never commands motion.** The brief mentions "an optional slow
   point-turn sweep". In this repo `mode_supervisor` is the sole publisher of `/rover_twist`
   (SP5, restated in SP8 and SP11), and no node outside it may reach for a twist. The sweep
   is therefore an **operator action**: the ground station tells the operator *"fewer than 2
   landmarks — turn slowly on the spot"*, the operator turns with the gamepad in manual
   mode, and the anchor node keeps accumulating throughout. `/site/anchor_command` carries
   `start` / `stop` / `reset` and nothing that moves a wheel.
4. **Marker orientation is not used at all, not even for the 125 mm offset.** The brief asks
   for the face detection to be projected 125 mm along the inward face normal. Substituting
   the *camera-to-marker ray* `r̂` for the true inward normal `n̂` displaces the estimated
   pole axis by the full chord between them:

   ```
   error = 0.125 · |n̂ − r̂| = 0.125 · 2·sin(θ/2)
   ```

   where θ is how far off-perpendicular you are viewing the face. **65 mm at 30°, 96 mm at
   45°** — and 45° is the hard bound, because the box carries four identical faces and the
   one you detect is always the one nearest perpendicular. (An earlier draft quoted
   `0.125·(1−cos θ)`, 17 mm and 37 mm; that is only the *radial* component. The tangential
   term `0.125·sin θ` is the larger one and it does not vanish. Review round 2 fixed it.)

   Committed anyway, with eyes open: 65 mm is the same order as the depth noise on a 4 m
   return through a thin pole (R3), it is bounded and never exceeds 96 mm, and over a 10 m
   landmark baseline it is a 0.4° yaw error. Using the ray keeps decision D3 ("positions
   only, never marker orientation") literally true in the code, and collapses the offset to
   **"add 0.125 m to the measured range, then unproject"**, one line that stage 2 and stage 3
   share. The alternative — a PnP pose per marker — buys back at most 96 mm at the cost of
   depending on an orientation that is ambiguous in 90° steps by construction.

---

## 1. Global constraints

Read these before your task. They are not negotiable and they are not restated per task.

### Layering

- **Nothing under `ground_station/` may import `rclpy`, `zed_msgs`, `cv2` or anything from
  `rover/`.** The ground station talks to the rover through `ground_station/ros_client.py`
  and its rosbridge connection, full stop.
- **`ground_station/models.py` imports neither Qt nor ROS.** Same for the two new pure GS
  modules this plan adds (`site_frame.py`, `landmark_table.py`): stdlib only —
  `json`, `math`, `dataclasses`, `pathlib`.
- **Rover Python nodes** live under `rover/src/`. Everything this plan adds on the rover goes
  in `rover/src/navi_localization/`, following `localization_status.py`'s split: the
  arithmetic lives in a pure module that imports no ROS and no OpenCV, the node shell is a
  thin `rclpy.node.Node` on top. A pure module that cannot be imported on the laptop is a
  bug — `zed_msgs` and `cv2` exist only on the Orin.
- **No `sys.path` shim between the two trees.** `ground_station/ui/wheel_view.py` reaches
  into `rover/src/navi_shaper` that way and degrades gracefully if it is missing; that
  precedent is deliberately *not* followed here. See design decision D1.

### Commands

Ground station suite, from the repo root — **354 passed today, and it stays green after
every task**:

```
.venv/bin/python3 -m pytest tests/ -q
```

`navi_localization` pure-module tests, laptop-safe (the full package suite needs `zed_msgs`,
which only exists on the Orin — so name your test files explicitly rather than running
`test/`):

```
bash -c 'source /opt/ros/humble/setup.bash && \
  PYTHONPATH=$PWD/rover/src/navi_localization:$PYTHONPATH \
  python3 -m pytest rover/src/navi_localization/test/test_landmark_geometry.py -q -p no:cacheprovider'
```

The whole `navi_localization` suite (Orin only, and only in the bring-up checklist):

```
bash -c 'source /opt/ros/humble/setup.bash && source rover/install/setup.bash; \
  cd rover/src/navi_localization && python3 -m pytest test/ -q'
```

Build (Orin only): `cd rover && colcon build --packages-select navi_localization --symlink-install`.

### Test discipline

- **TDD, every task.** Write the test file first, watch it fail for the right reason, then
  write the code. A task whose commit contains code without a test that would have failed
  before it is not done.
- **Pin behaviour, not pixels.** Assert that the RMS a widget shows is the RMS the solver
  produced; do not assert a stylesheet string, a font size, or a widget's geometry.
- **No test may require the Orin, a camera, a ROS graph or a network.** Everything in Tasks
  1–11 runs on the laptop. Node tests instantiate the node class against fake publishers and
  fake subscriptions the way `rover/src/navi_localization/test/test_localization_status.py`
  already does; if a node test needs a live graph it belongs in the bring-up checklist
  instead.

### The Orin is offline

Nothing in Tasks 1–11 may be blocked on the rover computer or the real camera. Anything that
genuinely needs hardware goes in §7, the hardware bring-up checklist, and **must not** appear
as a task acceptance criterion.

### Commits

One commit per task. Explicit `git add <paths>`, never `git add -A`.

```
git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit
```

Trailers on every commit:

```
Co-Authored-By: Claude <model> <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01JKPAhQayr2XFS87SjxAwzJ
```

Message voice — a sentence saying what changed and why, no `feat:`/`fix:` prefixes. Look at
`git log --oneline -15` for the register. Example of the right shape:

> *Site coordinates become map coordinates once, at Go: the ground station solves a rigid
> 2D fit over the landmarks it measured and converts the operator's waypoints at the wire,
> so nothing on the rover has to know the judges' grid exists*

**Never push.** If `.git/index.lock` exists, wait 2 s and retry — other agents work in this tree.

### Do not touch

`rover/src/navi_autonomy/`, `rover/src/navi_supervisor/`, `rover/src/navi_shaper/`,
`rover/src/navi_teleop/`, `sim/`, `ground_station/gamepad_input.py`, and the SP4–SP11 plan
files. `goal_relay` is not edited by this plan — that is the whole point of design decision
D2. The only shared GS files edited are `models.py`, `ros_client.py`, `ui/main_window.py`,
`ui/dashboard_page.py`, `ui/nav_row.py` and `ui/video_panel.py`, and each is edited by exactly
one task. On the rover the shared files are `navi_localization/pose_composition.py` (T3 only)
and `navi_localization/setup.py` (T7 and T8, one line each).

---

## 2. Design decisions (settled — implement, do not re-open)

**D1. The transform is ground-station property, and so is the code that computes it.**
`site_frame.py` and `landmark_table.py` live in `ground_station/`, as stdlib-only siblings of
`models.py`. Reasoning: the rover never applies the transform (D2), so the rover never needs
the module; putting it on the rover side would mean the GS reaching across the tree with a
`sys.path` shim (the `wheel_view.py` pattern) to import code its owner does not use. The
rover's *measurement* geometry — pixel + depth → map point, and the pole offset — is a
different thing, is genuinely rover-side, and lives in
`rover/src/navi_localization/navi_localization/landmark_geometry.py`. Two pure modules, no
shim, no shared import, and the only thing crossing the wire is JSON.

**D2. Startup-only, ground-station-only.** The transform is solved once before driving and
locked. There is no mid-run re-anchoring, no re-solve while a run is active, and **no change
to `goal_relay` or any rover autonomy code path**. The conversion happens in exactly one
place: `MainWindow._on_go_requested`, at the moment the waypoint list becomes a
`/nav_request`. With no transform locked, `_on_go_requested` behaves **byte-identically to
today**, and there is a regression test that says so.

**D3. Positions only, never orientation.** ERC landmarks are 250×250×310 mm boxes carrying
the *same* ArUco face on all four sides, so a marker's yaw is ambiguous in 90° steps and is
never read. The transform is a rigid 2D fit (x, y, yaw; **no scale**) solved in closed form
by the 2D Kabsch/Umeyama construction over ≥ 2 point correspondences.

**D4. The pole-axis offset is a range correction.** A landmark's published coordinate is its
**pole axis** at marker-centre height (0.417 m above ground). A detected marker face sits
0.125 m in front of that axis (half of the 250 mm box). Because the inward normal is taken as
the camera-to-marker ray (correction 4 above), the offset is applied as
`range_corrected = range_measured + 0.125` before unprojecting. **Asymmetry, deliberate and
documented in the code:** a *manual* depth click targets the pole or box body the operator
picked, and the default manual mode applies **no** offset (`target: "pole"`, `offset_m = 0`).
An operator who clicks the flat face of the box instead can say so (`target: "box_face"`) and
get the same 0.125 m correction. The probe request carries the choice; the node does not guess.

**D5. Three stages; later stages swap only the measurement source.**
Stage 1 (Tasks 1, 2, 4, 5, 6, 9) is the site-frame arithmetic, the landmark table, the SITE
card and the Go-time conversion — useful on its own, since an operator with a tape measure
can type two landmark positions in by hand. Stage 2 (Task 7) adds the click-and-probe
measurement. Stage 3 (Task 8) adds the ArUco anchor phase. Stages 2 and 3 both produce the
*same* thing: a map-frame landmark position with an id and a quality figure. The solver and
the card do not care which one produced it.

**D6. Marker identity is configuration, never code.** The dictionary name and the marker edge
length live in the landmark table file. Provisional ids are 51–64 and the edge is 150 mm, but
the final list is pending ERC Update Report #3, so **no marker id appears as a literal
anywhere in `ground_station/` or `rover/src/`** except in the example table and in test
fixtures. Same for the dictionary: `DICT_5X5_100` is a *default string in a config file*, not
an import.

**D7. The landmark table is JSON.** `pyyaml` is not a declared dependency of the ground
station (`pyproject.toml` lists PySide6, roslibpy, pygame) and this plan does not add one for
a file the operator edits twice a year. Stdlib `json`, an explicit `schema` key, and a loader
whose error messages name the offending row.

---

## 3. Interface contracts

**These are the whole contract.** Implementers receive their task text plus this section and
nothing else. Anything not written here is your choice; anything written here is fixed and
another task is being written against it right now.

### 3.1 Conventions

- All coordinates are **metres**, all angles **radians**, all frames **right-handed, z up**.
- The `map` frame is the rover's, born at boot pose (`/localization/pose` publishes
  base_footprint in it).
- The `site` frame is the judges', from the site map handed out at the competition.
- **The transform maps site → map:** `p_map = R(yaw) · p_site + (x, y)`.
- **Landmark ids are strings, always.** ArUco id 51 is `"51"`. A hand-measured landmark may
  be `"corner-post"`. Never an int on the wire, never an int as a dict key.
- Rover→GS status topics are `std_msgs/String` carrying JSON — the convention
  `/video_status`, `/mode_status`, `/nav_status` and `/drive_status` already set, and the
  reason there is no new `.msg` package here (an `ament_cmake` package for two messages).
- Every JSON field listed is **required unless marked optional**; a parser that meets a
  missing or wrong-typed field returns `None` for the whole payload rather than a
  half-built object.

  ⚠ **This is deliberately *stricter* than `parse_nav_status`, so copy its shape and not its
  stance.** `parse_nav_status` returns `None` only for a payload that is not JSON or not an
  object, and otherwise fills each bad field from that field's default — right for a status
  display, where a missing ETA should not blank the whole row. It is wrong here: these
  payloads carry **coordinates**, and a `_safe_float` default of `0.0` on a truncated frame
  puts a landmark at the map origin and poisons the fit. So: reuse `_safe_float` /
  `_safe_int` to *coerce*, but treat a `None` back from them on a required numeric field as
  a failure of the whole payload and return `None`. The one exception is a `ProbeResult`
  with `ok: false`, where `x`/`y`/`z`/`range_m` are required to be `null` — see §3.4.
- `stamp_s` is the publishing node's monotonic clock. It is meaningful only as a difference.
  The ground station judges staleness by its own clock.

### 3.2 `ground_station/site_frame.py` — the solver

```python
@dataclass(frozen=True)
class LandmarkPair:
    """One correspondence: a landmark whose site coordinate is published and
    whose map coordinate we measured."""
    id: str
    site_x: float
    site_y: float
    map_x: float
    map_y: float


@dataclass(frozen=True)
class SiteTransform:
    """site -> map. p_map = R(yaw) @ p_site + (x, y)."""
    x: float
    y: float
    yaw: float                  # radians, wrapped to (-pi, pi]
    rms_m: float                # RMS of the per-landmark residuals, metres
    max_residual_m: float       # the worst single residual
    worst_id: str | None        # the landmark that produced it
    n_points: int
    scale_hint: float           # see below; 1.0 means the baselines agree
    ids: tuple[str, ...]        # the ids that went into the fit, in order


class SiteFrameError(ValueError):
    """Raised when a fit cannot be attempted. The message is shown to the
    operator verbatim, so write it for a human under time pressure."""


def solve_site_to_map(pairs: Sequence[LandmarkPair]) -> SiteTransform: ...
def site_to_map(t: SiteTransform, x: float, y: float) -> tuple[float, float]: ...
def map_to_site(t: SiteTransform, x: float, y: float) -> tuple[float, float]: ...
def site_yaw_to_map_yaw(t: SiteTransform, yaw: float) -> float: ...
def residuals(t: SiteTransform, pairs: Sequence[LandmarkPair]) -> list[tuple[str, float]]: ...
def reexpress_at_lock_pose(t: SiteTransform, pose_x: float, pose_y: float,
                           pose_yaw: float) -> SiteTransform: ...
```

**The closed form** (do not use an iterative optimiser, do not import numpy):

```
c_s = centroid of the site points          c_m = centroid of the map points
a_i = site_i - c_s                          b_i = map_i - c_m
num = Σ (a_x·b_y - a_y·b_x)                 den = Σ (a_x·b_x + a_y·b_y)
yaw = atan2(num, den)
(x, y) = c_m - R(yaw)·c_s
```

**Residual** for landmark *i* is the Euclidean distance
`hypot(map_i - (R(yaw)·site_i + t))`, in metres. `rms_m` is
`sqrt(Σ residual_i² / n)`.

**`scale_hint`** is `Σ|b_i| / Σ|a_i|` — the ratio of the measured spread to the published
spread. The fit forces it to 1.0; reporting what it *would* have been is how the operator
catches a swapped pair of ids. Anything outside `[0.9, 1.1]` is worth a warning in the UI.

**Edge cases, each with a test:**

| Case | Behaviour |
| --- | --- |
| `len(pairs) < 2` | `SiteFrameError("need at least 2 landmarks, got N")` |
| duplicate `id` in `pairs` | `SiteFrameError("landmark '51' appears twice")` |
| site points all coincident (spread < 0.05 m) | `SiteFrameError("the landmarks are at the same site position — check the table")` |
| map points all coincident | `SiteFrameError("the measurements are at the same map position — the rover measured one landmark twice")` |
| `hypot(num, den) < 1e-9` | same coincidence errors; this is the only true degeneracy |
| **collinear points (any number)** | **NOT degenerate. A rigid 2D fit with no scale is fully determined by 2 points; collinearity only breaks affine/scaled fits. Solve normally, and have a test that says so.** |
| exactly 2 pairs | Solves. `rms_m` is then half the baseline-length error and nothing else — a mis-identification that happens to preserve the distance is invisible. `solve` still returns; the *card* is what warns (§3.6). Document this in the module docstring. |
| an outlier landmark | No RANSAC, no automatic rejection: with 2–4 points it would be guesswork. Report `max_residual_m` and `worst_id`; the operator unticks the landmark in the card and re-solves. |
| NaN / inf in any coordinate | `SiteFrameError("landmark '51' has a non-finite coordinate")` |

`yaw` is wrapped to `(-pi, pi]`. `site_yaw_to_map_yaw` adds `t.yaw` and wraps.

**`reexpress_at_lock_pose`** (review round 3): the ZED persists no area memory
(`area_memory_db_path: ''`), so a wrapper restart bears a NEW map frame at the rover's pose
at relaunch. With the rover unmoved from Lock through the relaunch, that new frame IS the
rover-at-lock frame, so the locked transform survives the restart as
`site->new_map = inverse(T_oldmap_base_at_lock) . t` — where `(pose_x, pose_y, pose_yaw)` is
the rover's `/localization/pose` at the moment of Lock. Closed form:
`yaw' = wrap(t.yaw - pose_yaw)`; the translation is the old-frame lock pose subtracted and
counter-rotated: `(x', y') = R(-pose_yaw) . ((t.x, t.y) - (pose_x, pose_y))`. The fit-quality
fields (`rms_m`, `max_residual_m`, `worst_id`, `n_points`, `scale_hint`, `ids`) carry over
unchanged — re-expression moves the frame, not the fit. Tests: identity pose returns `t`
field-for-field; a pure-rotation pose rotates a converted waypoint by exactly that angle;
round-tripping a landmark through `site_to_map` before and after equals the rover-relative
position either way.

### 3.3 `ground_station/landmark_table.py` — the published site map

File format (JSON, hand-edited by the operator on the warm-up day):

```json
{
  "schema": "navi.site_landmarks/1",
  "site_name": "ERC 2026 marsyard",
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

- `id` **string**, unique, required. `x`, `y` finite floats, required, metres, site frame,
  **the pole axis** — not the face.
- `note` optional, free text, **shown as Qt PlainText** wherever it reaches a widget.
- `marker` is optional as a whole; every key inside it has a default
  (`DICT_5X5_100`, `0.150`, `0.125`, `0.417`). It exists so D6 holds: the dictionary name and
  edge length are data, not code.
- `frame` and `units` are checked and must be `"site"` and `"m"`; anything else is an error
  naming what was found. `schema` must start with `"navi.site_landmarks/"`.

API:

```python
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
    landmarks: tuple[Landmark, ...]
    def by_id(self, id: str) -> Landmark | None: ...
    def __len__(self) -> int: ...

class LandmarkTableError(ValueError):
    """Message names the offending entry; it is shown to the operator verbatim."""

def parse_landmark_table(text: str) -> LandmarkTable: ...
def load_landmark_table(path) -> LandmarkTable: ...   # pathlib/str, utf-8
def landmark_table_json(table: LandmarkTable) -> str: ...  # round-trips
```

Errors (each with a test): not JSON; not an object; wrong/absent `schema`; `landmarks` absent
or not a list or empty; an entry with no `id`; a non-string `id`; a duplicate `id`; a
non-finite or missing `x`/`y`; `frame`/`units` wrong. **Unknown extra keys are ignored** — the
operator will paste things in from the judges' sheet.

### 3.4 Stage 2 — the depth probe

**`/site/probe_request`** — `std_msgs/String`, ground station → rover:

```json
{
  "request_id": "p-1756738800.412-3",
  "label": "51",
  "u": 812, "v": 431,
  "width": 1280, "height": 720,
  "target": "pole",
  "patch_px": 11
}
```

- `request_id` unique per request; the reply carries it back. Format
  `"p-{monotonic:.3f}-{counter}"`.
- `label` is the landmark id the operator says they clicked. The rover **does not verify it**;
  it is carried through so the reply can be matched to a table row.
- `u`, `v` are pixel coordinates **in the image the operator clicked**, whose dimensions are
  `width` × `height`. The node rescales to the depth image's own size. This is not optional
  pedantry: the GS camera view is an H.264 stream from `video_sender` and its size has no
  relationship to the depth image's.
- `target` ∈ `{"pole", "box_face"}`, default `"pole"`. `"pole"` → offset 0 m.
  `"box_face"` → the 0.125 m range correction of D4.
- `patch_px` optional, odd, default 11, clamped to [1, 51].

**`/site/probe_result`** — `std_msgs/String`, rover → ground station:

```json
{
  "request_id": "p-1756738800.412-3",
  "ok": true,
  "label": "51",
  "x": 4.12, "y": -1.03, "z": 0.44,
  "frame_id": "map",
  "range_m": 4.27,
  "samples": 37,
  "valid_fraction": 0.31,
  "stamp_s": 1234.5,
  "error": null
}
```

On failure `ok` is `false`, `x`/`y`/`z`/`range_m` are `null` and `error` is a sentence for the
operator. The defined failures, verbatim strings:

- `"no depth image yet"`
- `"no rover pose yet"`
- `"localisation is not OK"` (from `/localization/status`)
- `"no valid depth at that pixel"` (fewer than 25 % of the patch valid)
- `"pixel is outside the image"`

`valid_fraction` is the share of the patch that produced a finite in-range depth. `samples` is
the count of those. A result is always published, success or failure — a probe that silently
produces nothing is a dead button.

### 3.5 Stage 3 — the ArUco anchor phase

**`/site/anchor_command`** — `std_msgs/String`, ground station → rover:

```json
{"action": "start"}
```

`action` ∈ `{"start", "stop", "reset"}`. `start` begins accumulating, `stop` freezes,
`reset` clears everything accumulated. **Nothing here commands motion** (correction 3).

**`/site/landmark_sightings`** — `std_msgs/String`, rover → ground station, 1 Hz while the
phase is anything but idle, and once on every transition:

```json
{
  "stamp_s": 1234.5,
  "phase": "running",
  "frame_id": "map",
  "dictionary": "DICT_5X5_100",
  "image_size": [1280, 720],
  "detector_ok": true,
  "error": null,
  "sightings": [
    {"id": "51", "x": 4.12, "y": -1.03, "z": 0.42,
     "n": 63, "spread_m": 0.031, "range_m": 4.27,
     "last_seen_s": 0.4, "quality": "good"}
  ]
}
```

- `phase` ∈ `{"idle", "running", "stopped"}`.
- `detector_ok` is false with an `error` string when OpenCV or the dictionary name could not
  be resolved — `"unknown ArUco dictionary 'DICT_5X5_250'"`, `"cv2.aruco not available"`.
  The GS shows this; a stage 3 that fails silently is worse than one that does not run.
- `x`, `y`, `z` are the **component-wise median** of the accumulated map-frame positions,
  already pole-axis corrected.
- `n` is how many detections are in the accumulator for this id (capped, see below).
- `spread_m` is `1.4826 · median(|p_i − p_median|)` over the accumulated points — a robust
  σ estimate in metres.
- `quality` ∈ `{"good", "weak", "noisy"}`:
  `weak` if `n < min_samples` (default 50), `noisy` if `spread_m > spread_warn_m`
  (default 0.15), otherwise `good`. `weak` wins over `noisy` when both apply.
- Sightings older than `stale_after_s` (default 60) are still reported, with their
  `last_seen_s`; nothing is silently dropped mid-phase.

### 3.6 Ground-station wire additions

`ground_station/models.py` gains, in the same style as `parse_nav_status` / `nav_request_json`:

```python
@dataclass
class ProbeResult:
    request_id: str; ok: bool; label: str
    x: float | None; y: float | None; z: float | None
    range_m: float | None; samples: int; valid_fraction: float
    error: str | None

@dataclass
class Sighting:
    id: str; x: float; y: float; z: float
    n: int; spread_m: float; range_m: float
    last_seen_s: float; quality: str

@dataclass
class SightingsReport:
    phase: str; dictionary: str; detector_ok: bool
    error: str | None; sightings: list          # list[Sighting]

def parse_probe_result(payload: str) -> ProbeResult | None: ...
def parse_sightings(payload: str) -> SightingsReport | None: ...
def probe_request_json(request_id, label, u, v, width, height,
                       target="pole", patch_px=11) -> str: ...
def anchor_command_json(action: str) -> str: ...
def new_probe_id(now_s: float, counter: int) -> str: ...
```

**`ProbeResult` carries no `quality`, and `SiteCard.set_measurement` wants one.** The mapping
is fixed here so T5 and T9 cannot disagree — it belongs to the card, not to the wire:

```
"good"  if valid_fraction >= 0.60
"weak"  if valid_fraction <  0.60
```

with a failed probe (`ok: false`) producing no measurement at all. A probe is a single
deliberate click, so there is no `spread_m` to be `"noisy"` about.

`ground_station/ros_client.py` gains, following the existing pattern exactly (lazy
`_topic_factory` on first publish, a `self.is_connected` guard that prints to stderr and
drops, a stored topic handle initialised to `None` **in `__init__` beside the other
`_*_topic` attributes**, and a `Signal` on `self.signals`):

```python
probe_result_received   = Signal(object)     # ProbeResult | None
sightings_received      = Signal(object)     # SightingsReport | None

def subscribe_probe_result(self, topic_name="/site/probe_result") -> None
def subscribe_landmark_sightings(self, topic_name="/site/landmark_sightings") -> None
def send_probe_request(self, request_id, label, u, v, width, height,
                       target="pole", patch_px=11,
                       topic_name="/site/probe_request") -> None
def send_anchor_command(self, action, topic_name="/site/anchor_command") -> None
```

### 3.7 `SiteCard` — the widget contract

`ground_station/ui/site_card.py`, class `SiteCard(QWidget)`. Attributes the tests poke
(plain attributes, no getters), signals other code connects:

```python
# signals
table_load_requested = Signal(str)      # a file path the operator chose
probe_requested      = Signal(str, str) # (landmark_id, target) - the window owns the pixel
anchor_start_requested = Signal()
anchor_stop_requested  = Signal()
anchor_reset_requested = Signal()
solve_requested      = Signal()
lock_changed         = Signal(object)   # SiteTransform when locked, None when cleared
camera_restarted     = Signal()         # wrapper relaunched, rover unmoved since Lock

# attributes
self.table_label      # QLabel: site name + landmark count, or "no table loaded"
self.load_button      # QPushButton
self.landmark_list    # QListWidget, one checkable row per landmark
self.target_combo     # QComboBox: "pole" / "box face"
self.probe_button     # QPushButton, enabled only with a landmark selected
self.anchor_button    # QPushButton, checkable: start / stop the anchor phase
self.reset_button     # QPushButton
self.solve_button     # QPushButton, enabled only with >= 2 ticked measured landmarks
self.lock_button      # QPushButton, checkable
self.camera_restart_button  # QPushButton "Camera restarted", enabled only while locked
self.state_pill       # QLabel: NO TABLE / 0 OF 3 MEASURED / SOLVED / LOCKED
self.rms_pill         # QLabel: "RMS 0.06 m" or the SiteFrameError message
self.detail_label     # QLabel: worst residual + its id, and the scale hint when off
self.transform        # SiteTransform | None - the last solve
self.locked           # bool

def set_table(self, table) -> None
def set_measurement(self, landmark_id: str, x: float, y: float, quality: str) -> None
def apply_sightings(self, report) -> None      # SightingsReport -> many set_measurement
def apply_probe_result(self, result) -> None   # ProbeResult -> one set_measurement
```

Styling: cards via `theme.card_style()`, pills via `theme.pill_style(bg, fg)`, section titles
via `theme.section_title_style()`. **Every string that came off the wire — a landmark `note`,
an `error`, a dictionary name — is set with `setTextFormat(Qt.PlainText)` before
`setText`.** That rule already holds across this UI and it holds here.

Warnings the card must show, because the solver deliberately does not raise for them:

- exactly 2 landmarks → `detail_label` says *"2 landmarks: the residual only checks the
  distance between them. A third landmark is what catches a mis-identified marker."*
- `scale_hint` outside `[0.9, 1.1]` → *"measured spread is N % of the published spread —
  check the landmark ids"*
- `rms_m > 0.5` → the RMS pill turns warning-coloured. Never blocks the lock: the operator
  decides.

### 3.8 The Go-time conversion

`MainWindow._on_go_requested(waypoints)` becomes:

```python
if self._site_transform is not None and self._site_locked:
    t = self._site_transform
    waypoints = [Waypoint(*site_to_map(t, w.x, w.y),
                          None if w.yaw is None else site_yaw_to_map_yaw(t, w.yaw))
                 for w in waypoints]
self._nav_run_id = new_run_id(time())
self.ros_client.send_nav_request("go", waypoints, self._nav_run_id)
```

With no locked transform the method is byte-for-byte today's behaviour. **This is the only
place in the entire codebase where a site coordinate becomes a map coordinate.**

### 3.9 The other direction: a click on the plan canvas

`NavMapView.point_clicked` emits **map-frame** world coordinates (`nav_map_view.py:124`), and
`NavRow.append_world_point` puts them straight into the waypoint list. Under a locked
transform that list is read as *site* by §3.8 and converted **again** — the same click lands
somewhere the operator never pointed at, and the further the site origin is from the map
origin the further off it is.

**So the canvas click is converted back the other way, in `append_world_point`, with
`map_to_site`.** That is what `map_to_site` in §3.2 exists for; nothing else calls it in
anger. The invariant, stated once and pinned by tests in T6 and T11:

> **`NavRow.waypoints` always holds numbers in the frame the operator types in** — site when
> a transform is set, map when it is not. Everything that leaves the row for the canvas goes
> through `site_to_map`; everything that arrives from the canvas goes through `map_to_site`;
> the wire conversion at Go is the same one function, applied once.

### 3.10 Surviving the wrapper restart (review round 3 — this is not optional)

The stage-3 procedure restarts the ZED wrapper twice (anchor config in, drive config out),
and §0's measured fx says HD2K is all but mandatory, so the restarts WILL happen. The ZED
keeps no area file, so each restart bears a new `map` frame at the rover's then-current pose
— and the anchor sweep rotates the rover between the old frame's birth and Lock, so the
locked transform is wrong by the sweep angle the moment the drive config comes up. Silent,
and at drive-config decode range no landmark is measurable to catch it.

The contract: `MainWindow` stores the last `/localization/pose` seen at the moment
`lock_changed` fires with a transform (the **lock pose**). The SITE card's
`camera_restarted` signal (button enabled only while locked) makes the window replace the
transform with `reexpress_at_lock_pose(t, *lock_pose)` and hand the result back to the card
via `lock_changed`-consistent state; the card shows `LOCKED (re-expressed)`. The operator
contract is one sentence: **the rover must not move between Lock and pressing the button.**
T1 owns the function, T5 the button+signal, T9 the lock-pose capture and the swap; T11 drives
the whole sequence once.

---

## 4. File structure

**Create**

| Path | What |
| --- | --- |
| `ground_station/site_frame.py` | the rigid 2D fit (§3.2). stdlib only |
| `ground_station/landmark_table.py` | the table schema + loader (§3.3). stdlib only |
| `ground_station/ui/site_card.py` | the SITE drawer (§3.7) |
| `docs/site/landmarks.example.json` | a three-landmark example table |
| `docs/site/README.md` | the ops procedure (§8) and the file format, for the operator |
| `rover/src/navi_localization/navi_localization/landmark_geometry.py` | pure: pixel+depth+pose → map point, pole offset, accumulator. No ROS, no cv2 |
| `rover/src/navi_localization/navi_localization/site_probe.py` | stage-2 rclpy node |
| `rover/src/navi_localization/navi_localization/site_anchor.py` | stage-3 rclpy node (imports cv2 lazily) |
| `rover/src/navi_localization/config/zed_front_anchor.yaml` | anchor-mode wrapper overrides. **Data only — never launched by this plan's code** |
| `tests/test_site_frame.py`, `tests/test_landmark_table.py`, `tests/test_site_card.py`, `tests/test_site_anchor_end_to_end.py` | GS tests |
| `rover/src/navi_localization/test/test_landmark_geometry.py`, `test_site_probe.py`, `test_site_anchor.py` | rover tests |

**Modify**

| Path | Task | What |
| --- | --- | --- |
| `ground_station/models.py` | 4 | the four parsers/encoders of §3.6 |
| `ground_station/ros_client.py` | 4 | two signals, two subscribes, two publishes |
| `ground_station/ui/nav_row.py` | 6 | `set_site_transform`, and `map_to_site` on a canvas click (§3.9) |
| `ground_station/ui/dashboard_page.py` | 5 | the SITE drawer beside `node_list` |
| `ground_station/ui/main_window.py` | 9 | header button, transform state, probe round-trip, Go conversion |
| `ground_station/ui/video_panel.py` | 9 | one `clicked` signal, in **source-frame** pixels |
| `rover/src/navi_localization/navi_localization/pose_composition.py` | 3 | `transform_point`, if it is not already there |
| `tests/test_models.py`, `tests/test_ros_client.py` | 4 | new cases |
| `tests/test_nav_row.py` | 6 | new cases |
| `tests/test_main_window.py`, `tests/test_video_panel.py` | 9 | new cases incl. the no-transform regression |
| `rover/src/navi_localization/test/test_pose_composition.py` | 3 | a case for `transform_point` |
| `rover/src/navi_localization/setup.py`, `package.xml` | 7, 8 | one console_script each |

---

## 5. Waves and the dependency graph

```
WAVE 1  (four tasks, fully parallel, no dependencies between them)
  T1  site_frame.py            (GS, pure)
  T2  landmark_table.py        (GS, pure) + example table
  T3  landmark_geometry.py     (rover, pure)
  T4  models.py + ros_client.py wire

WAVE 2  (four tasks, parallel; each depends only on wave 1)
  T5  SiteCard widget + dashboard drawer      <- T1, T2
  T6  NavRow site display                     <- T1
  T7  site_probe.py rclpy node (stage 2)      <- T3
  T8  site_anchor.py rclpy node (stage 3)     <- T3

WAVE 3
  T9  main_window wiring + Go conversion      <- T4, T5, T6
  T10 anchor ZED config + operator docs       <- (none; parallel with T9)

WAVE 4
  T11 ground-station end-to-end integration test  <- T9
```

**Why this graph.** The four wave-1 tasks are the four halves of the contract made real, and
nothing in wave 1 imports anything else in wave 1 — they are pure modules and a wire layer.
Wave 2 splits by *file ownership*, not by feature, so no two agents edit the same file:
T5 owns `site_card.py` + `dashboard_page.py`, T6 owns `nav_row.py`, T7 and T8 own one rover
node each. T9 is alone in wave 3 because `main_window.py` is edited by exactly one task and
it needs the widget (T5), the display hook (T6) and the wire (T4) to already exist. T11 is
last because it drives the whole GS chain.

**The one collision.** T7 and T8 both add a line to
`rover/src/navi_localization/setup.py` (`entry_points`) and possibly to `package.xml`. Expect
a one-line conflict; resolve it by keeping **both** lines. Nothing else is shared.

**The two files outside the create-list that get touched, and by whom.** T3 adds
`transform_point` to `rover/src/navi_localization/navi_localization/pose_composition.py`
(it has `compose`, `inverse` and a private `_rotate`, but no public point transform) and a
case to that package's existing `test/test_pose_composition.py`. T9 adds one `clicked` signal
to `ground_station/ui/video_panel.py`. Neither file is touched by any other task in any wave,
so both are still single-owner — they were simply missing from §4 in round 1.

---

## 6. Tasks

---

### Task 1 — `site_frame.py`: the rigid 2D fit

**Wave 1. Depends on nothing.**

**Create** `ground_station/site_frame.py` and `tests/test_site_frame.py`.

Implement §3.2 exactly: the dataclasses, `SiteFrameError`, and the five functions. Stdlib
only — `math` and `dataclasses`. No numpy, no scipy, no iterative solver. The closed form in
§3.2 is the whole algorithm; it is four sums and an `atan2`.

Write a module docstring that says, in prose an operator could follow: what site and map are,
which direction the transform goes, that there is no scale because the judges' metres and the
rover's metres are the same metres, that collinear landmarks are fine and only coincident ones
are not, and what the residual means physically (how far the fitted transform puts a landmark
from where the rover actually saw it).

**Tests — write them first.** `tests/test_site_frame.py`, no Qt, no fixtures beyond plain
tuples:

1. A known rotation+translation: build 4 site points, apply a chosen `(x, y, yaw)` by hand to
   get the map points, solve, and assert the recovered transform matches to 1e-9 and `rms_m`
   is ~0.
2. Round trip: `map_to_site(t, *site_to_map(t, 3.0, -4.0))` returns `(3.0, -4.0)` to 1e-9.
3. Exactly 2 points solves, and `rms_m` is 0 when their separation matches.
4. Two points whose measured separation is 0.2 m longer than published → `rms_m == 0.1`
   (half the baseline error). Assert the number, and put the reasoning in a comment.
5. **Collinear 3 points solve normally** — the test name should say so out loud, e.g.
   `test_three_collinear_landmarks_are_not_degenerate`.
6. One outlier among four: the fit still returns, `worst_id` is the outlier's id, and
   `max_residual_m` is close to the injected error. Dropping that pair and re-solving gives
   `rms_m` ~0.
7. `yaw` is wrapped: a 190° rotation comes back as −170°, not +190°.
8. `site_yaw_to_map_yaw` adds and wraps.
9. Every row of the edge-case table in §3.2 raises `SiteFrameError` with a message that
   mentions the offending id where the table says it does. Assert on a substring, not the
   whole sentence.
10. `scale_hint` is 1.0 for a clean fit and ≈ 1.1 when every measured baseline is 10 % long.
11. Non-finite input raises rather than producing a NaN transform.

**Test command**

```
.venv/bin/python3 -m pytest tests/test_site_frame.py -q
.venv/bin/python3 -m pytest tests/ -q          # must still be >= 354 passed
```

**Acceptance**
- `ground_station/site_frame.py` imports nothing but `math` and `dataclasses`
  (`grep -n '^import\|^from' ground_station/site_frame.py` shows only those).
- All eleven behaviours above have a test and it passes.
- Full GS suite green, count not below 354.
- One commit, house-style message.

---

### Task 2 — `landmark_table.py`: the judges' site map as a file

**Wave 1. Depends on nothing.**

**Create** `ground_station/landmark_table.py`, `tests/test_landmark_table.py`,
`docs/site/landmarks.example.json`.

Implement §3.3 exactly. Stdlib only — `json`, `dataclasses`, `pathlib`. Every error message is
a sentence an operator reads under a tent in the sun with a laptop on their knees: it names
the entry (`"landmark 3 ('52') has no 'y'"`) and says what to do.

`docs/site/landmarks.example.json` holds three landmarks with plausible marsyard coordinates
and a `marker` block with the provisional values (`DICT_5X5_100`, `0.150`, `0.125`, `0.417`)
and a `note` on one entry. Add a `"site_name": "EXAMPLE — replace before the run"` so nobody
drives against it by accident.

**Tests — write them first.**

1. The example file at `docs/site/landmarks.example.json` loads via `load_landmark_table` and
   yields three landmarks and the documented `MarkerSpec`. **This test is what keeps the
   shipped example valid.**
2. `parse_landmark_table` round-trips through `landmark_table_json`.
3. `by_id` finds by string id and returns `None` for an unknown one.
4. An absent `marker` block yields every default.
5. A partial `marker` block (only `dictionary`) keeps the other defaults.
6. Unknown extra keys at the top level and inside a landmark are ignored, not an error.
7. Each error in §3.3 raises `LandmarkTableError`, asserted on a substring, with a test per
   error: not JSON, not an object, bad `schema`, missing `landmarks`, empty `landmarks`,
   missing `id`, non-string `id`, duplicate `id`, missing/non-finite `x`, wrong `frame`,
   wrong `units`.
8. `note` survives verbatim including characters that would be markup (`"<b>north</b>"`) —
   the loader never sanitises; the widget sets PlainText.

**Test command**

```
.venv/bin/python3 -m pytest tests/test_landmark_table.py -q
.venv/bin/python3 -m pytest tests/ -q
```

**Acceptance**
- No `yaml` import anywhere (`grep -rn yaml ground_station/` finds nothing new).
- `docs/site/landmarks.example.json` exists and its loading test passes.
- Every error case in §3.3 has a test.
- Full GS suite green. One commit.

---

### Task 3 — `landmark_geometry.py`: a pixel becomes a landmark

**Wave 1. Depends on nothing.**

**Create** `rover/src/navi_localization/navi_localization/landmark_geometry.py` and
`rover/src/navi_localization/test/test_landmark_geometry.py`.

This module is the shared arithmetic of stages 2 and 3. It imports **`math` and `dataclasses`
only** — no `rclpy`, no `zed_msgs`, no `cv2`, no numpy. It must import cleanly on a laptop with
no ROS sourced at all. That is the whole reason it exists as a separate file.

Reuse `navi_localization.pose_composition` for the frame arithmetic
(`Transform`, `compose`, `inverse`, `CAMERA_IN_BASE_FOOTPRINT`) — read that file first. If it
has no point-transform helper, add `transform_point(t: Transform, p) -> tuple[float,float,float]`
there (rotate by the quaternion, add the translation) with its own test; do not reimplement
quaternion maths in the new module.

**API**

```python
@dataclass(frozen=True)
class Intrinsics:
    fx: float; fy: float; cx: float; cy: float; width: int; height: int
    def scaled_to(self, width: int, height: int) -> "Intrinsics":
        """Same camera, different published image size. All four numbers
        scale linearly; this is how a click on a 640x360 video stream is
        read against a 1280x720 depth image."""

# The LEFT optical frame (z forward, x right, y down) expressed in
# base_footprint. The rotation is the fixed optical->body convention.
#
# The translation is NOT CAMERA_IN_BASE_FOOTPRINT's. That constant is the
# camera BODY centre - pose_composition.py says so: it is the URDF's
# zed_front_camera_joint, the 1/4" mounting screw. Depth and the rectified
# left image are both expressed at the LEFT lens, which on a ZED 2i sits
# half the 120 mm stereo baseline to the left of the body centre, i.e.
# +0.060 m in base_footprint y. Sixty millimetres is half the 125 mm offset
# this plan bothers to correct for and it is systematic, not noise, so it is
# not dropped: write it as a named term, not a magic number.
ZED_BASELINE_M = 0.120
CAMERA_OPTICAL_IN_BASE_FOOTPRINT: Transform   # y = 0.0 + ZED_BASELINE_M / 2

def rescale_pixel(u, v, from_wh, to_wh) -> tuple[float, float]: ...

def ray_in_optical(u, v, intr: Intrinsics) -> tuple[float, float, float]:
    """Unit vector from the optical centre through pixel (u, v)."""

def point_in_optical(u, v, range_m, intr) -> tuple[float, float, float]:
    """range_m along that unit ray. Note: `range_m` is a RANGE along the
    ray, not a z-depth. The ZED publishes z-depth in its depth image, so
    `depth_to_range` converts before this is called."""

def depth_to_range(u, v, depth_m, intr) -> float:
    """z-depth (what the depth image holds) -> range along the ray."""

def apply_face_offset(range_m: float, offset_m: float) -> float:
    """The pole-axis correction of D4: the marker face is `offset_m` in
    FRONT of the pole axis, so the axis is `offset_m` FURTHER along the same
    ray. Because the inward normal is taken as the camera-to-marker ray, the
    whole correction is one addition. Pass 0.0 for a manual pole click."""

def landmark_point_in_map(u, v, depth_m, intr,
                          footprint_in_map: Transform,
                          offset_m: float = 0.0,
                          optical_in_footprint: Transform = CAMERA_OPTICAL_IN_BASE_FOOTPRINT
                          ) -> tuple[float, float, float]:
    """The whole chain: pixel + z-depth -> range -> pole-axis range ->
    optical point -> base_footprint -> map."""

def median_depth(patch: Sequence[float], min_m: float, max_m: float
                 ) -> tuple[float | None, int, float]:
    """(median, n_valid, valid_fraction) over a flat patch of depth values,
    dropping NaN, inf and anything outside [min_m, max_m]. Returns
    (None, n, frac) when nothing is valid. The median, not the mean: one
    background return through a gap beside a 6 cm pole would drag a mean
    metres away."""

class SightingAccumulator:
    def __init__(self, max_samples: int = 150,
                 min_samples: int = 50,
                 spread_warn_m: float = 0.15): ...
    def add(self, id: str, x: float, y: float, z: float, t: float) -> None
    def reset(self) -> None
    def ids(self) -> list[str]
    def snapshot(self, now: float) -> list["AccumulatedSighting"]

@dataclass(frozen=True)
class AccumulatedSighting:
    id: str; x: float; y: float; z: float
    n: int; spread_m: float; last_seen_s: float; quality: str
```

`SightingAccumulator` keeps a per-id ring buffer of the most recent `max_samples` points.
`snapshot` returns the **component-wise median** per axis, `spread_m` as
`1.4826 · median(|p_i − p_median|)` (2D distance in the xy plane; z is reported but not part
of the spread), and `quality` per §3.5 with `weak` winning over `noisy`.

**Tests — write them first.** All pure, all laptop-runnable:

1. `Intrinsics.scaled_to` halves fx, fy, cx, cy and the size (the 1280×720 → 640×360 case
   this project actually has), and is idempotent when the size is unchanged.
2. `rescale_pixel` maps the centre to the centre and a corner to the corner.
3. `ray_in_optical` at the principal point is `(0, 0, 1)`; its length is 1 for an
   off-centre pixel.
4. `depth_to_range` equals the depth at the principal point and is larger off-centre, by the
   exact factor `1/cos(angle)` — assert the number.
5. `apply_face_offset(4.0, 0.125) == 4.125`; `apply_face_offset(4.0, 0.0) == 4.0`.
6. `landmark_point_in_map` with an identity `footprint_in_map` and a centre pixel puts the
   point straight ahead of the **left lens**: `x = 0.345 + range`, `z = 0.548`, and
   `y = CAMERA_OPTICAL_IN_BASE_FOOTPRINT.y` (0.060, half the stereo baseline). **Derive the
   expectation from the constant, never from a hand-typed `0.0`** — a literal zero here is
   how the 60 mm lens offset gets frozen into a test and stops being findable. Assert the
   `x` and `z` literals, which are the thing that catches an optical-frame axis error, and
   assert `y` against the constant.
7. The same with the rover yawed 90° in map: the point moves onto the map `+y` axis.
8. A landmark measured with `offset_m = 0.125` lands 0.125 m further from the camera than the
   same pixel with `offset_m = 0`, along the same bearing.
9. `median_depth` drops NaN, inf, and out-of-range values; returns the median of what is left;
   returns `(None, 0, 0.0)` for an all-invalid patch; and is unmoved by a single wild outlier
   (put a 60.0 in a patch of 4.1s and assert the median is still ~4.1).
10. `SightingAccumulator`: median converges on the true point under symmetric noise; the ring
    buffer never exceeds `max_samples`; `quality` is `weak` below `min_samples`, `noisy` above
    `spread_warn_m`, `weak` when both; `last_seen_s` is `now − t_last`; `reset` empties it.
11. **The import test:** `import navi_localization.landmark_geometry` succeeds in a process
    where `rclpy`, `cv2` and `zed_msgs` are all absent. Assert
    `"rclpy" not in sys.modules and "cv2" not in sys.modules` after the import.

**Test command**

```
bash -c 'source /opt/ros/humble/setup.bash && \
  PYTHONPATH=$PWD/rover/src/navi_localization:$PYTHONPATH \
  python3 -m pytest rover/src/navi_localization/test/test_landmark_geometry.py -q -p no:cacheprovider'
```

(If you also touched `pose_composition.py`, add `test/test_pose_composition.py` to that
command — it is laptop-safe too.)

**Acceptance**
- The module imports with no ROS sourced at all: `python3 -c "import sys; sys.path.insert(0,'rover/src/navi_localization'); import navi_localization.landmark_geometry"` succeeds from the repo root.
- All eleven behaviours tested and passing.
- `grep -n 'import' rover/src/navi_localization/navi_localization/landmark_geometry.py`
  shows only `math`, `dataclasses`, `typing` and `navi_localization.pose_composition`.
- One commit.

---

### Task 4 — the wire: parsers, encoders, and four `ros_client` methods

**Wave 1. Depends on nothing.**

**Modify** `ground_station/models.py`, `ground_station/ros_client.py`,
`tests/test_models.py`, `tests/test_ros_client.py`.

Read `parse_nav_status`, `parse_path_summary`, `nav_request_json` and `new_run_id` in
`models.py`, and `subscribe_nav_status` / `send_nav_request` in `ros_client.py` first. Follow
those patterns to the letter — the lazy `_topic_factory` on first publish, the
`if not self.is_connected: print(..., file=sys.stderr); return` guard, the stored topic
handle, the `Signal(object)` on `self.signals`, the `std_msgs/String` type string.

Implement §3.6. Both parsers return `None` for anything malformed rather than a partly built
object — a truncated JSON frame from a lossy link must not produce a landmark at (0, 0).
Coerce with the existing `_safe_float` / `_safe_int` helpers; add `_safe_str` if you need one.

**Tests — write them first.**

In `tests/test_models.py`:
1. A full valid `/site/probe_result` parses into the documented `ProbeResult`.
2. A failure result (`ok: false`, null coordinates, an `error` string) parses, `ok` is False,
   coordinates are `None`, `error` survives verbatim.
3. Malformed payloads return `None`: not JSON, not an object, `sightings` not a list, a
   sighting with a non-numeric `x`, a missing `request_id`.
4. `parse_sightings` on a full report gives the phase, the dictionary, `detector_ok` and a
   list of `Sighting`s in wire order.
5. `parse_sightings` with `detector_ok: false` keeps the `error` and yields an empty list.
6. `probe_request_json` produces exactly the §3.4 keys, `patch_px` clamped odd and in
   [1, 51], `target` defaulting to `"pole"`, an unknown `target` rejected (raise `ValueError`
   — this is a programming error, not operator input).
7. `anchor_command_json` accepts only `start` / `stop` / `reset`.
8. `new_probe_id` is unique for the same timestamp with a different counter, and is a `str`.

In `tests/test_ros_client.py` (follow the existing fake topic-factory fixtures there):
9. `subscribe_probe_result` creates a `std_msgs/String` subscription on `/site/probe_result`
   and a delivered message emits `probe_result_received` with a parsed `ProbeResult`.
10. Same for `subscribe_landmark_sightings` on `/site/landmark_sightings`.
11. `send_probe_request` while disconnected publishes nothing and does not raise.
12. `send_probe_request` while connected publishes on `/site/probe_request` with the JSON of
    §3.4, and creates the topic exactly once across two calls.
13. Same pair for `send_anchor_command`.

**Test command**

```
.venv/bin/python3 -m pytest tests/test_models.py tests/test_ros_client.py -q
.venv/bin/python3 -m pytest tests/ -q
```

**Acceptance**
- No `rclpy` import appears anywhere under `ground_station/`
  (`grep -rn 'import rclpy' ground_station/` is empty).
- All thirteen behaviours tested and passing; full GS suite green.
- The four `ros_client` methods have docstrings in the register of the existing ones — a
  sentence about what the topic carries and one about why it is shaped that way.
- One commit.

---

### Task 5 — the SITE drawer

**Wave 2. Depends on T1 (`site_frame`) and T2 (`landmark_table`).**

**Create** `ground_station/ui/site_card.py`, `tests/test_site_card.py`.
**Modify** `ground_station/ui/dashboard_page.py`.

Read `ground_station/ui/node_list_widget.py`, `ground_station/ui/nav_row.py` and
`ground_station/theme.py` first.

**Where it goes, and why.** The dashboard's outer layout is
`QHBoxLayout(self)` → `left` (stretch 3) + `node_list`. `left` is
`mode_row / stage / map_row / drive_row / bottom`, and `stage` is the protected area: the
camera and the plan grid side by side across the full upper half, with the bottom row already
capped at `BOTTOM_CARD_HEIGHT` precisely so it cannot eat into the stage. The SITE card
therefore becomes a **second right-hand drawer, a twin of `node_list`**: added to the outer
layout beside it, `setVisible(False)` at construction, and shown by a header toggle (wired in
T9). Reasoning: anchoring is a once-per-mission job like reading the node list, it needs
vertical room for a landmark list with residuals, and hidden it costs the camera and the plan
exactly nothing. Do not put it in `bottom` — that row is height-capped and already holds the
waypoint editor and the wheel box.

In `dashboard_page.py`: construct `self.site_card = SiteCard()`, `setVisible(False)`, and
`layout.addWidget(self.site_card)` after `node_list`. Nothing else in that file changes.

Implement §3.7. The card's own logic:

- `set_table(table)` fills `landmark_list` with one checkable row per landmark, each showing
  `id`, the site x/y to 2 dp, and the `note` if present — **PlainText**. All rows start ticked.
  `table_label` shows `site_name` and the count. `state_pill` → `"0 OF 3 MEASURED"`.
- `set_measurement(id, x, y, quality)` stores the map-frame measurement for that id and
  updates its row: a measured landmark shows its residual after a solve and its `quality`.
  A measurement for an id that is not in the table is **kept and shown greyed with
  "not in table"** — it is exactly the information an operator needs when the dictionary or
  the id list is wrong. It is never included in a fit.
- `solve_requested` → the *window* is not involved: the card builds the `LandmarkPair` list
  from ticked ∩ in-table ∩ measured, calls `solve_site_to_map`, stores `self.transform`, and
  fills `rms_pill` / `detail_label` / the per-row residuals. On `SiteFrameError` it puts the
  message in `rms_pill`, leaves `self.transform` as it was, and does not emit `lock_changed`.
- `lock_button` toggled on → only enabled when `self.transform is not None`; sets
  `self.locked = True`, emits `lock_changed(self.transform)`, disables `solve_button`,
  `probe_button`, `anchor_button` and every tick box, and sets `state_pill` to `"LOCKED"`.
  Toggled off → `self.locked = False`, emits `lock_changed(None)`, re-enables everything.
  **Locking is a deliberate one-way gesture the operator can undo before Go and would not
  undo during a run** — the window is what refuses to unlock mid-run (T9).
- The three warnings of §3.7.

**Tests — write them first.** `tests/test_site_card.py`, in the style of
`tests/test_nav_row.py` (pytest-qt, `qtbot`, poke attributes, assert behaviour):

1. A fresh card says `"NO TABLE"` and `solve_button` / `probe_button` / `lock_button` are
   disabled.
2. `set_table` with three landmarks gives three rows, all ticked, and `"0 OF 3 MEASURED"`.
3. A `note` containing `"<b>x</b>"` appears literally in the row text — assert the text, and
   assert `textFormat` is PlainText on whatever widget renders it.
4. One measurement → `"1 OF 3 MEASURED"`, `solve_button` still disabled.
5. Two measurements → `solve_button` enabled; pressing it produces a `self.transform` and an
   RMS in `rms_pill`.
6. Feed measurements that are an exact rigid image of the table → `rms_pill` shows ~0.00 m.
7. Feed a deliberately wrong third measurement → `detail_label` names that landmark as the
   worst residual, and unticking it and re-solving drops the RMS.
8. Exactly two landmarks → `detail_label` carries the two-landmark caveat.
9. `scale_hint` off by 15 % → the scale warning appears.
10. Measurement for an unknown id → the row appears, says "not in table", and is **not** in
    `transform.ids` after a solve.
11. `lock_button` cannot be checked with no transform; after a solve, checking it emits
    `lock_changed` with the `SiteTransform` and disables the editing controls; unchecking
    emits `lock_changed(None)` and re-enables them.
12. `apply_sightings` with a `SightingsReport` of two `good` sightings produces two
    measurements; a `detector_ok: false` report puts its `error` on `state_pill` and adds no
    measurements.
13. `apply_probe_result` with `ok: false` puts the `error` where the operator can see it and
    adds no measurement.
14. `DashboardPage().site_card` exists and starts hidden, and `DashboardPage().nav_row` and
    `video_panel` are unchanged — a regression assertion that the stage did not move.

**Test command**

```
.venv/bin/python3 -m pytest tests/test_site_card.py -q
.venv/bin/python3 -m pytest tests/ -q
```

**Acceptance**
- Card styling goes through `theme.card_style()`, `theme.pill_style()`,
  `theme.section_title_style()` — no hand-rolled hex colours in `site_card.py`
  (`grep -n '#[0-9a-fA-F]\{6\}' ground_station/ui/site_card.py` is empty).
- Every wire-sourced string is set as PlainText.
- All fourteen behaviours tested and passing; full GS suite green.
- `site_card.py` imports no `rclpy` and does not import `ros_client`.
- One commit.

---

### Task 6 — the NAV row knows when its numbers are site numbers

**Wave 2. Depends on T1.**

**Modify** `ground_station/ui/nav_row.py`, `tests/test_nav_row.py`.

Read `nav_row.py` first, in particular `refresh_waypoints`, which currently does
`self.map_view.set_waypoints(self.waypoints.items)` and
`self.waypoints_changed.emit(self.waypoints.items)`.

**Keep this minimal — it is display only.** Add:

```python
def set_site_transform(self, transform) -> None:
    """A locked site->map transform, or None. The waypoint LIST stays in the
    numbers the operator typed; only the map drawing and the labels change,
    because the canvas draws the map frame and always will. The conversion
    that reaches the rover happens once, in MainWindow._on_go_requested."""
```

When a transform is set:
- `self.map_view.set_waypoints(...)` receives **converted** waypoints (site→map) so the dots
  land where the rover will actually go;
- the waypoint list rows and the editor's section title gain a `"(site)"` marker so the
  operator can see which grid they are typing in. ⚠ `editor_title` is currently a **local**
  `QLabel("WAYPOINTS")` in `__init__` (`nav_row.py:154`) — promote it to `self.editor_title`
  first, that is the whole change;
- `go_requested` still emits the **unconverted** (site) waypoints. The window converts.

**And the other direction, which is the easy one to miss.** `self.map_view.point_clicked` is
connected to `append_world_point` and delivers **map**-frame world coordinates. With a
transform set, a click on the canvas would drop a map number into a list the window then
converts *again*, and the waypoint lands somewhere nobody pointed at. So:

```python
def append_world_point(self, x: float, y: float) -> None:
    """The map view's entry point: a click on the canvas appends a waypoint
    exactly as typing coordinates and pressing Add would - which means it
    has to arrive in the same frame the operator types in. The canvas draws
    the map frame, so with a site transform locked the click is converted
    BACK to site here. §3.9: the list always holds the operator's frame."""
    if self._site_transform is not None:
        x, y = map_to_site(self._site_transform, x, y)
    self.waypoints.add(Waypoint(x, y))
    self.refresh_waypoints()
```

With no transform (`None`, the default and today's state) **every one of these paths is
exactly today's behaviour** — the branch is not taken and the method is the one that is
there now.

The double use of the transform — the row for drawing, the window for the wire — is
deliberate and is the reason both are pinned by tests. Say so in the docstring.

**Tests — write them first**, appended to `tests/test_nav_row.py`:

1. With no transform, `refresh_waypoints` passes the waypoints to `map_view.set_waypoints`
   unchanged, and the row text has no `"site"` in it — the regression assertion.
2. With a transform set, `map_view.set_waypoints` receives converted coordinates; assert the
   numbers against `site_frame.site_to_map`.
3. With a transform set, `go_requested` still carries the **site** numbers the operator typed.
4. `set_site_transform(None)` restores case 1 exactly.
5. The list rows and section title show the site marker only when a transform is set.
6. **The canvas round trip.** With a transform set, emitting `map_view.point_clicked(mx, my)`
   appends a waypoint whose stored coordinates are `map_to_site(t, mx, my)`, **and** the dot
   the canvas then receives back through `refresh_waypoints` is `(mx, my)` again to 1e-9.
   A click lands where it was clicked; that is the whole assertion.
7. The same emission with no transform appends `(mx, my)` verbatim — today's behaviour.

**Test command**

```
.venv/bin/python3 -m pytest tests/test_nav_row.py -q
.venv/bin/python3 -m pytest tests/ -q
```

**Acceptance**
- Behaviour with `site_transform is None` is unchanged, pinned by test 1 and test 4.
- `nav_row.py` still imports nothing from `ros_client` and no `rclpy`.
- Full GS suite green. One commit.

---

### Task 7 — `site_probe.py`: the rover answers a click

**Wave 2. Depends on T3.**

**Create** `rover/src/navi_localization/navi_localization/site_probe.py`,
`rover/src/navi_localization/test/test_site_probe.py`.
**Modify** `rover/src/navi_localization/setup.py` (one `console_scripts` line),
`package.xml` if a dependency is genuinely new.

Read `localization_status.py` for the node shape: `declare_parameter` in `__init__`,
subscriptions created there, a thin `main()`, all arithmetic delegated to a pure module.

The node:

- Parameters (all `declare_parameter`, all with the defaults given):
  `depth_topic` (`/zed_front/zed_node/depth/depth_registered`),
  `camera_info_topic` (`/zed_front/zed_node/depth/camera_info`),
  `pose_topic` (`/localization/pose`),
  `status_topic` (`/localization/status`),
  `request_topic` (`/site/probe_request`), `result_topic` (`/site/probe_result`),
  `min_depth_m` (0.3), `max_depth_m` (10.0), `min_valid_fraction` (0.25),
  `face_offset_m` (0.125), `require_localisation_ok` (True).
  **The depth bounds mirror `zed_front.yaml`'s `depth.min_depth: 0.3` /
  `depth.max_depth: 10.0`** — the SDK returns nothing outside them, so a wider filter here
  buys nothing and a narrower one silently throws away good returns. If that config changes,
  these change with it; say so in a comment beside the declarations.
  `require_localisation_ok` compares `/localization/status`'s `state` field against the
  string `"ok"` — `tracker.py` publishes `off` / `searching` / `ok` and nothing else.
  **⚠ Verify `depth_topic` and `camera_info_topic` against the running wrapper before you
  trust them — that verification is item 2 of the bring-up checklist, not an acceptance
  criterion of this task.** They are parameters precisely so a wrong guess is a launch
  argument and not a code change.
- Subscribes: `sensor_msgs/Image` (depth), `sensor_msgs/CameraInfo`,
  `nav_msgs/Odometry` (pose), `std_msgs/String` (localisation status, request).
  Publishes `std_msgs/String` on `result_topic`.
- Keeps only the **latest** depth image and camera info. A probe is answered against whatever
  is current; there is no queue and no time synchronisation — the rover is stationary during
  anchoring and a 100 ms skew is millimetres.
- On a request: parse; validate; rescale `(u, v)` from the request's `width`/`height` to the
  depth image's actual size with `landmark_geometry.rescale_pixel`; take the `patch_px` ×
  `patch_px` window; decode the depth values; `median_depth`; then `landmark_point_in_map`
  with `offset_m = face_offset_m if target == "box_face" else 0.0`; publish the result.
- **Every request gets exactly one result**, success or failure, with the `request_id` echoed
  and one of the §3.4 error strings. A malformed request with no readable `request_id` is
  logged and dropped — that is the only silent case.
- Depth decoding: the ZED publishes `32FC1` metres. Support `32FC1` (`struct.unpack` /
  `array`, honouring `step` and `is_bigendian`) and reject any other `encoding` with the
  error `"unsupported depth encoding 'mono16'"`. **Do not import `cv_bridge`** — it is a
  heavyweight dependency for what is a strided read of a float buffer, and it would make this
  node untestable on the laptop.

**Tests — write them first.** `rover/src/navi_localization/test/test_site_probe.py`. This
file imports `rclpy` and `sensor_msgs`, so it runs with ROS sourced but **needs no graph and
no `zed_msgs`** — construct the node with `rclpy.init()` / `destroy_node()` the way
`test_localization_status.py` does, and drive it by calling its callbacks directly with
hand-built messages. Publish by monkeypatching the publisher with a recorder.

1. A synthetic 32FC1 depth image of a known constant, identity pose, centre pixel, `target`
   `"pole"` → the published result has `ok: true` and the coordinates
   `landmark_geometry.landmark_point_in_map` gives for the same inputs. Assert against the
   pure function, not against re-derived literals.
2. The same request with `target: "box_face"` lands 0.125 m further along the bearing.
3. A click given in 640×360 coordinates against a 1280×720 depth image resolves to the same
   world point as the equivalent 1280×720 click — the rescale test.
4. A patch that is all NaN → `ok: false`, `error == "no valid depth at that pixel"`,
   coordinates `None`.
5. A patch that is 20 % valid with `min_valid_fraction` 0.25 → the same failure;
   30 % valid → success.
6. A request before any depth image → `"no depth image yet"`. Before any pose →
   `"no rover pose yet"`.
7. Localisation status not OK with `require_localisation_ok` True → `"localisation is not
   OK"`; with the parameter False the probe proceeds.
8. `u` or `v` outside the image → `"pixel is outside the image"`.
9. An unsupported `encoding` → the encoding error, naming the encoding.
10. Garbage on the request topic (not JSON, JSON that is not an object, missing `u`)
    publishes nothing and does not raise — pin it with a recorder that asserts zero
    publishes and a node that is still alive afterwards.
11. Two requests in a row produce two results with the two matching `request_id`s.

**Test command**

```
bash -c 'source /opt/ros/humble/setup.bash && \
  PYTHONPATH=$PWD/rover/src/navi_localization:$PYTHONPATH \
  python3 -m pytest rover/src/navi_localization/test/test_site_probe.py -q -p no:cacheprovider'
```

**Acceptance**
- `grep -n 'zed_msgs\|cv_bridge\|import cv2' rover/src/navi_localization/navi_localization/site_probe.py`
  is empty — this node must be testable without any of the three.
- Every topic name and threshold is a declared parameter; none is a literal at a call site.
- Every request produces exactly one result except an unparseable one; all eleven behaviours
  tested and passing.
- `setup.py` gains `site_probe = navi_localization.site_probe:main`.
- One commit. **If `setup.py` conflicts with Task 8, keep both lines.**

---

### Task 8 — `site_anchor.py`: the ArUco anchor phase

**Wave 2. Depends on T3.**

**Create** `rover/src/navi_localization/navi_localization/site_anchor.py`,
`rover/src/navi_localization/test/test_site_anchor.py`.
**Modify** `setup.py` (one `console_scripts` line).

Read Task 7's node first if it exists; this one is its sibling and shares
`landmark_geometry`.

The node:

- Parameters: `image_topic` (`/zed_front/zed_node/left/image_rect_color`),
  `depth_topic`, `camera_info_topic`, `pose_topic`, `status_topic` (as Task 7),
  `command_topic` (`/site/anchor_command`), `sightings_topic`
  (`/site/landmark_sightings`),
  `dictionary` (`"DICT_5X5_100"`), `marker_edge_m` (0.150), `face_offset_m` (0.125),
  `max_samples` (150), `min_samples` (50), `spread_warn_m` (0.15),
  `report_interval_s` (1.0), `detect_interval_s` (0.2), `min_depth_m` (0.3),
  `max_depth_m` (10.0, mirroring `zed_front.yaml` as in Task 7), `range_source` (`"depth"`).
  **⚠ `image_topic` and the exact dictionary name must be verified on hardware
  (checklist items 2 and 3). They are parameters for that reason.**
- `cv2` is imported **lazily inside a `try`, at `start`, not at module import**:

  ```python
  try:
      import cv2
      self._aruco_dict = cv2.aruco.getPredefinedDictionary(
          getattr(cv2.aruco, self._dictionary_name))
  except Exception as exc:
      self._detector_ok = False
      self._detector_error = f"..."
  ```

  A missing OpenCV, or a dictionary name that is not an attribute of `cv2.aruco`, sets
  `detector_ok = False` and puts a sentence in `error`. It **never** raises out of a callback
  and never stops the node — the report is what tells the operator, and a node that died is
  a node that tells them nothing.
- Phase machine: `idle` → `start` → `running` → `stop` → `stopped`; `reset` clears the
  accumulator and returns to the current phase. In `idle` and `stopped` no detection runs.
- On each image while `running`, at most every `detect_interval_s`: detect markers; for each
  marker take the **centroid of its four corners** as the pixel; take the range from the
  depth image at that pixel via `median_depth` over a small patch (`range_source: "depth"`),
  falling back to nothing at all if the depth is invalid (skip the detection — do **not**
  guess a range); apply `apply_face_offset(range, face_offset_m)`; `landmark_point_in_map`;
  `accumulator.add(str(marker_id), ...)`.
  **`marker_edge_m` is carried and reported but not used for ranging** — the whole point of
  D3/correction 4 is that no pose estimation happens. It exists in the config because the
  operator needs the number recorded and because a future range fallback would want it.
- Publishes the §3.5 report every `report_interval_s` while not `idle`, and once on every
  phase transition.
- **Never publishes a twist, never publishes `/mode_request`, never publishes
  `/drive_command`.** The operator turns the rover by hand.

**Tests — write them first.** `test_site_anchor.py`. The detection step is the only part that
needs `cv2`; **structure the node so the tests never call it.** Give the node a seam:

```python
def _detect(self, image_msg) -> list[tuple[str, float, float]]:
    """[(id, u, v)] — the only method that touches cv2. Tests replace it."""
```

Tests replace `_detect` with a stub returning known ids and pixels. That makes every
behaviour below laptop-runnable without OpenCV.

1. `start` moves the phase to `running` and publishes a report immediately.
2. With `_detect` stubbed to return one marker at a known pixel and a synthetic constant
   depth image, 60 ticks produce a sighting with `n == 60`, `quality == "good"`, and
   coordinates matching `landmark_geometry` for the same inputs **including the 0.125 m
   offset**.
3. 30 ticks → `quality == "weak"` with `min_samples` 50.
4. Detections scattered by 0.4 m → `quality == "noisy"`, and the median is still near the
   true point (assert both).
5. Two markers → two sightings, ids as **strings**.
6. `stop` freezes the phase and stops accumulating; a further tick does not raise `n`.
7. `reset` empties the accumulator; the next report has no sightings.
8. A detection whose depth patch is all NaN is **skipped**, not accumulated at a guessed
   range — assert `n` did not rise.
9. `max_samples` caps the ring buffer.
10. With the dictionary set to a name `cv2.aruco` does not have, the node still constructs,
    `detector_ok` is False, the `error` names the dictionary, and reports keep being
    published.
11. **The no-motion assertion, and it is not optional:** after `start`, `stop`, `reset` and
    a hundred ticks, the node has created **no** publisher on `/rover_twist`,
    `/manual_twist`, `/autonomy_twist`, `/mode_request` or `/drive_command`. Enumerate the
    node's publishers and assert the topic set.
12. Garbage on `/site/anchor_command` (not JSON, unknown action) is ignored and does not
    change the phase.

**Test command**

```
bash -c 'source /opt/ros/humble/setup.bash && \
  PYTHONPATH=$PWD/rover/src/navi_localization:$PYTHONPATH \
  python3 -m pytest rover/src/navi_localization/test/test_site_anchor.py -q -p no:cacheprovider'
```

**Acceptance**
- The module imports and the node constructs on a machine with **no OpenCV** — test 10 is
  what proves it.
- No marker id and no dictionary name is a literal outside a parameter default and the tests.
- Test 11 passes: no motion publisher of any kind exists.
- All twelve behaviours tested and passing.
- `setup.py` gains `site_anchor = navi_localization.site_anchor:main`.
- One commit. **If `setup.py` conflicts with Task 7, keep both lines.**

---

### Task 9 — the window: header button, probe round-trip, and the conversion at Go

**Wave 3. Depends on T4, T5, T6.**

**Modify** `ground_station/ui/main_window.py`, `tests/test_main_window.py`.

Read `main_window.py` around lines 217–270 (the header, `link_button`, `nodes_button`,
`_on_nodes_toggled`), `_connect_to` (line 403), and `_on_go_requested` (line 787).

1. **Header toggle.** Beside `self.nodes_button`, a `self.site_button = QPushButton("Site ▸")`,
   checkable, tooltip *"Show or hide the site anchor: landmarks, the fit, and the lock."*,
   `toggled` → `_on_site_toggled`, which does
   `self.dashboard_page.site_card.setVisible(shown)` and flips the arrow, exactly as
   `_on_nodes_toggled` does. Insert it **before** `nodes_button` in the header layout, so the
   two drawers sit together and the STOP button keeps its hard-right position and its spacing.
2. **Subscriptions.** In `_connect_to`, inside the existing subscribe-before-connect `try`
   block and beside the other `subscribe_*` calls: `subscribe_probe_result()` and
   `subscribe_landmark_sightings()`; connect `probe_result_received` →
   `self.dashboard_page.site_card.apply_probe_result` and `sightings_received` →
   `...apply_sightings`.
3. **Probe round-trip.** `site_card.probe_requested(landmark_id, target)` →
   `_on_probe_requested`, which reads the click position the operator last made in the camera
   panel and sends `send_probe_request(...)` with a `new_probe_id`.

   **How the click arrives — and this task owns the file it needs.** Add to
   `ground_station/ui/video_panel.py` one signal and nothing else:

   ```python
   clicked = Signal(int, int, int, int)   # (u, v, width, height), SOURCE-frame pixels
   ```

   ⚠ **It must carry source-frame pixels, not label pixels.** `_render_frame` scales the
   pixmap with `Qt.KeepAspectRatio` into `self.image_label`, so the picture is **letterboxed**
   inside the label: a raw `event.position()` is offset by the bars and scaled by an unknown
   factor, and a probe built from it measures the wrong part of the world — quietly, with a
   plausible-looking number. So `mousePressEvent` on the `AspectLabel` must

   - take the rendered `pixmap()`'s size `(pw, ph)` and the label's size `(lw, lh)`,
   - subtract the letterbox origin `((lw - pw) / 2, (lh - ph) / 2)`,
   - reject a click outside `[0, pw) × [0, ph)` (in the bars) by emitting nothing,
   - scale by `receiver.width / pw` and `receiver.height / ph`,
   - emit `(u, v, receiver.width, receiver.height)`.

   Nothing else in the panel changes: no state, no second job, no reshaping. `width` and
   `height` on the wire are then the stream's own dimensions, which is exactly what §3.4's
   rescale contract needs, and the panel is honest even if the layout ever gives the label a
   non-16:9 box. The window remembers the last emission. With no click yet, the card's state
   pill says *"click the landmark in the camera view first"* and nothing is sent.

   Add the mapping cases to `tests/test_video_panel.py`: a centre click on a letterboxed
   render returns the centre of the source frame; a click in the bar emits nothing; a click
   at a known off-centre point returns the source pixel derived from the ratio, not the label
   coordinate.
4. **The lock.** `site_card.lock_changed(transform)` → `_on_site_lock_changed`, which stores
   `self._site_transform` and `self._site_locked`, calls
   `self.dashboard_page.nav_row.set_site_transform(transform)`, and updates the header pill
   text so a locked site frame is visible without opening the drawer (`"SITE: LOCKED
   RMS 0.06 m"` / nothing when unlocked).
5. **Refuse to change the transform mid-run.** If `_on_site_lock_changed` fires while
   `self._last_nav_state` is in `NAV_ACTIVE_STATES`, ignore it, put the button back where it
   was, and print a line to stderr. A transform that changes under a running plan would move
   goals the rover is already driving to. This is D2 made mechanical.
6. **The conversion**, §3.8, in `_on_go_requested`. Nothing else in that method changes.

**Tests — write them first**, appended to `tests/test_main_window.py` (follow the existing
fake-`ros_client` fixtures there):

1. **The regression, and it is the most important test in this plan:** with no transform, a Go
   sends exactly the waypoints the row emitted, with the same `run_id` shape, on the same
   topic, with the same JSON as today. Write it so it would fail if a single coordinate moved.
2. With a locked transform, a Go sends **converted** coordinates; assert against
   `site_frame.site_to_map` on the same inputs.
3. A waypoint with a yaw gets `site_yaw_to_map_yaw`; a waypoint with `yaw is None` still
   sends `null`.
4. Unlocking restores case 1.
5. `site_button` toggling shows and hides `dashboard_page.site_card` and flips its arrow text.
6. `_connect_to` subscribes both new topics; a delivered `SightingsReport` reaches
   `site_card.apply_sightings`.
7. `probe_requested` with no click recorded sends nothing; with a click recorded it sends one
   `send_probe_request` carrying the clicked pixel and its image size.
8. A `lock_changed` arriving while the nav state is `"running"` is refused: `_site_transform`
   is unchanged and no conversion happens on the next Go.
9. The header still contains the STOP button, and it is still enabled — the standing
   assertion this repo makes on every header change.

**Test command**

```
.venv/bin/python3 -m pytest tests/test_main_window.py -q
.venv/bin/python3 -m pytest tests/ -q
```

**Acceptance**
- With no locked transform, `_on_go_requested` is behaviourally identical to today (test 1).
- `main_window.py` gains no `rclpy` import and no import from `rover/`.
- The mid-run refusal (test 8) passes.
- Full GS suite green, count not below 354 plus the new tests.
- One commit.

---

### Task 10 — the anchor-mode camera config, and the operator's page

**Wave 3. Parallel with T9. Depends on nothing.**

**Create** `rover/src/navi_localization/config/zed_front_anchor.yaml`, `docs/site/README.md`.

**The config file is data. No code in this repo loads it, launches it, or switches to it.**
It exists so that the operator procedure in §8 is a file to point at rather than a paragraph
to retype at 6 a.m.

Copy `rover/src/navi_localization/config/zed_front.yaml` and change exactly three things,
each with a comment saying why and what it costs:

```yaml
general:
    pub_downscale_factor: 1.0     # 1280x720. At 2.0 a 150 mm 5x5 tag needs 25 px to decode
                                  # and gets fx/167 metres of range - 1.3 to 2.6 m depending
                                  # on which ZED 2i lens is fitted (see §0 correction 1).
                                  # Landmarks are 3-8 m away. Costs bandwidth the video
                                  # sender does not want during a run - which is why this
                                  # file is only for the stationary anchor phase.
region_of_interest:
    # NOT '[]'. The drive config's own comment says the top-half mask exists
    # because the sun shade above the lens would otherwise be tracked by VIO
    # as a "stationary" object that moves with the camera. The anchor phase
    # measures landmarks IN THE ZED'S MAP FRAME, so a VIO that goes bad here
    # does not degrade the anchor - it invalidates it, silently. Lower the
    # boundary instead of removing it: y = 0.35 buys ~11 degrees of headroom
    # at 720 rows, which is far more than the couple of degrees of pitch that
    # R2 is about, and still keeps the shade and most of the sky out.
    automatic_roi: false
    manual_polygon: '[[0.0,0.35],[1.0,0.35],[1.0,1.0],[0.0,1.0]]'
    apply_to_depth: true
```

**Checklist item 9 is what confirms 0.35 is low enough and not too low.** If the shade shows
above y = 0.35 on the real rover, raise the number until it does not, and record what you
used — a landmark measured while VIO is tracking the sun shade is worse than no landmark.

Optionally document (commented out, not enabled) `grab_resolution: 'HD2K'` with the range
arithmetic: HD2K roughly doubles fx again and pushes detection from ~4 m to ~7 m, at a frame
rate the Orin will not sustain while also running Nav2 — which is fine, because during the
anchor phase nothing else is running.

`docs/site/README.md` is written **for the operator, not for a developer**: the landmark table
format with the example inline, what each field means, the pole-axis convention (published
coordinate = pole axis at 0.417 m, not the face), and §8's competition-day procedure copied in
full. It should be readable on a phone.

**Tests.** There is nothing to unit-test in a YAML file the code does not read, so pin the
things that *can* rot:

`tests/test_landmark_table.py` (extend it, or a new `tests/test_site_docs.py`):
1. `docs/site/landmarks.example.json` parses (already Task 2's test — do not duplicate it;
   instead assert `docs/site/README.md` exists and contains the example's `site_name`, so a
   changed example forces the doc to be looked at).
2. `rover/src/navi_localization/config/zed_front_anchor.yaml` exists, is valid YAML, and its
   `pub_downscale_factor` is `1.0` while `zed_front.yaml`'s is `2.0`. `pyyaml` 6.0.3 **is**
   installed in `.venv` today but is **not** in `pyproject.toml`'s dependencies, so a bare
   `import yaml` at module scope would make the GS suite fail on a clean checkout. Either
   guard it (`pytest.importorskip("yaml")`) or assert on the text with a regex and say which
   you chose in a comment. The point of the test is that nobody quietly makes the two files
   the same.
3. The anchor config's `region_of_interest.manual_polygon` is **not** `'[]'` and its top
   boundary is above 0.0 — the mask is relaxed, not removed. This is R9 in a test: an
   implementer or a later editor "simplifying" it to an empty polygon is the failure mode.

**Test command**

```
.venv/bin/python3 -m pytest tests/ -q
```

**Acceptance**
- `grep -rn 'zed_front_anchor' rover/src/navi_localization/navi_localization/ rover/src/navi_localization/launch/`
  is **empty** — no code path references the anchor config.
- `docs/site/README.md` contains the full §8 procedure and the pole-axis note.
- The drift test passes. Full GS suite green. One commit.

---

### Task 11 — end to end, in the ground station

**Wave 4. Depends on T9.**

**Create** `tests/test_site_anchor_end_to_end.py`.

One test file that drives the whole ground-station chain against a fake `ros_client`, with no
rover, no ROS and no camera. This is the test that would catch a contract drift between two
tasks that never saw each other's code.

The scenario, as a single test plus a handful of variants:

1. Build a `MainWindow` with a fake ros client. Load `docs/site/landmarks.example.json` into
   the SITE card.
2. Choose a ground truth `(x, y, yaw)`. Compute, for each of the three example landmarks, the
   map position it would have under that transform. Feed them in as a
   `/site/landmark_sightings` payload — **as JSON text through
   `models.parse_sightings`**, not as pre-built objects, so the wire format is exercised.
3. Press solve. Assert the recovered transform matches the ground truth to 1e-6 and
   `rms_m` ≈ 0.
4. Lock.
5. Type three waypoints in **site** coordinates into the NAV row and press Go.
6. Assert the JSON that reached `ros_client.send_nav_request` carries the **map** coordinates
   — computed independently in the test from the ground-truth transform, not from
   `site_frame`. This is the one place where re-deriving the arithmetic by hand is the right
   call: it is what makes the test an independent check rather than a tautology.

Variants:
7. Same run with noise added to the sightings: `rms_m` is within a factor of 2 of the injected
   σ, and the Go coordinates are within the expected error of the truth.
8. Same run with one sighting given the **wrong id**: the RMS is visibly bad, `worst_id`
   names it, unticking it and re-solving fixes the fit, and the resulting Go is correct.
9. A stage-2 path: instead of sightings, three `/site/probe_result` payloads through
   `parse_probe_result` produce the same locked transform and the same Go.
10. **No anchoring at all:** the same three waypoints and a Go, with the SITE drawer never
    opened, produce byte-identical JSON to a `MainWindow` built without any of this plan's
    code paths touched. Assert the exact JSON string.
    ⚠ `nav_request_json` embeds a `run_id` from `new_run_id(time())`, so two windows cannot
    agree byte-for-byte unless the clock is pinned. **Monkeypatch the `time` that
    `main_window` imported** (`ground_station.ui.main_window.time`) to a constant for both
    halves of the comparison; do not "fix" it by excising `run_id` from the comparison, which
    is the one field that would hide a changed call signature.

**Test command**

```
.venv/bin/python3 -m pytest tests/test_site_anchor_end_to_end.py -q
.venv/bin/python3 -m pytest tests/ -q
```

**Acceptance**
- All ten assertions pass, and variant 10 in particular.
- The test imports nothing from `rover/` and nothing that needs ROS.
- Full GS suite green. One commit.

---

## 7. Risks

**R1 — the dictionary is wrong.** ERC's markers come from the arucogen generator, which the
rules describe as "not the original library". `DICT_5X5_100`, `DICT_5X5_250` and `DICT_5X5_50`
share their first 50 codewords, so an id in 51–64 may decode under one and not another. If the
dictionary is wrong, *nothing detects* — a total, silent stage-3 failure.
**Mitigations:** the dictionary is a parameter and a table field, never code (D6); the
sightings report carries `detector_ok` and the dictionary name so the GS shows what the rover
is actually looking for; checklist item 3 is to print the real tag and try every 5×5
dictionary against it. Stage 2 does not depend on the dictionary at all — that is the point of
building it first.

**R2 — the ROI eats the landmark's depth.** Quantified in §0: at 4 m the marker centre is
~2 % of the frame height inside the unmasked region, and the drive config masks everything
above the horizon *including depth*. A slight upward camera pitch, or a landmark on rising
ground, removes the depth entirely and stage 3 accumulates nothing while the detector happily
finds the marker. **Mitigations:** the anchor config relaxes the ROI (Task 10); the sightings
report distinguishes "no marker detected" from "detected but skipped for want of depth"
(Task 8, test 8) so the operator can tell these two apart in the field; checklist item 5.

**R3 — depth noise on a thin pole and a small target.** The pole is ~60 mm across; at 4 m in a
640-wide image it is under 4 px, so a patch centred on it will contain background returns.
The medians in `median_depth` and in the accumulator are the defence, and `spread_m` is the
number that tells the operator the defence is failing. The 0.125 m offset is itself the same
order as the expected noise, which is fine for a transform fitted over a 10 m baseline: a
0.1 m error on a landmark 15 m from its neighbour is a 0.4° yaw error, i.e. 0.14 m of
cross-track at 20 m. Say this in the SITE card's tooltip so the operator can calibrate their
worry.

**R4 — two landmarks is a fit with no check.** With exactly 2 points the residual only sees a
baseline-length mismatch; a pair of ids swapped between two equidistant poles gives a perfect
RMS and a transform that is wrong by the angle between them. The rules guarantee ≥ 2 visible,
not ≥ 3. **Mitigation:** the card says so in words (§3.7), and the ops procedure (§8) tells
the operator to walk-measure a third point with the tape if only two markers are reachable —
a hand-entered landmark is a first-class citizen because ids are strings.

**R5 — the sim cannot test any of this.** Gazebo has no ArUco boxes and the sim's camera is
not the ZED. Stages 2 and 3 are therefore verified by unit tests against synthetic depth
buffers and stubbed detections, and by the field experiment in the checklist — **not** in the
simulator. Do not spend time trying to make the sim prove stage 3; it cannot, and the
checklist is honest about that.

**R6 — resolution switching becomes a runtime feature by accident.** The temptation, once the
anchor config exists, is to have the anchor node restart the wrapper. It must not: a node that
can restart the camera is a node that can take the camera away during a run.
**Mitigation:** Task 10's acceptance criterion is a `grep` that must come back empty.

**R7 — the transform changes under a running plan.** Guarded mechanically in Task 9 step 5 and
tested. Worth restating because it is the failure that would put the rover somewhere nobody
asked for.

**R8 — an ERC Update Report changes the marker ids or the landmark heights.** Everything
id-shaped is data (D6) and the 0.417 m centre height and 0.125 m offset are both table fields
with defaults, so a rules change is a JSON edit and a redeploy of nothing. Keep it that way.

**R9 — relaxing the ROI corrupts the frame the anchor is measured in.** The drive config's
top-half mask is not tidiness: `zed_front.yaml` says the sun shade above the lens *"MUST be
masked out or VIO would track a 'stationary' object that moves with the camera."* Stage 3
measures landmarks in the ZED's `map` frame, so a VIO that drifts during anchoring produces
landmark positions that are individually plausible and collectively wrong — a fit with a
respectable RMS and a real heading error. This is worse than stage 3 returning nothing.
**Mitigations:** the anchor config lowers the ROI boundary to y = 0.35 rather than removing
it (Task 10), Task 10's third test refuses an empty polygon, and checklist item 9 confirms on
hardware that the shade is still masked and `/localization/status` stays `ok` throughout an
anchor phase. If it does not, anchor with the drive ROI at shorter range — a landmark at 2 m
is measured in a frame you can trust, and two trustworthy landmarks beat four drifting ones.

**R10 — `fx` is unknown until the rover is on.** Every range number in this plan is a
bracket, not a measurement (§0 correction 1): the two ZED 2i lens variants differ by a factor
of two and the sim's `horizontal_fov` is not a calibration. Nothing in Tasks 1–11 depends on
the value — it arrives from `camera_info` at runtime and `Intrinsics` is built from the
message — but the *operator procedure* does: whether 1280×720 suffices or HD2K is needed is
answered by checklist item 4 and nowhere else. Do not let a number from this document reach
`docs/site/README.md` as if it were measured.

---

## 8. Hardware bring-up checklist

**None of this is a task and none of it blocks Tasks 1–11.** The Orin is offline while this
work happens. Every item below is done on the rover, with the camera, before the competition,
and each has a place to write the answer down.

1. **Deploy.** From the repo root: `./deploy_rover.sh` (rsync + colcon on the Orin) or, by
   hand, `rsync -az rover/ star@a_navi:navi/` then
   `ssh star@a_navi 'source /opt/ros/humble/setup.bash && cd navi && colcon build --packages-select navi_localization --symlink-install'`.
   Then `./deploy_rover.sh --test` and confirm the full `navi_localization` suite (which needs
   `zed_msgs`) is green on the Orin.
2. **Verify every topic name.** With the wrapper running:
   `ros2 topic list | grep zed_front`. Confirm, and write into the launch defaults if they
   differ: the depth image topic, its `camera_info`, and the rectified left colour image.
   `ros2 topic echo /zed_front/zed_node/depth/camera_info --once` gives the real fx, fy, cx,
   cy — check them against what `Intrinsics` receives.
   `ros2 run tf2_ros tf2_echo base_footprint zed_front_left_camera_optical_frame` gives the
   true optical-frame offset; compare with `CAMERA_OPTICAL_IN_BASE_FOOTPRINT` and correct the
   constant if they disagree by more than a centimetre.
3. **Find the dictionary.** Print one 150 mm tag from the ERC generator. On the Orin, loop
   `cv2.aruco.getPredefinedDictionary` over every `DICT_5X5_*` and every `DICT_4X4_*` against
   a still of the printed tag; record which ones decode and to what id. Put the winner in the
   landmark table's `marker.dictionary`. **Do this before the range experiment.**
4. **The range experiment (R5's answer).** Printed 150 mm tag on a stand at marker height
   (0.417 m centre). Walk it out in 0.5 m steps from 1 m and record, at each distance, for
   both the drive config and the anchor config: does the marker decode, and does the depth
   patch at its centroid return a valid range? Write the two numbers — decode range and depth
   range — into `docs/site/README.md`. **Expected, from §0 correction 1's bracket and the fx
   you wrote down in item 2: `0.150·fx/25` metres, so 1.3–2.6 m at 640×360 and 2.7–5.3 m at
   1280×720.** Landmarks are 3–8 m away, so if the measured 1280×720 range lands at the
   bottom of that bracket the anchor procedure needs HD2K and §9's step 5 changes. Note that
   the depth range is capped at 10 m by `depth.max_depth` regardless of what decodes.
5. **The ROI check (R2).** With the *drive* config running, put the tag at 4 m and confirm
   whether depth exists at its centroid. Then repeat with the anchor config. Photograph both
   depth images. This is the item most likely to change the plan.
6. **The HD2K anchor procedure.** The frame question is answered offline (review round 3):
   `area_memory_db_path` is empty, so the map frame does NOT survive a wrapper restart — it
   is reborn at the rover's pose at relaunch. That is exactly what §3.10's re-expression is
   built on. The hardware item is therefore to verify the re-expression end to end and time
   the cycle: anchor config in, anchor phase, Lock, wrapper restart with `zed_front.yaml`,
   **Camera restarted** pressed with the rover untouched — then drive to a taped mark whose
   site coordinates are known and measure the arrival error. Repeat once with a deliberate
   90° gamepad sweep during the anchor phase, which is the case that used to invalidate the
   lock silently.
7. **A live probe.** Ground station connected, `site_probe` running, click a real landmark in
   the camera view, and check the returned map position against a tape measure from the
   rover. Twice, at 2 m and 5 m.
8. **A live anchor phase.** Two markers in view, `start`, watch `n` climb and `spread_m`
   settle, `stop`, solve, and compare the resulting transform against the rover's known
   placement. Record the RMS you actually get — that number is what the operator will judge
   "good" against on competition day.
9. **The ROI-vs-VIO check (R9), and it gates the anchor config.** With the sun shade fitted
   and the *anchor* config running, look at the left image and confirm the shade is still
   below the ROI's top boundary (y = 0.35 of the frame height, ~252 rows down in a 720-row
   image). Then leave the rover stationary for two minutes in sun and watch
   `/localization/status` and `/localization/pose`: the state must stay `ok` and the pose
   must not walk. Repeat while turning slowly on the spot, which is what the operator
   actually does in §9 step 7. **If the pose drifts, the anchor config is unusable as
   written** — raise the boundary until the shade is masked again and re-measure item 4's
   range with the tighter polygon, or abandon the relaxed ROI and anchor at short range in
   the drive config. Record which you chose in `docs/site/README.md`.
10. **The 60 mm lens offset.** `tf2_echo base_footprint zed_front_left_camera_optical_frame`
    from item 2 also settles the sign: confirm the left optical centre is at
    +0.060 m in base_footprint `y` (left of the mounting screw) and not −0.060, and correct
    `CAMERA_OPTICAL_IN_BASE_FOOTPRINT` if the URDF disagrees. A sign error here is a 0.12 m
    lateral bias on every landmark, in the same direction every time, which a rigid fit
    absorbs into the translation and never reports as residual.

---

## 9. Competition-day procedure

Written for the operator. This is what goes in `docs/site/README.md`.

**The evening before / warm-up day**
1. Take the judges' site map. Type the landmark coordinates into a copy of
   `docs/site/landmarks.example.json` — **pole axis positions**, in site metres, ids as the
   strings printed on the markers. Save it somewhere you will find it.
2. Put the confirmed dictionary name and marker edge length in the `marker` block.
3. Load the file in the ground station's **Site** drawer once and confirm every landmark
   appears with the coordinates you typed. A typo found here is free.

**On the start point, before anything moves**
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

**Judging the fit**
11. Read the RMS. Under ~0.10 m is good; over ~0.5 m the card warns and you should look at
    the worst landmark it names.
12. If one landmark is clearly wrong, untick it and solve again. If the scale warning
    appears, you have probably matched two measurements to the wrong ids — check them.
13. **With only two landmarks, the RMS proves almost nothing.** Get a third if you can, even
    by tape measure.

**Locking and going**
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

## 10. Self-review

*Done after writing; findings applied inline above.*

1. **Does anything under `ground_station/` import `rclpy` or reach into `rover/`?** No.
   `site_frame.py` and `landmark_table.py` are `math`/`json`/`dataclasses` only; `site_card.py`
   is PySide6 + `ground_station.theme` + the two new pure modules; `models.py` and
   `ros_client.py` keep their existing import sets. D1 exists precisely so no `sys.path` shim
   is needed, and Task 3's acceptance criterion greps for it. ✅
2. **Does any rover autonomy code path change?** No. `goal_relay`, `mode_supervisor`,
   `navi_autonomy`, `navi_supervisor`, `navi_teleop` and `navi_shaper` are in the do-not-touch
   list, and the only rover files created are two new nodes and one pure module inside
   `navi_localization`. The transform never crosses to the rover in any form. ✅
3. **Is the no-transform path really a no-op?** Three tests say so from three directions:
   Task 6 test 1 and 4 (the row), Task 9 test 1 and 4 (the wire), Task 11 variant 10 (byte
   equality of the emitted JSON). That triple is deliberate — a silent regression in Go is the
   worst thing this plan could do. ✅
4. **Can anything here move the rover?** Task 8 test 11 enumerates the anchor node's
   publishers and asserts none of the five motion topics is among them. `site_probe` publishes
   one topic. The ground station gains two publishers, both `/site/*`. The sweep is an operator
   action by design (correction 3). ✅
5. **Is every wire field defined and every failure named?** §3.4 and §3.5 give field tables
   with types and null semantics, and every error is a verbatim string that a test asserts on.
   The probe's contract — *one result per request, always* — is an explicit acceptance
   criterion, because a button that sometimes answers is a button an operator stops trusting.
6. **Gap found while reviewing, now fixed.** An earlier draft had stage 3 use
   `cv2.aruco.estimatePoseSingleMarkers` for the face normal. That contradicts D3, adds a
   dependency on marker orientation that is ambiguous in 90° steps by construction, and makes
   the geometry untestable without OpenCV. Replaced by the camera-to-marker ray, whose error
   is bounded at 17 mm for a 30° view — one addition instead of a PnP solve, and D3 stays
   literally true in the code. `marker_edge_m` is consequently carried but unused, which the
   task says out loud so nobody "fixes" it.
7. **Second gap, now fixed.** The draft assumed a full-resolution image topic existed. Reading
   `zed_front.yaml` showed `pub_downscale_factor: 2.0` is global, and the pixel arithmetic
   showed detection at 640×360 dies around 1.5 m — the feature would have been built and then
   found not to work in the field. This produced correction 1, Task 10, and checklist item 4.
8. **Third gap, now fixed.** The same file's `region_of_interest` masks the top half of the
   frame *including depth*, and a landmark at 4 m sits 2 % of the frame height inside the
   boundary. Nothing in the brief mentioned it and it would have shown up as "the markers
   detect but no sighting ever appears". It is now R2, part of Task 10's config, an explicit
   distinguishable failure in Task 8, and checklist item 5.
9. **Fourth gap, now fixed.** The draft had `_on_go_requested` convert but nothing stopped the
   operator from unlocking mid-run. Task 9 step 5 refuses it and test 8 pins it.
10. **Is the file-ownership split real?** `main_window.py` and `video_panel.py` → T9 only.
    `nav_row.py` → T6 only. `dashboard_page.py` → T5 only. `models.py`/`ros_client.py` → T4
    only. `pose_composition.py` → T3 only. The single shared
    file across the whole plan is `rover/src/navi_localization/setup.py`, edited by T7 and T8
    with one line each, and the resolution ("keep both") is written into both tasks. ✅
11. **Does every task have a runnable command and an acceptance criterion that is checkable
    without hardware?** Yes — every command is either the GS pytest invocation or the
    `PYTHONPATH`-scoped `navi_localization` one naming a specific laptop-safe test file, and
    every hardware-dependent claim is in §7 instead. The two topic names this plan cannot
    verify offline (`depth_topic`, `image_topic`) are declared parameters, marked ⚠ in their
    tasks, and are checklist item 2.
12. **Task count and shape.** Eleven tasks in four waves, one commit each, contracts fixed in
    §3 so wave 1's four agents and wave 2's four agents never need to talk. Stage 1 is
    complete and useful after T9 alone; stage 2 after T7; stage 3 after T8. Nothing in the
    plan requires the Orin to be reachable.

---

### Review round 2 — an adversarial pass against the tree, 2026-09-01

Every claim in §0 was re-opened against the actual file. The GS suite was re-run: **354
passed in 8.5 s**, as claimed. What follows is what did not survive.

**Applied — critical**

- *A click on the plan canvas would have been converted twice.* `NavMapView.point_clicked`
  emits map-frame world coordinates (`nav_map_view.py:124`) and `NavRow.append_world_point`
  adds them verbatim; under a lock §3.8 would then convert that map number as if it were
  site. → New §3.9 states the invariant ("the list always holds the operator's frame"),
  Task 6 converts back with `map_to_site` in `append_world_point`, and T6 tests 6 and 7 pin
  the round trip in both directions. This is what `map_to_site` was in the contract for; the
  round-1 plan never called it.
- *§3.1 told implementers to copy the wrong parser stance.* It claimed a bad field makes
  `parse_nav_status` return `None` for the whole payload. It does not — it returns `None`
  only for non-JSON or non-object input and otherwise fills each bad field from that field's
  default. Copying that here would put a truncated landmark at the map origin. → §3.1 now
  says the new parsers are deliberately stricter, why, and names the `ok: false` exception.
- *Task 9 needed a file no task owned.* The probe click has to come from
  `ground_station/ui/video_panel.py`, which §1 and §4 both declared untouched — and the
  round-1 escape hatch ("put it in the card's own preview instead") would have left an
  implementer inventing a second video path at the end of wave 3. → `video_panel.py` and
  `tests/test_video_panel.py` are now T9's, listed in §4 and in §1's shared-file sentence.
- *And the click would have been in the wrong pixels.* `_render_frame` scales with
  `Qt.KeepAspectRatio`, so the picture is letterboxed inside `image_label`; raw label
  coordinates are offset by the bars. → Task 9 step 3 now specifies the letterbox-origin
  subtraction, the in-bar rejection, and emission in **source-frame** pixels, with three
  cases in `tests/test_video_panel.py`.
- *`manual_polygon: '[]'` would have poisoned the frame the anchor is measured in.*
  `zed_front.yaml` masks the top half because the sun shade would otherwise be tracked by VIO
  as a stationary object that moves with the camera — and stage 3 measures landmarks in that
  very `map` frame. → §0 correction 2 says so, the anchor config lowers the boundary to
  y = 0.35 instead of removing the mask, Task 10 gains a test that refuses an empty polygon,
  R9 is new, and checklist item 9 gates the config on hardware.

**Applied — important**

- *The 125 mm ray-vs-normal error bound was understated ~4×.* The cost is
  `0.125·2·sin(θ/2)` — 65 mm at 30°, 96 mm at 45° — not `0.125·(1−cos θ)`; round 1 counted
  only the radial component and dropped the larger tangential one. → §0 correction 4 carries
  the right formula, notes that θ ≤ 45° because the detected face is always the most
  perpendicular of four, and re-argues the decision honestly at the true magnitude. The
  decision stands.
- *The pixel arithmetic was internally inconsistent and unverifiable offline.* fx = 518 at
  1280 and fx = 1050 at 2208 describe two different lenses (the ratio must be 1.725), and by
  the plan's own 25 px decode threshold HD720 gives 19 px at 4 m — i.e. ~3.1 m, not the
  "~4 m" quoted in checklist item 4. The ZED 2i ships in a 2.1 mm and a 4 mm variant whose fx
  differ by a factor of two, and this repo holds no calibration. → §0 correction 1 is now the
  formula plus a two-lens bracket, explicitly marked as pending `camera_info`; checklist
  item 4 expects the bracket; R10 forbids these numbers reaching the operator's page as
  measurements.
- *`max_depth_m` defaulted to 20.0 against an SDK clamped at 10.0.* `zed_front.yaml` sets
  `depth.max_depth: 10.0`, so nothing beyond it ever returns. → 10.0 in Tasks 7 and 8, with
  the reason and the coupling written beside the declaration; §0 records the clamp.
- *The optical constant used the camera body centre.* `CAMERA_IN_BASE_FOOTPRINT` is the
  URDF's mounting screw; depth and the rectified left image are expressed at the **left
  lens**, half a 120 mm baseline away — 60 mm, systematic, half the offset this plan bothers
  to correct, and Task 3 test 6 was about to freeze it in as a literal `y = 0`. → the
  constant is defined off `ZED_BASELINE_M`, test 6 derives its expectation from the constant,
  and checklist item 10 settles the sign on hardware. A sign error here is a 0.12 m lateral
  bias a rigid fit absorbs into the translation and never reports as residual.
- *Task 3 edited two files that appeared in no table.* It adds `transform_point` to
  `pose_composition.py` (which has `compose`, `inverse` and a private `_rotate`, but no
  public point transform) and a case to `test/test_pose_composition.py`. → both in §4, and
  §5 now states that neither is touched by another task.
- *T11 variant 10 could not have passed as written.* `nav_request_json` embeds
  `new_run_id(time())`, so two windows never produce byte-identical JSON. → the variant now
  says to pin `ground_station.ui.main_window.time`, and says why excising `run_id` from the
  comparison is the wrong fix.
- *§3.8's snippet used an unbound `t`.* → bound from `self._site_transform`.
- *`ProbeResult` carries no `quality` but `set_measurement` demands one.* T5 and T9 would
  have invented different answers. → §3.6 fixes the mapping on `valid_fraction` at 0.60, and
  says a failed probe produces no measurement.
- *T4 would have missed the `__init__` handles.* Every `_*_topic` in `RosBridgeClient` is
  initialised to `None` in `__init__`; §3.6 now says the new ones go there too.

**Noted, not applied — minor**

- §3.7 gives `state_pill` a four-value vocabulary (NO TABLE / N OF M MEASURED / SOLVED /
  LOCKED), but T5 test 12 and T9 step 3 both put sentences on it (a `detector_ok: false`
  error, "click the landmark in the camera view first"). Either is fine; an implementer
  should pick one place for transient messages and stay there. `detail_label` is the natural
  home and `state_pill` the natural place for the state word.
- `editor_title` in `nav_row.py:154` is a local, not an attribute — T6 has to promote it
  before it can append "(site)". Called out inline in Task 6, a minute's work.
- §3.6's `SightingsReport` drops `frame_id`, `image_size` and `stamp_s`, which §3.5 puts on
  the wire. Deliberate (the GS judges staleness by its own clock and only ever gets `map`),
  but a reader comparing the two sections will notice the asymmetry.
- §0 credited `/localization/pose` to `elevation_mapper.py:60`; it is published by
  `localization_status.py:52`. Corrected in the table.

**Could not be verified offline, and is marked as such wherever it is used**

The Orin is off, so: the real `fx`/`fy`/`cx`/`cy`; whether
`/zed_front/zed_node/depth/camera_info` and
`/zed_front/zed_node/left/image_rect_color` are the live names; the 0.417 m tag-centre height
(it is in Figure 9 of the rules, an image, not text — the 250×250×310 mm box, the 150 mm 5×5
tag, the pole-axis convention, the provisional ids 51–64 and "at least two landmarks visible
from the starting point" all **were** confirmed against the rules text); the sign of the left
lens offset in the URDF; which ArUco dictionary the ERC generator's tags decode under; and
whether restarting the wrapper preserves the `map` frame (checklist item 6, which remains the
single item that can invalidate the whole stage-3 procedure).


### Review round 3 (Fable, 2026-09-01, live rover attached)

1. **Critical — the restart rotated the frame out from under the lock.** The anchor sweep
   turns the rover after the old map frame's birth; the drive-config relaunch bears a new
   frame at the post-sweep pose; the locked transform was wrong by the sweep angle, silently,
   and ops step 15's landmark check could not run at drive-config range. Root fact verified
   offline: `area_memory_db_path: ''` — no frame survives a wrapper restart. Fix: §3.2
   `reexpress_at_lock_pose`, §3.7 `camera_restarted` button/signal, new §3.10, ops steps
   14–15 rewritten, checklist 6 rewritten as the end-to-end verification of the fix.
2. **fx, lens variant and topic names measured live** and written into §0 correction 1;
   checklist items 2 and 4 now verify rather than discover. HD2K is the working assumption
   for the anchor config.
3. Waves and file ownership re-checked against the tree as it stands after the GS redesign
   commits (14e7b94): the six shared GS files named in §1 are the current ones; no collision
   found beyond the documented setup.py line. T1/T5/T9 implementers must be briefed that the
   contracts gained §3.10 and the two API/widget additions above.
