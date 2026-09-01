"""The rigid 2D site->map fit — the arithmetic behind the SITE anchor.

**What "site" and "map" are.** The rover's own world is the `map` frame: it
is born at the rover's boot pose, and `/localization/pose` reports every
later position in it. The judges hand out a site map with landmark
coordinates in their own grid — that is the `site` frame. Nothing about the
two frames is related until an operator measures where two or more of the
judges' landmarks actually sit in the rover's `map` frame; this module turns
those measured correspondences into a single rigid transform, `site -> map`.

**Which direction it goes.** `solve_site_to_map` takes pairs of
(site coordinate, measured map coordinate) and returns a `SiteTransform`
such that `p_map = R(yaw) @ p_site + (x, y)`. `site_to_map` applies it
forward — the direction a waypoint typed in the judges' grid needs to travel
to become a `/nav_request` the rover understands. `map_to_site` applies the
inverse — used only where a coordinate already in `map` (e.g. a click on the
plan canvas) has to be re-expressed as if the operator had typed it in the
judges' grid.

**Why there is no scale.** The judges' metre and the rover's metre are the
same metre — nobody is handing out a map at a different scale, only at a
different origin and heading. The fit is therefore a pure rotation and
translation (3 degrees of freedom: x, y, yaw), solved once in closed form
from the correspondences and never adjusted afterwards. `scale_hint` is
reported alongside the fit purely as a diagnostic: it says what scale factor
*would* have best explained the data, so an operator can spot a swapped pair
of landmark ids (its baseline would look mysteriously stretched or
compressed) even though the solver itself never applies one.

**Collinear landmarks are fine.** A fit that also solved for scale, or for a
general affine map, would need three non-collinear points to be well posed.
This fit does not: with no scale to disentangle, two points already fully
determine a rigid transform, and any number of additional points on the same
line only adds more evidence for the same rotation. The only real
degeneracy is when the landmarks carry no directional information at all —
every site point (or every map point) sitting at the same spot, so there is
no baseline to align — and that is reported as an error, not silently
absorbed.

**What the residual means physically.** For a landmark id, feed its
published site coordinate through the fitted transform and compare the
result to the map coordinate the rover actually measured for it: the
residual is the distance, in metres, between "where the fit says that
landmark should be" and "where the rover saw it". `rms_m` is the root-mean-
square of those distances over every landmark in the fit; `max_residual_m`
and `worst_id` name the single worst offender, which is usually a
mis-identified marker rather than measurement noise.

With only two landmarks the residual degenerates to a single number split
evenly between the two points (see `solve_site_to_map`'s docstring) and
cannot, by itself, catch an id swap that happens to preserve the baseline
length — a third landmark is what makes the residual mean something.
"""

import math
from dataclasses import dataclass


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
    yaw: float
    rms_m: float
    max_residual_m: float
    worst_id: str | None
    n_points: int
    scale_hint: float
    ids: tuple


class SiteFrameError(ValueError):
    """Raised when a fit cannot be attempted. The message is shown to the
    operator verbatim, so write it for a human under time pressure."""


def _wrap(angle: float) -> float:
    """Wrap an angle in radians to (-pi, pi]."""
    wrapped = (angle + math.pi) % (2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def _rotate(x: float, y: float, yaw: float):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y, s * x + c * y


def solve_site_to_map(pairs) -> SiteTransform:
    """Solve the rigid 2D site->map transform over ``pairs``.

    Closed form (2D Kabsch/Umeyama, no scale):

        c_s = centroid of the site points          c_m = centroid of the map points
        a_i = site_i - c_s                          b_i = map_i - c_m
        num = sum(a_x*b_y - a_y*b_x)                 den = sum(a_x*b_x + a_y*b_y)
        yaw = atan2(num, den)
        (x, y) = c_m - R(yaw) . c_s

    With exactly two landmarks the fit is exact in direction (num/den is the
    angle between the two centred vectors regardless of their magnitude), so
    any length mismatch between the published and measured baselines is
    absorbed entirely by translation and split evenly between the two
    points from their shared centroid: ``rms_m`` ends up half the baseline
    length error and nothing else. A mis-identification that happens to
    preserve the baseline length is invisible to it — that is what a third
    landmark is for.
    """
    pairs = list(pairs)
    n = len(pairs)
    if n < 2:
        raise SiteFrameError(f"need at least 2 landmarks, got {n}")

    seen = set()
    for p in pairs:
        if p.id in seen:
            raise SiteFrameError(f"landmark '{p.id}' appears twice")
        seen.add(p.id)

    for p in pairs:
        coords = (p.site_x, p.site_y, p.map_x, p.map_y)
        if not all(math.isfinite(c) for c in coords):
            raise SiteFrameError(f"landmark '{p.id}' has a non-finite coordinate")

    c_sx = sum(p.site_x for p in pairs) / n
    c_sy = sum(p.site_y for p in pairs) / n
    c_mx = sum(p.map_x for p in pairs) / n
    c_my = sum(p.map_y for p in pairs) / n

    a = [(p.site_x - c_sx, p.site_y - c_sy) for p in pairs]
    b = [(p.map_x - c_mx, p.map_y - c_my) for p in pairs]

    site_spread = max(math.hypot(ax, ay) for ax, ay in a)
    map_spread = max(math.hypot(bx, by) for bx, by in b)

    site_coincident_msg = "the landmarks are at the same site position — check the table"
    map_coincident_msg = (
        "the measurements are at the same map position — "
        "the rover measured one landmark twice"
    )

    if site_spread < 0.05:
        raise SiteFrameError(site_coincident_msg)
    if map_spread < 0.05:
        raise SiteFrameError(map_coincident_msg)

    num = sum(ax * by - ay * bx for (ax, ay), (bx, by) in zip(a, b))
    den = sum(ax * bx + ay * by for (ax, ay), (bx, by) in zip(a, b))

    if math.hypot(num, den) < 1e-9:
        # The only true degeneracy beyond simple coincidence: the
        # correspondences carry no consistent rotation at all (e.g. a
        # reflection rather than any single rotation), so num and den both
        # vanish even though neither point cloud is actually coincident.
        if site_spread <= map_spread:
            raise SiteFrameError(site_coincident_msg)
        raise SiteFrameError(map_coincident_msg)

    yaw = _wrap(math.atan2(num, den))
    cy, sy = math.cos(yaw), math.sin(yaw)
    x = c_mx - (cy * c_sx - sy * c_sy)
    y = c_my - (sy * c_sx + cy * c_sy)

    partial = SiteTransform(
        x=x, y=y, yaw=yaw, rms_m=0.0, max_residual_m=0.0, worst_id=None,
        n_points=n, scale_hint=1.0, ids=tuple(p.id for p in pairs),
    )

    res = []
    for p in pairs:
        mx, my = site_to_map(partial, p.site_x, p.site_y)
        res.append((p.id, math.hypot(p.map_x - mx, p.map_y - my)))

    rms = math.sqrt(sum(d * d for _, d in res) / n)
    worst_id, max_residual = max(res, key=lambda r: r[1])

    sum_a = sum(math.hypot(ax, ay) for ax, ay in a)
    sum_b = sum(math.hypot(bx, by) for bx, by in b)
    scale_hint = (sum_b / sum_a) if sum_a > 0.0 else 1.0

    return SiteTransform(
        x=x, y=y, yaw=yaw, rms_m=rms, max_residual_m=max_residual,
        worst_id=worst_id, n_points=n, scale_hint=scale_hint,
        ids=tuple(p.id for p in pairs),
    )


def site_to_map(t: SiteTransform, x: float, y: float):
    rx, ry = _rotate(x, y, t.yaw)
    return rx + t.x, ry + t.y


def map_to_site(t: SiteTransform, x: float, y: float):
    dx, dy = x - t.x, y - t.y
    c, s = math.cos(t.yaw), math.sin(t.yaw)
    return c * dx + s * dy, -s * dx + c * dy


def site_yaw_to_map_yaw(t: SiteTransform, yaw: float) -> float:
    return _wrap(yaw + t.yaw)


def residuals(t: SiteTransform, pairs):
    """Per-landmark residual, in metres, in the order given."""
    out = []
    for p in pairs:
        mx, my = site_to_map(t, p.site_x, p.site_y)
        out.append((p.id, math.hypot(p.map_x - mx, p.map_y - my)))
    return out


def reexpress_at_lock_pose(
    t: SiteTransform, pose_x: float, pose_y: float, pose_yaw: float
) -> SiteTransform:
    """Re-express ``t`` (site -> old_map) as site -> new_map, where new_map
    is the frame the ZED wrapper bears fresh at ``(pose_x, pose_y, pose_yaw)``
    — the rover's own ``/localization/pose`` in old_map at the moment of
    Lock.

    The ZED persists no area memory (``area_memory_db_path: ''``), so
    restarting the wrapper bears a brand new ``map`` frame at the rover's
    pose at relaunch. With the rover unmoved from Lock through the restart,
    that new frame *is* the rover-at-lock frame, so the locked transform
    survives the restart as::

        site -> new_map = inverse(T_oldmap_base_at_lock) . t

    Closed form: ``yaw' = wrap(t.yaw - pose_yaw)``; the translation is the
    old-frame lock pose subtracted and counter-rotated:
    ``(x', y') = R(-pose_yaw) . ((t.x, t.y) - (pose_x, pose_y))``.

    The fit-quality fields (``rms_m``, ``max_residual_m``, ``worst_id``,
    ``n_points``, ``scale_hint``, ``ids``) carry over unchanged —
    re-expression moves the frame, not the fit.
    """
    dx, dy = t.x - pose_x, t.y - pose_y
    c, s = math.cos(pose_yaw), math.sin(pose_yaw)
    x = c * dx + s * dy
    y = -s * dx + c * dy
    yaw = _wrap(t.yaw - pose_yaw)

    return SiteTransform(
        x=x, y=y, yaw=yaw, rms_m=t.rms_m, max_residual_m=t.max_residual_m,
        worst_id=t.worst_id, n_points=t.n_points, scale_hint=t.scale_hint,
        ids=t.ids,
    )
