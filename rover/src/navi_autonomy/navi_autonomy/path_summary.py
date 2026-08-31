"""The plan, small enough for the field link.

Spec section 7: "the plan is drawn in the Gazebo mirror via
/nav_path_summary (decimated; the raw /plan is too heavy for the field
link)". A Nav2 plan over a 0.05 m costmap is one pose per cell - ~940 of
them over 47 m, about 60 KB as a nav_msgs/Path. This turns that into at
most MAX_POINTS vertices, roughly 1 KB of JSON.

Ramer-Douglas-Peucker rather than every-Nth-point: an any-angle Theta*
path is long straight runs joined by a few corners, and the corners are
the entire content. Dropping every second point keeps the straights and
rounds the corners - exactly backwards.

Pure Python, no ROS and no numpy: it runs on the laptop under plain
pytest (spec section 9 rung 1).
"""

import math

# Two costmap cells. Below this the deviation is smaller than the map the
# plan was computed on can represent.
TOLERANCE_M = 0.10

# Hard cap on the vertices that cross the link. Reached by doubling the
# tolerance until the polyline fits - a very wiggly plan loses detail
# rather than the link losing its budget.
MAX_POINTS = 60

FRAME_ID = "map"


def _perpendicular_distance(point, start, end) -> float:
    (px, py), (ax, ay), (bx, by) = point, start, end
    dx, dy = bx - ax, by - ay
    if dx == 0.0 and dy == 0.0:
        return math.hypot(px - ax, py - ay)
    # Twice the triangle's area over the base length.
    return abs(dy * (px - ax) - dx * (py - ay)) / math.hypot(dx, dy)


def _rdp(points, tolerance: float) -> list:
    if len(points) < 3:
        return list(points)
    worst_index, worst = 0, 0.0
    for i in range(1, len(points) - 1):
        d = _perpendicular_distance(points[i], points[0], points[-1])
        if d > worst:
            worst_index, worst = i, d
    if worst <= tolerance:
        return [points[0], points[-1]]
    left = _rdp(points[:worst_index + 1], tolerance)
    right = _rdp(points[worst_index:], tolerance)
    return left[:-1] + right


def decimate(points, tolerance: float = TOLERANCE_M,
             max_points: int = MAX_POINTS) -> list:
    """At most `max_points` vertices, ends always kept.

    Recursion depth: _rdp splits at the worst point, so the depth is
    bounded by the number of retained vertices, not by len(points) - a
    941-point plan that decimates to 6 recurses 6 deep, not 941.
    """
    points = [(float(x), float(y)) for x, y in points]
    if len(points) < 3:
        return points
    result = _rdp(points, tolerance)
    # Doubling rather than a binary search: at most ~10 rounds to go from
    # 0.10 m to the length of any plan the rover can hold, and each round
    # is linear in the points that survived the previous one.
    while len(result) > max_points:
        tolerance *= 2.0
        result = _rdp(points, tolerance)
    return result


def polyline_length(points) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(points, points[1:]))


def summary_payload(run_id, points, waypoints, now_s: float) -> dict:
    """The /nav_path_summary JSON body. `points` is the RAW plan; the
    decimation happens here so no caller can forget it."""
    decimated = decimate(points)
    return {
        "run_id": run_id,
        "frame_id": FRAME_ID,
        "stamp_s": float(now_s),
        "points": [[x, y] for x, y in decimated],
        "waypoints": [[float(x), float(y)] for x, y in waypoints],
        "length_m": polyline_length(decimated),
        "source_points": len(points),
    }
