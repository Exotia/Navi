"""Glare detection and detour geometry - pure numpy, no ROS.

The ZED 2i's visual odometry is a camera pipeline: when the rover drives
toward the sun, the sun saturates the frame, auto-exposure crushes
everything else to black chasing a bright average, and tracking is lost.
There is no recovery from inside the camera driver - the fix has to be
behavioural, upstream of it. `saturated_fractions` and `glare_side` turn a
raw frame into "is the sun in view, and which side of the frame is it on",
and `detour_point` / `DetourPlanner` turn that into a temporary waypoint
that steers the camera away from the sun before continuing toward the real
goal - tacking like a sailboat working upwind, one short leg at a time
rather than fighting the glare head-on.
"""

import math

import numpy as np

#: 8-bit channel value at or above which a pixel counts as blown out.
SATURATION_LEVEL = 250

#: A half of the frame is "glaring" when at least this fraction of its
#: pixels are saturated.
GLARE_FRACTION = 0.02

#: One half must have at least this many times the other's saturated
#: fraction before a side is named. Prevents a uniformly bright sky from
#: being read as a direction.
GLARE_MARGIN = 1.5

# Below this separation, rover_xy and goal_xy no longer define a direction
# worth trusting: the rover->goal unit vector's error blows up as the
# distance shrinks toward it, and a perpendicular offset from a direction
# that is mostly float noise would send the rover somewhere arbitrary.
_MIN_LEG_M = 1e-9


def saturated_fractions(image, saturation_level=SATURATION_LEVEL) -> tuple:
    """(left_fraction, right_fraction): the fraction of pixels in each half
    of the frame that are saturated.

    `image` is a numpy array, either 2-D (H x W, greyscale) or 3-D
    (H x W x C, any channel count - a pixel counts as saturated when ALL of
    its colour channels are at or above saturation_level, so a saturated
    pixel is white, not merely a strong red).

    An odd width splits with the centre column excluded from both halves.
    An empty image (either dimension 0) returns (0.0, 0.0) rather than
    dividing by zero.
    """
    image = np.asarray(image)
    if image.ndim == 2:
        saturated = image >= saturation_level
    elif image.ndim == 3:
        saturated = np.all(image >= saturation_level, axis=-1)
    else:
        raise ValueError(f"image must be 2-D or 3-D, got shape {image.shape}")

    height, width = saturated.shape
    if height == 0 or width == 0:
        return (0.0, 0.0)

    # half = width // 2 on both sides, so an odd width leaves the centre
    # column (index `half`) in neither slice - it belongs to no side.
    half = width // 2
    left = saturated[:, :half]
    right = saturated[:, width - half:]
    left_fraction = float(left.mean()) if left.size else 0.0
    right_fraction = float(right.mean()) if right.size else 0.0
    return (left_fraction, right_fraction)


def glare_side(left_fraction, right_fraction,
               fraction=GLARE_FRACTION, margin=GLARE_MARGIN):
    """'left', 'right' or None - which side of the frame the glare is on.

    Returns None when neither half reaches `fraction` (no glare worth
    reacting to), and also when the two halves are within `margin` of each
    other (glare everywhere, or a bright overcast sky: there is no side to
    steer away from, so naming one would be invention).
    """
    if left_fraction < fraction and right_fraction < fraction:
        return None

    lo = min(left_fraction, right_fraction)
    hi = max(left_fraction, right_fraction)
    # lo == 0.0 means one side has no saturation at all, so the margin is
    # infinite and there is nothing to divide by - the glaring side wins
    # outright. Otherwise the margin test is the whole point of this
    # function: a sky that is merely uniformly bright must not be read as a
    # direction.
    if lo > 0.0 and hi < margin * lo:
        return None

    return 'left' if left_fraction > right_fraction else 'right'


def detour_point(rover_xy, goal_xy, side, offset_m=2.0, along_fraction=0.5) -> tuple:
    """A temporary waypoint that keeps the glare out of the camera.

    `rover_xy` and `goal_xy` are (x, y) in metres in the map frame. `side`
    is the value glare_side returned - 'left' or 'right', naming where the
    glare IS. The returned point is offset to the OPPOSITE side, so driving
    to it turns the camera away from the sun.

    The point sits `along_fraction` of the way from the rover to the goal,
    displaced `offset_m` perpendicular to that line. Coordinates are
    right-handed with +x forward and +y left (REP-103), so "left of the
    rover->goal direction" is +90 degrees from it.

    Returns `goal_xy` unchanged when `side` is None, or when the rover is
    already essentially at the goal (the direction is undefined, so there
    is no perpendicular to offset along).
    """
    if side is None:
        return (goal_xy[0], goal_xy[1])

    rx, ry = rover_xy
    gx, gy = goal_xy
    dx = gx - rx
    dy = gy - ry
    distance = math.hypot(dx, dy)
    if distance < _MIN_LEG_M:
        return (goal_xy[0], goal_xy[1])

    ux, uy = dx / distance, dy / distance
    # Rotating the heading (ux, uy) by +90 degrees (counter-clockwise) gives
    # (-uy, ux), and with +y left that rotation is exactly "left of
    # heading" - the convention the docstring promises.
    left_x, left_y = -uy, ux

    # The glare is on `side`; the detour goes the other way.
    sign = 1.0 if side == 'right' else -1.0

    base_x = rx + along_fraction * dx
    base_y = ry + along_fraction * dy
    return (base_x + sign * offset_m * left_x, base_y + sign * offset_m * left_y)


class DetourPlanner:
    """Decides, one leg at a time, whether the next Nav2 goal is the real
    waypoint or a detour around the glare.

    A "leg" is one real waypoint. The planner allows at most `max_detours`
    detours per leg, so a rover that cannot escape the glare eventually
    drives at the goal anyway rather than tacking forever and running the
    mission clock out.
    """

    def __init__(self, offset_m=2.0, along_fraction=0.5, max_detours=4):
        self._offset_m = offset_m
        self._along_fraction = along_fraction
        self._max_detours = max_detours
        self._detours_taken = 0

    def begin_leg(self) -> None:
        """A new real waypoint has been dispatched: reset the counter."""
        self._detours_taken = 0

    @property
    def detours_taken(self) -> int:
        return self._detours_taken

    def next_target(self, rover_xy, goal_xy, side) -> tuple:
        """Returns ((x, y), is_detour).

        When `side` names a glare side and this leg has detours left, the
        point is a detour (is_detour True) and the counter increments.
        Otherwise it is `goal_xy` itself with is_detour False.

        `rover_xy` may be None (no pose seen yet): then there is no
        geometry to work from, so the real goal is returned.
        """
        if rover_xy is None or side is None or self._detours_taken >= self._max_detours:
            return ((goal_xy[0], goal_xy[1]), False)

        point = detour_point(rover_xy, goal_xy, side, self._offset_m, self._along_fraction)
        if point == (goal_xy[0], goal_xy[1]):
            # detour_point declined - the rover is already essentially at the
            # goal, so there is no direction to offset from. Calling that a
            # detour would spend one of the leg's allowance on a goal that is
            # the real one anyway, and would send it to Nav2 under the detour
            # callbacks, so the run would learn nothing from its arrival.
            return (point, False)
        self._detours_taken += 1
        return (point, True)
