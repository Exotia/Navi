"""Which models go into the world, and which do not.

In semi-autonomous mode the ground the operator sees has to be the ground
the rover has seen, so the organisers' static scan is left out of the world
and terrain_writer spawns the rover's own map instead. The ground plane
stays in both modes as a horizon, but in semi mode it is lowered to
SEMI_GROUND_Z: the map frame's origin is the ZED's start, so the real
ground is ~0.55 m below z = 0 and a plane there would hide the map.

Kept out of the launch file so it can be tested without a Gazebo, a mesh or
a ROS graph - the launch file's job is to call it.
"""

SITE_SCAN_MARKER = '<!-- SITE_SCAN_MODEL -->'
GROUND_POSE_MARKER = '<!-- GROUND_POSE -->'
MESH_PLACEHOLDER = 'MAP_MESH_PATH'

# Where the grey plane sits in semi mode. The rover's map frame has its
# origin where the ZED started, not on the floor: on flat ground the
# localised base_footprint - and every terrain tile, drawn at true map
# heights - is about 0.55 m *below* z = 0. A plane at 0 hides all of it.
# The rover has gravity off and is placed by set_entity_state, so it needs
# nothing to stand on; the plane only remains as a horizon, far enough
# down that no mapped slope reaches it.
SEMI_GROUND_Z = -5.0


def site_scan_required(mode: str) -> bool:
    """True when this mode displays the organisers' scan, so needs its mesh."""
    return mode != 'semi'


def compose_world(world_text: str, scan_text: str, mode: str,
                  mesh_path: str = '') -> str:
    """The world SDF for `mode`, with the scan model spliced in or left out."""
    if SITE_SCAN_MARKER not in world_text:
        raise RuntimeError(
            f"site.world no longer contains {SITE_SCAN_MARKER}. That marker "
            "is where the terrain model goes; without it the simulation "
            "would come up with no ground but the plane and no error.")
    if GROUND_POSE_MARKER not in world_text:
        raise RuntimeError(
            f"site.world no longer contains {GROUND_POSE_MARKER} in its ground "
            "model; semi mode needs it to lower the plane under the map.")
    if not site_scan_required(mode):
        return (world_text
                .replace(SITE_SCAN_MARKER, '')
                .replace(GROUND_POSE_MARKER, f'<pose>0 0 {SEMI_GROUND_Z} 0 0 0</pose>'))
    return (world_text
            .replace(GROUND_POSE_MARKER, '')
            .replace(SITE_SCAN_MARKER, scan_text.replace(MESH_PLACEHOLDER, mesh_path)))
