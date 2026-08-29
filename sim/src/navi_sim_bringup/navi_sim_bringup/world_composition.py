"""Which models go into the world, and which do not.

In semi-autonomous mode the ground the operator sees has to be the ground
the rover has seen, so the organisers' static scan is left out of the world
and terrain_writer spawns the rover's own map instead. The ground plane at
z = 0 stays in both modes: it is what the rover stands on, and without it a
rover whose map has not arrived yet is in the void.

Kept out of the launch file so it can be tested without a Gazebo, a mesh or
a ROS graph - the launch file's job is to call it.
"""

SITE_SCAN_MARKER = '<!-- SITE_SCAN_MODEL -->'
MESH_PLACEHOLDER = 'MAP_MESH_PATH'


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
    if not site_scan_required(mode):
        return world_text.replace(SITE_SCAN_MARKER, '')
    return world_text.replace(SITE_SCAN_MARKER,
                              scan_text.replace(MESH_PLACEHOLDER, mesh_path))
