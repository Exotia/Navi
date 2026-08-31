"""A regression test for the fixed set of topics sim_bridge carries.

Its own file, not test_sim_bridge.py, which tests the bridge script and
never touches the launch file at all. Loaded by path with the same
importlib.util.spec_from_file_location idiom test_sim_bridge.py already
uses for scripts/sim_bridge.py - launch/sim.launch.py is not an installed
module either.

A topic that silently stops being carried is exactly the failure that
leaves the Gazebo mirror blank with nothing to look at, so this only
checks the module-level constant - it does not call _actions()/
generate_launch_description(), which raise/require Gazebo and the
gitignored site mesh.
"""

import importlib.util
import pathlib

_PATH = pathlib.Path(__file__).resolve().parent.parent / "launch" / "sim.launch.py"
_spec = importlib.util.spec_from_file_location("sim_launch", _PATH)
sim_launch = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sim_launch)


def test_the_nav_path_summary_is_among_the_bridged_topics():
    assert "/nav_path_summary:std_msgs/msg/String" in sim_launch.BRIDGED_TOPICS
