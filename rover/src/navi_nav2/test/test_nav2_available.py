"""Every Nav2 package this bringup names must actually be installed.

A missing plugin package does not fail at launch - it fails inside
pluginlib during the configure transition, tens of seconds later, with a
class-loader message that reads like a typo in the parameter file. This
test is that failure, moved to the front and given a name.

Verified 2026-08-31: laptop and Orin both carry navigation2 1.1.20, the
same 31 ros-humble-nav2* packages, arm64 and amd64 respectively.
"""

import shutil
import subprocess

import pytest

REQUIRED_PACKAGES = [
    'nav2_lifecycle_manager',
    'nav2_planner',
    'nav2_controller',
    'nav2_behaviors',
    'nav2_bt_navigator',
    'nav2_velocity_smoother',
    'nav2_collision_monitor',
    'nav2_costmap_2d',
    'nav2_theta_star_planner',
    'nav2_smac_planner',
    'nav2_regulated_pure_pursuit_controller',
    'nav2_rotation_shim_controller',
    'nav2_msgs',
    'nav2_common',
]


@pytest.mark.parametrize('package', REQUIRED_PACKAGES)
def test_the_package_is_installed(package):
    assert shutil.which('ros2') is not None, "source /opt/ros/humble/setup.bash first"
    found = subprocess.run(['ros2', 'pkg', 'prefix', package],
                           capture_output=True, text=True)
    assert found.returncode == 0, (
        f"{package} is missing. On a machine with internet: "
        f"sudo apt install ros-humble-{package.replace('_', '-')}. "
        f"On the Orin, carry the debs over - see the runbook in "
        f"rover/src/navi_nav2/launch/nav2_bringup.launch.py.")


def test_the_planner_and_controller_plugins_load_from_pluginlib():
    """The four plugin classes named in params/nav2_rover.yaml, spelled the
    way pluginlib spells them - checked against the installed plugin XMLs,
    not against memory."""
    import os
    wanted = {
        'nav2_theta_star_planner': 'nav2_theta_star_planner/ThetaStarPlanner',
        'nav2_smac_planner': 'nav2_smac_planner/SmacPlanner2D',
        'nav2_rotation_shim_controller':
            'nav2_rotation_shim_controller::RotationShimController',
        'nav2_regulated_pure_pursuit_controller':
            'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
    }
    for package, class_name in wanted.items():
        share = subprocess.run(['ros2', 'pkg', 'prefix', '--share', package],
                               capture_output=True, text=True, check=True).stdout.strip()
        blob = ''
        for entry in os.listdir(share):
            if entry.endswith('.xml') and entry != 'package.xml':
                with open(os.path.join(share, entry)) as handle:
                    blob += handle.read()
        assert class_name in blob, f"{class_name} not declared by {package}"
