"""The image-transport override list exists twice - as parameters in
config/zed_front.yaml, which is what actually reaches the node, and as
IMAGE_TOPICS in launch/localization.launch.py, which is what documents and
generates them. The wrapper's included launch accepts no extra node
parameters, so the two copies cannot be collapsed into one; this test keeps
them from drifting apart.

Drift is silent and expensive: a topic missing from the YAML gets its default
transports back, including ffmpeg, whose libx264 setup is what the start-up
budget was spent on.
"""

import importlib.util
import pathlib

import yaml

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parent.parent
LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'localization.launch.py'
CONFIG_FILE = PACKAGE_ROOT / 'config' / 'zed_front.yaml'


def _load_launch_module():
    spec = importlib.util.spec_from_file_location('localization_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _yaml_plugin_parameters():
    parameters = yaml.safe_load(CONFIG_FILE.read_text())['/**']['ros__parameters']
    return {name: value for name, value in parameters.items()
            if name.endswith('.enable_pub_plugins')}


def test_yaml_matches_the_launch_files_list():
    assert _yaml_plugin_parameters() == _load_launch_module().raw_only_plugin_parameters()


def test_every_topic_is_pinned_to_raw_only():
    for name, value in _yaml_plugin_parameters().items():
        assert value == ['image_transport/raw'], name


def test_the_override_file_the_launch_file_points_at_is_the_one_tested():
    source = LAUNCH_FILE.read_text()
    assert f"'config', '{CONFIG_FILE.name}'" in source


def _parameters():
    return yaml.safe_load(CONFIG_FILE.read_text())['/**']['ros__parameters']


def test_spatial_mapping_is_on_with_the_numbers_the_map_was_designed_for():
    mapping = _parameters()['mapping']

    assert mapping['mapping_enabled'] is True
    assert mapping['resolution'] == 0.05
    assert mapping['max_mapping_range'] == 8.0
    assert mapping['fused_pointcloud_freq'] == 1.0


def test_the_mapping_resolution_matches_the_grid_the_mapper_bins_into():
    # A grid finer than the cloud is a comb of empty cells; a grid coarser
    # than the cloud throws measurements away. They have to be one number.
    from navi_localization.elevation_grid import RESOLUTION

    assert _parameters()['mapping']['resolution'] == RESOLUTION


def test_mapping_never_reaches_further_than_the_depth_that_is_published():
    parameters = _parameters()

    assert parameters['mapping']['max_mapping_range'] <= parameters['depth']['max_depth']


def test_depth_is_neural_at_fifteen_frames_the_calibration_picked():
    # 2026-08-29 calibration on the Orin at 25 W: NEURAL at 30 fps dropped
    # to 25/23 Hz (model latency), NEURAL_PLUS to 6.7 Hz; NEURAL at 15 fps
    # holds 15/15 Hz at 29 % GPU with 5-10x less depth noise than
    # PERFORMANCE. See docs/superpowers/specs/2026-08-29-tiled-map-design.md.
    parameters = _parameters()
    assert parameters['depth']['depth_mode'] == 'NEURAL'
    assert parameters['general']['grab_frame_rate'] == 15
    assert parameters['general']['pub_frame_rate'] == 15.0


def test_the_launch_file_starts_the_elevation_mapper():
    source = LAUNCH_FILE.read_text()

    assert "executable='elevation_mapper'" in source
