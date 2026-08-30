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
    assert mapping['resolution'] == 0.02
    assert mapping['max_mapping_range'] == 6.0
    assert mapping['fused_pointcloud_freq'] == 1.0


def test_the_mapping_resolution_is_at_least_as_fine_as_the_grid():
    # A grid finer than the cloud is a comb of empty cells. A cloud finer
    # than the grid is fine: the grid takes the 20th percentile of several
    # points per cell, and the 5 cm obstacle voxels draw solid.
    from navi_localization.elevation_grid import RESOLUTION

    assert _parameters()['mapping']['resolution'] <= RESOLUTION


def test_depth_confidence_filters_noise_but_keeps_textureless_surfaces():
    depth = _parameters()['depth']
    assert depth['depth_confidence'] == 50
    assert depth['depth_texture_conf'] == 100


def test_the_floor_is_not_the_map_origin_and_video_stays_640x360():
    parameters = _parameters()
    # floor_alignment picked floors 0.5 m apart between starts on the bench.
    assert parameters['pos_tracking']['floor_alignment'] is False
    # HD1080 + NEURAL pinned the Orin GPU at 99 % at 25 W (2026-08-30);
    # HD720 + 2 cm mapping bursts to 99 % but averages ~60 % and holds
    # pose at 15 Hz.
    assert parameters['general']['grab_resolution'] == 'HD720'
    assert parameters['general']['pub_downscale_factor'] == 2.0     # 1280/2 = 640


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
