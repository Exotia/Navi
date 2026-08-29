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
