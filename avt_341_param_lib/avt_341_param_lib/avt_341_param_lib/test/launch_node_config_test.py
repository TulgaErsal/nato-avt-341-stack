import os
from pathlib import Path

import pytest
from launch.substitutions import SubstitutionFailure
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

from avt_341_param_lib.launch_node_config import NodeConfigCollection


def load(tmp_path: Path, text: str) -> NodeConfigCollection:
    path = tmp_path / 'metadata.yaml'
    path.write_text(text, encoding='utf-8')
    return NodeConfigCollection(path)


class SubstitutionContext:

    @property
    def environment(self):
        return os.environ

    def perform_substitution(self, substitution):
        return substitution.perform(self)


def resolve_env(metadata: NodeConfigCollection, node_fqn: str):
    context = SubstitutionContext()
    return {
        name: perform_substitutions(
            context, normalize_to_list_of_substitutions(value))
        for name, value in metadata.get_additional_env(node_fqn).items()
    }


def test_exact_selector_is_root_scoped(tmp_path):
    metadata = load(
        tmp_path,
        """
object_tracking_node:
  remappings:
    image: /camera/image
""")

    assert metadata.get_remappings('/object_tracking_node') == [
        ('image', '/camera/image')]
    assert metadata.get_remappings('/veh1/object_tracking_node') == []


def test_wildcards_match_expected_namespace_depths(tmp_path):
    metadata = load(
        tmp_path,
        """
/**/object_tracking_node:
  remappings:
    image: /camera/image
/*/planner:
  remappings:
    goal: selected_goal
""")

    tracker_rule = [('image', '/camera/image')]
    assert metadata.get_remappings('/object_tracking_node') == tracker_rule
    assert metadata.get_remappings('/veh1/object_tracking_node') == tracker_rule
    assert metadata.get_remappings('/fleet/veh1/object_tracking_node') == tracker_rule

    assert metadata.get_remappings('/veh1/planner') == [
        ('goal', 'selected_goal')]
    assert metadata.get_remappings('/fleet/veh1/planner') == []


def test_one_wildcard_section_applies_to_multiple_vehicle_nodes(tmp_path):
    metadata = load(
        tmp_path,
        """
/**/object_tracking_node:
  remappings:
    camera_info: /flir_camera/camera_info
    points/input: /ouster/points
""")

    expected = [
        ('camera_info', '/flir_camera/camera_info'),
        ('points/input', '/ouster/points'),
    ]
    assert metadata.get_remappings('/veh1/object_tracking_node') == expected
    assert metadata.get_remappings('/veh2/object_tracking_node') == expected


def test_nested_selector_sections(tmp_path):
    metadata = load(
        tmp_path,
        """
/**:
  navigation:
    planner:
      remappings:
        input: shared_input
""")

    assert metadata.get_remappings('/veh1/navigation/planner') == [
        ('input', 'shared_input')]
    assert metadata.get_remappings('/navigation/planner') == [
        ('input', 'shared_input')]
    assert metadata.get_remappings('/veh1/planner') == []


def test_matching_sections_merge_each_metadata_field_in_document_order(
    tmp_path, monkeypatch
):
    monkeypatch.setenv('BASE_PATH', '/base')
    metadata = load(
        tmp_path,
        """
/**:
  remappings:
    common: broad
    broad_only: broad_value
  additional_env:
    COMMON: broad
    BROAD_ONLY: "$env_var{BASE_PATH}/broad"
/veh1/**:
  remappings:
    common: vehicle_specific
    vehicle_only: vehicle_value
  additional_env:
    COMMON: vehicle_specific
    VEHICLE_ONLY: vehicle_value
""")

    assert metadata.get_remappings('/veh1/planner') == [
        ('broad_only', 'broad_value'),
        ('common', 'vehicle_specific'),
        ('vehicle_only', 'vehicle_value'),
    ]
    assert resolve_env(metadata, '/veh1/planner') == {
        'BROAD_ONLY': '/base/broad',
        'COMMON': 'vehicle_specific',
        'VEHICLE_ONLY': 'vehicle_value',
    }
    assert metadata.get_remappings('/veh2/planner') == [
        ('common', 'broad'),
        ('broad_only', 'broad_value'),
    ]
    assert resolve_env(metadata, '/veh2/planner') == {
        'COMMON': 'broad',
        'BROAD_ONLY': '/base/broad',
    }


def test_additional_env_supports_embedded_values_defaults_and_joining(
    tmp_path, monkeypatch
):
    monkeypatch.setenv('MCR_ROOT', '/opt/matlab')
    monkeypatch.setenv('LD_LIBRARY_PATH', '/existing')
    metadata = load(
        tmp_path,
        """
/**/uab_perception_node:
  additional_env:
    LOG_LEVEL: debug
    LD_LIBRARY_PATH:
      separator: ":"
      values:
        - "$env_var{MCR_ROOT:-/default/matlab}/runtime/glnxa64"
        - "$env_var{MCR_ROOT:-/default/matlab}/bin/glnxa64"
        - "$env_var{LD_LIBRARY_PATH:-}"
""")

    assert resolve_env(metadata, '/veh1/uab_perception_node') == {
        'LOG_LEVEL': 'debug',
        'LD_LIBRARY_PATH': (
            '/opt/matlab/runtime/glnxa64:'
            '/opt/matlab/bin/glnxa64:'
            '/existing'
        ),
    }


def test_additional_env_uses_declared_defaults(tmp_path, monkeypatch):
    monkeypatch.delenv('MCR_ROOT', raising=False)
    monkeypatch.delenv('LD_LIBRARY_PATH', raising=False)
    metadata = load(
        tmp_path,
        """
/**/uab_perception_node:
  additional_env:
    LD_LIBRARY_PATH:
      separator: ":"
      values:
        - "$env_var{MCR_ROOT:-/default/matlab}/runtime/glnxa64"
        - "$env_var{LD_LIBRARY_PATH:-}"
""")

    assert resolve_env(metadata, '/veh1/uab_perception_node') == {
        'LD_LIBRARY_PATH': '/default/matlab/runtime/glnxa64:',
    }


def test_missing_required_environment_variable_fails_when_resolved(
    tmp_path, monkeypatch
):
    monkeypatch.delenv('REQUIRED_ROOT', raising=False)
    metadata = load(
        tmp_path,
        """
/**/node:
  additional_env:
    PATH_WITH_REQUIRED_VALUE: "$env_var{REQUIRED_ROOT}/bin"
""")

    with pytest.raises(SubstitutionFailure, match='REQUIRED_ROOT'):
        resolve_env(metadata, '/veh1/node')


def test_results_are_defensive_collections(tmp_path):
    metadata = load(
        tmp_path,
        """
/**:
  remappings:
    input: output
  additional_env:
    MODE: value
""")
    first_remappings = metadata.get_remappings('/veh1/node')
    first_remappings.append(('extra', 'value'))
    assert metadata.get_remappings('/veh1/node') == [('input', 'output')]

    first_env = metadata.get_additional_env('/veh1/node')
    first_env['MODE'].append('changed')
    first_env['EXTRA'] = ['value']
    second_env = metadata.get_additional_env('/veh1/node')
    assert list(second_env) == ['MODE']
    assert len(second_env['MODE']) == 1


@pytest.mark.parametrize('path_value', [None, ''])
def test_none_and_empty_paths_create_empty_collection(path_value):
    metadata = NodeConfigCollection(path_value)
    assert metadata.get_remappings('/veh1/node') == []
    assert metadata.get_additional_env('/veh1/node') == {}


@pytest.mark.parametrize('contents', ['', '{}\n'])
def test_empty_files_and_mappings_are_valid(tmp_path, contents):
    metadata = load(tmp_path, contents)
    assert metadata.get_remappings('/veh1/node') == []
    assert metadata.get_additional_env('/veh1/node') == {}


def test_missing_file_raises_file_not_found(tmp_path):
    with pytest.raises(FileNotFoundError):
        NodeConfigCollection(tmp_path / 'missing.yaml')


@pytest.mark.parametrize(
    'contents',
    [
        '- not\n- a\n- mapping\n',
        '/node: scalar\n',
        '/node: {}\n',
        '/node:\n  remappings: []\n',
        '/node:\n  remappings: {}\n  unexpected: {}\n',
        '/node:\n  additional_env: []\n',
        '/node:\n  additional_env:\n    1: value\n',
        '/node:\n  additional_env:\n    BAD-NAME: value\n',
        '/node:\n  additional_env:\n    MODE: 1\n',
        '/node:\n  additional_env:\n    PATHS:\n      values: [one]\n',
        '/node:\n  additional_env:\n    PATHS:\n      separator: ":"\n',
        '/node:\n  additional_env:\n    PATHS:\n      separator: 1\n      values: [one]\n',
        '/node:\n  additional_env:\n    PATHS:\n      separator: ":"\n      values: []\n',
        '/node:\n  additional_env:\n    PATHS:\n      separator: ":"\n      values: [1]\n',
        '/node:\n  additional_env:\n    PATHS: "$env_var{UNCLOSED"\n',
        '/node:\n  additional_env:\n    PATHS: "$env_var{BAD-NAME}"\n',
        '/bad//node:\n  remappings: {}\n',
        '/veh*/node:\n  remappings: {}\n',
        "'':\n  remappings: {}\n",
        '/node:\n  remappings:\n    1: output\n',
        "/node:\n  remappings:\n    '': output\n",
        '/node:\n  remappings:\n    input: 1\n',
        "/node:\n  remappings:\n    input: ''\n",
        'remappings:\n  input: output\n',
        'additional_env:\n  MODE: value\n',
    ],
)
def test_invalid_configurations_include_the_file_path(tmp_path, contents):
    path = tmp_path / 'invalid.yaml'
    path.write_text(contents, encoding='utf-8')
    with pytest.raises(ValueError, match='invalid.yaml'):
        NodeConfigCollection(path)


def test_invalid_yaml_includes_the_file_path(tmp_path):
    path = tmp_path / 'invalid.yaml'
    path.write_text('/node: [\n', encoding='utf-8')
    with pytest.raises(ValueError, match='invalid.yaml'):
        NodeConfigCollection(path)


@pytest.mark.parametrize(
    'node_fqn',
    ['', '/', 'relative/node', '/trailing/', '/bad//node', '/**/node', None],
)
@pytest.mark.parametrize('getter_name', ['get_remappings', 'get_additional_env'])
def test_metadata_getters_require_an_absolute_node_fqn(node_fqn, getter_name):
    with pytest.raises(ValueError, match='fully qualified'):
        getattr(NodeConfigCollection(), getter_name)(node_fqn)


# ---------------------------------------------------------------------------
# Shipped avt_341 deployment configuration (krc_mrzr node_config.yaml)
# ---------------------------------------------------------------------------

def find_krc_mrzr_node_config() -> Path:
    for parent in Path(__file__).resolve().parents:
        candidate = (
            parent / 'avt_341' / 'parameters' / 'krc_mrzr' / 'node_config.yaml')
        if candidate.is_file():
            return candidate
    raise RuntimeError('Could not locate avt_341 krc_mrzr node_config.yaml')


KRC_MRZR_EXPECTED_REMAPPINGS = {
    'perception_local_node': [
        ('avt_341/terrain_slope', 'avt_341/terrain_slope_local'),
        ('avt_341/terrain_rms', 'avt_341/terrain_rms_local'),
    ],
    'perception_global_node': [
        ('avt_341/terrain_slope', 'avt_341/terrain_slope_global'),
        ('avt_341/terrain_rms', 'avt_341/terrain_rms_global'),
        ('avt_341/occupancy_grid', 'avt_341/occupancy_grid_low_res'),
        ('avt_341/occupancy_grid_updates',
         'avt_341/occupancy_grid_low_res_updates'),
    ],
    'perception_rms_node': [
        ('avt_341/occupancy_grid', 'avt_341/rms_perception/occupancy_grid'),
        ('avt_341/segmentation_grid',
         'avt_341/rms_perception/segmentation_grid'),
    ],
    'grid_compression_global': [
        ('avt_341/occupancy_grid', 'avt_341/occupancy_grid_low_res'),
        ('avt_341/occupied_cells', 'avt_341/occupied_cells_low_res'),
    ],
    'segmentation_grid_processor_node': [
        ('avt_341/segmentation_grid', 'avt_341/normal_segmentation_grid'),
    ],
    'mission_manager_node': [
        ('avt_341/comm_messages', '/avt_341/comm_messages'),
    ],
    'speed_zones_node': [
        ('avt_341/comm_messages', '/avt_341/comm_messages'),
    ],
    'uab_perception_node': [
        ('avt_341/points', '/ouster/points'),
        ('avt_341/camera/image_raw', '/flir_camera/image_rect_color'),
        ('avt_341/camera/camera_info', '/flir_camera/camera_info'),
        ('avt_341/odom', 'avt_341/odometry'),
        ('avt_341/occupancy_grid',
         'avt_341/terrain_seg/occupancy_grid'),
        ('avt_341/segmentation_grid',
         'avt_341/terrain_seg/segmentation_grid'),
    ],
    'object_detector_node': [
        ('image', '/flir_camera/image_rect_color'),
    ],
    'object_tracking_node': [
        ('camera_info', '/flir_camera/camera_info'),
        ('image', '/flir_camera/image_rect_color'),
        ('points/input', '/ouster/points'),
        ('detection_2d', 'detections/vision'),
        ('avt_341/reset', 'avt_341/reset'),
        ('task', 'avt_341/mission_task_state'),
        ('avt_341/odometry/estimated/odom', 'odometry/estimated'),
        ('avt_341/reset_ack', 'avt_341/reset_ack'),
    ],
}


@pytest.mark.parametrize(
    'node_name, expected', KRC_MRZR_EXPECTED_REMAPPINGS.items())
def test_shipped_krc_mrzr_config_preserves_base_launch_remappings(
        node_name, expected):
    node_config = NodeConfigCollection(find_krc_mrzr_node_config())
    assert node_config.get_remappings(f'/veh1/{node_name}') == expected


def test_shipped_krc_mrzr_tracker_section_applies_to_all_vehicles():
    node_config = NodeConfigCollection(find_krc_mrzr_node_config())
    expected = KRC_MRZR_EXPECTED_REMAPPINGS['object_tracking_node']

    assert node_config.get_remappings('/veh1/object_tracking_node') == expected
    assert node_config.get_remappings('/veh2/object_tracking_node') == expected


def test_shipped_krc_mrzr_config_leaves_unconfigured_nodes_untouched():
    node_config = NodeConfigCollection(find_krc_mrzr_node_config())
    assert node_config.get_remappings('/veh1/control_node') == []
    assert node_config.get_additional_env('/veh1/control_node') == {}


def test_shipped_krc_mrzr_config_uses_existing_matlab_environment(monkeypatch):
    monkeypatch.setenv('MCR_ROOT', '/opt/MATLAB_Runtime/R2025b')
    monkeypatch.setenv('LD_LIBRARY_PATH', '/existing/one:/existing/two')
    node_config = NodeConfigCollection(find_krc_mrzr_node_config())

    assert resolve_env(node_config, '/veh1/uab_perception_node') == {
        'LD_LIBRARY_PATH': ':'.join([
            '/opt/MATLAB_Runtime/R2025b/runtime/glnxa64',
            '/opt/MATLAB_Runtime/R2025b/bin/glnxa64',
            '/opt/MATLAB_Runtime/R2025b/sys/os/glnxa64',
            '/opt/MATLAB_Runtime/R2025b/extern/bin/glnxa64',
            '/existing/one:/existing/two',
        ]),
    }


def test_shipped_krc_mrzr_config_uses_matlab_environment_defaults(monkeypatch):
    monkeypatch.delenv('MCR_ROOT', raising=False)
    monkeypatch.delenv('LD_LIBRARY_PATH', raising=False)
    node_config = NodeConfigCollection(find_krc_mrzr_node_config())
    root = '/usr/local/MATLAB/MATLAB_Runtime/R2025b'

    assert resolve_env(node_config, '/veh1/uab_perception_node') == {
        'LD_LIBRARY_PATH': ':'.join([
            f'{root}/runtime/glnxa64',
            f'{root}/bin/glnxa64',
            f'{root}/sys/os/glnxa64',
            f'{root}/extern/bin/glnxa64',
            '',
        ]),
    }
