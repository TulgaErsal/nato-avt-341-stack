import os
from pathlib import Path

import pytest
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

from avt_341_param_lib.launch_metadata import MetadataCollection

METADATA_FILE = (
    Path(__file__).resolve().parents[1]
    / 'parameters'
    / 'metadata'
    / 'krc_mrzr.yaml'
)

EXPECTED_REMAPPINGS = {
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


class SubstitutionContext:

    @property
    def environment(self):
        return os.environ

    def perform_substitution(self, substitution):
        return substitution.perform(self)


def resolve_env(metadata, node_fqn):
    context = SubstitutionContext()
    return {
        name: perform_substitutions(
            context, normalize_to_list_of_substitutions(value))
        for name, value in metadata.get_additional_env(node_fqn).items()
    }


@pytest.mark.parametrize('node_name, expected', EXPECTED_REMAPPINGS.items())
def test_shipped_metadata_preserves_base_launch_remappings(node_name, expected):
    metadata = MetadataCollection(METADATA_FILE)
    assert metadata.get_remappings(f'/veh1/{node_name}') == expected


def test_tracker_wildcard_section_applies_to_multiple_vehicle_namespaces():
    metadata = MetadataCollection(METADATA_FILE)
    expected = EXPECTED_REMAPPINGS['object_tracking_node']

    assert metadata.get_remappings('/veh1/object_tracking_node') == expected
    assert metadata.get_remappings('/veh2/object_tracking_node') == expected


def test_unconfigured_node_has_no_metadata():
    metadata = MetadataCollection(METADATA_FILE)
    assert metadata.get_remappings('/veh1/control_node') == []
    assert metadata.get_additional_env('/veh1/control_node') == {}


def test_shipped_metadata_uses_existing_matlab_environment(monkeypatch):
    monkeypatch.setenv('MCR_ROOT', '/opt/MATLAB_Runtime/R2025b')
    monkeypatch.setenv('LD_LIBRARY_PATH', '/existing/one:/existing/two')
    metadata = MetadataCollection(METADATA_FILE)

    assert resolve_env(metadata, '/veh1/uab_perception_node') == {
        'LD_LIBRARY_PATH': ':'.join([
            '/opt/MATLAB_Runtime/R2025b/runtime/glnxa64',
            '/opt/MATLAB_Runtime/R2025b/bin/glnxa64',
            '/opt/MATLAB_Runtime/R2025b/sys/os/glnxa64',
            '/opt/MATLAB_Runtime/R2025b/extern/bin/glnxa64',
            '/existing/one:/existing/two',
        ]),
    }


def test_shipped_metadata_uses_matlab_environment_defaults(monkeypatch):
    monkeypatch.delenv('MCR_ROOT', raising=False)
    monkeypatch.delenv('LD_LIBRARY_PATH', raising=False)
    metadata = MetadataCollection(METADATA_FILE)
    root = '/usr/local/MATLAB/MATLAB_Runtime/R2025b'

    assert resolve_env(metadata, '/veh1/uab_perception_node') == {
        'LD_LIBRARY_PATH': ':'.join([
            f'{root}/runtime/glnxa64',
            f'{root}/bin/glnxa64',
            f'{root}/sys/os/glnxa64',
            f'{root}/extern/bin/glnxa64',
            '',
        ]),
    }
