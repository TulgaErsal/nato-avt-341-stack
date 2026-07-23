import os

import pytest
import yaml

from avt_341_param_lib.parse_runtime_yaml import (
    _patterns_intersect,
    resolve_params_files,
)

VEHICLES = ['veh1', 'veh2']


def write(tmp_path, name, text):
    path = tmp_path / name
    path.write_text(text)
    return str(path)


def resolved_doc(paths, vehicles=VEHICLES, work_dir=None):
    resolved = resolve_params_files(paths, vehicles, work_dir)
    with open(resolved[0]) as f:
        return yaml.safe_load(f)


def test_passthrough_returns_original_path(tmp_path):
    plain = write(tmp_path, 'plain.yaml', '/**:\n  ros__parameters:\n    max_speed: 4.0\n')
    assert resolve_params_files([plain], VEHICLES) == [plain]


def test_python_expression(tmp_path):
    path = write(tmp_path, 'a.yaml', (
        '/**:\n'
        '  ros__parameters:\n'
        "    data_file: $python{os.path.join('base', 'file.csv')}\n"
        '    count: $python{2 + 3}\n'
        '    has_share_lookup: $python{callable(get_package_share_directory)}\n'
    ))
    doc = resolved_doc([path])
    params = doc['/**']['ros__parameters']
    assert params['data_file'] == os.path.join('base', 'file.csv')
    assert params['count'] == 5
    assert params['has_share_lookup'] is True


def test_python_error_reports_expression(tmp_path):
    path = write(tmp_path, 'a.yaml', '/**:\n  ros__parameters:\n    x: $python{1 / 0}\n')
    with pytest.raises(RuntimeError, match=r'\$python\{1 / 0\}'):
        resolve_params_files([path], VEHICLES)


def test_ref_concrete_selector(tmp_path):
    path = write(tmp_path, 'a.yaml', (
        '/veh1/planner:\n'
        '  ros__parameters:\n'
        '    cruise_speed: 3.3\n'
        '/veh1/controller:\n'
        '  ros__parameters:\n'
        '    speed_limit: $ref{veh1/planner/cruise_speed}\n'
    ))
    doc = resolved_doc([path])
    assert doc['/veh1/controller']['ros__parameters']['speed_limit'] == 3.3


def test_ref_relative_selector_normalized_against_vehicles(tmp_path):
    # 'planner' is not a vehicle id -> /*/planner, matching /veh2/planner
    path = write(tmp_path, 'a.yaml', (
        '/veh2/planner:\n'
        '  ros__parameters:\n'
        '    cruise_speed: 7.5\n'
        '/**:\n'
        '  ros__parameters:\n'
        '    copied: $ref{planner/cruise_speed}\n'
    ))
    doc = resolved_doc([path])
    assert doc['/**']['ros__parameters']['copied'] == 7.5


def test_ref_wildcard_first_match_wins_across_files_and_sections(tmp_path):
    first = write(tmp_path, 'first.yaml', (
        '/**:\n'
        '  ros__parameters:\n'
        '    other: 0\n'
        '/**/planner:\n'
        '  ros__parameters:\n'
        '    max_speed: 1.0\n'
        '/**/controller:\n'
        '  ros__parameters:\n'
        '    max_speed: 2.0\n'
    ))
    second = write(tmp_path, 'second.yaml', (
        '/**:\n'
        '  ros__parameters:\n'
        '    max_speed: 3.0\n'
        '    copied: $ref{**/max_speed}\n'
    ))
    resolved = resolve_params_files([first, second], VEHICLES)
    assert resolved[0] == first  # untouched file passes through
    with open(resolved[1]) as f:
        doc = yaml.safe_load(f)
    assert doc['/**']['ros__parameters']['copied'] == 1.0  # first file, first section


def test_ref_dotted_nested_and_flat_keys(tmp_path):
    path = write(tmp_path, 'a.yaml', (
        '/**/perception:\n'
        '  ros__parameters:\n'
        '    clear_method:\n'
        '      type: raytrace\n'
        '/**/other:\n'
        '  ros__parameters:\n'
        '    flat.key: 11\n'
        '/**:\n'
        '  ros__parameters:\n'
        '    a: $ref{**/clear_method.type}\n'
        '    b: $ref{**/flat.key}\n'
    ))
    doc = resolved_doc([path])
    assert doc['/**']['ros__parameters']['a'] == 'raytrace'
    assert doc['/**']['ros__parameters']['b'] == 11


def test_ref_nested_namespace_section_form(tmp_path):
    path = write(tmp_path, 'a.yaml', (
        'veh2:\n'
        '  planner:\n'
        '    ros__parameters:\n'
        '      cruise_speed: 9.0\n'
        '/**:\n'
        '  ros__parameters:\n'
        '    copied: $ref{veh2/planner/cruise_speed}\n'
    ))
    doc = resolved_doc([path])
    assert doc['/**']['ros__parameters']['copied'] == 9.0


def test_ref_chain_to_python_and_ref_across_files(tmp_path):
    # the first file references a not-yet-resolved template chain in the second
    first = write(tmp_path, 'first.yaml', (
        '/**:\n'
        '  ros__parameters:\n'
        '    leaf: $ref{**/mid}\n'
    ))
    second = write(tmp_path, 'second.yaml', (
        '/**/planner:\n'
        '  ros__parameters:\n'
        '    mid: $ref{**/root}\n'
        "    root: $python{10 * 2}\n"
    ))
    resolved = resolve_params_files([first, second], VEHICLES)
    with open(resolved[0]) as f:
        first_doc = yaml.safe_load(f)
    with open(resolved[1]) as f:
        second_doc = yaml.safe_load(f)
    assert first_doc['/**']['ros__parameters']['leaf'] == 20
    assert second_doc['/**/planner']['ros__parameters']['mid'] == 20
    assert second_doc['/**/planner']['ros__parameters']['root'] == 20


def test_ref_cycle_raises(tmp_path):
    path = write(tmp_path, 'a.yaml', (
        '/**/planner:\n'
        '  ros__parameters:\n'
        '    a: $ref{**/b}\n'
        '    b: $ref{**/a}\n'
    ))
    with pytest.raises(RuntimeError, match='Circular'):
        resolve_params_files([path], VEHICLES)


def test_ref_no_match_raises(tmp_path):
    path = write(tmp_path, 'a.yaml', '/**:\n  ros__parameters:\n    x: $ref{**/missing}\n')
    with pytest.raises(RuntimeError, match='matches no parameter'):
        resolve_params_files([path], VEHICLES)


def test_ref_selector_must_not_match_wrong_node(tmp_path):
    path = write(tmp_path, 'a.yaml', (
        '/veh1/planner:\n'
        '  ros__parameters:\n'
        '    cruise_speed: 3.3\n'
        '/**:\n'
        '  ros__parameters:\n'
        '    x: $ref{veh1/controller/cruise_speed}\n'
    ))
    with pytest.raises(RuntimeError, match='matches no parameter'):
        resolve_params_files([path], VEHICLES)


def test_malformed_templates_raise(tmp_path):
    no_selector = write(tmp_path, 'a.yaml', '/**:\n  ros__parameters:\n    x: $ref{max_speed}\n')
    with pytest.raises(RuntimeError, match='Malformed reference'):
        resolve_params_files([no_selector], VEHICLES)
    partial = write(
        tmp_path, 'b.yaml', "/**:\n  ros__parameters:\n    x: '$python{1} tail'\n")
    with pytest.raises(RuntimeError, match='entire value'):
        resolve_params_files([partial], VEHICLES)


def test_resolved_output_is_valid_params_yaml_without_residue(tmp_path):
    work_dir = tmp_path / 'out'
    work_dir.mkdir()
    path = write(tmp_path, 'a.yaml', (
        '/**:\n'
        '  ros__parameters:\n'
        "    data_file: $python{os.path.join('base', 'file.csv')}\n"
        '    plain: kept\n'
    ))
    resolved = resolve_params_files([path], VEHICLES, str(work_dir))
    assert resolved[0] != path
    assert os.path.basename(resolved[0]) == '00_a.yaml'
    text = open(resolved[0]).read()
    assert '$python{' not in text and '$ref{' not in text
    doc = yaml.safe_load(text)
    assert doc['/**']['ros__parameters']['plain'] == 'kept'


def test_patterns_intersect():
    assert _patterns_intersect(['**'], ['veh1', 'planner'])
    assert _patterns_intersect(['**', 'planner'], ['veh1', 'planner'])
    assert _patterns_intersect(['*', 'planner'], ['**', 'planner'])
    assert _patterns_intersect(['**'], [])
    assert not _patterns_intersect(['veh1', 'planner'], ['veh2', 'planner'])
    assert not _patterns_intersect(['*'], [])
    assert not _patterns_intersect(['veh1'], ['veh1', 'planner'])
