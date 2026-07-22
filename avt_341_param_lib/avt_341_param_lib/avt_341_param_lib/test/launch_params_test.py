import os
from collections import OrderedDict

import pytest

from avt_341_param_lib.launch_params import (
    ParameterCollection,
    _normalize_selector,
    convert_cli_value,
    convert_typed_value,
    load_template_metadata,
    relevant_params_files,
    selector_matches,
)

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
TEMPLATE_A = os.path.join(TEST_DIR, 'launch_template_a.yaml')
TEMPLATE_B = os.path.join(TEST_DIR, 'launch_template_b.yaml')

VEHICLES = ['veh1', 'veh2']


def make_collection() -> ParameterCollection:
    # planner and controller share template a, sensor uses template b
    return ParameterCollection.from_node_templates({
        'planner': TEMPLATE_A,
        'controller': TEMPLATE_A,
        'sensor': TEMPLATE_B,
    })


def resolve(cli_args, vehicles=VEHICLES):
    return make_collection().resolve(OrderedDict(cli_args), vehicles)


def test_metadata():
    specs = load_template_metadata(TEMPLATE_A)
    assert set(specs.keys()) == {'cruise_speed', 'planner.mode', 'ids'}
    assert specs['cruise_speed'].param_type == 'double'
    assert specs['cruise_speed'].default_value == 1.5
    assert specs['planner.mode'].param_type == 'string'
    assert specs['planner.mode'].default_value == 'grid'
    assert specs['ids'].param_type == 'int_array'
    assert specs['ids'].default_value == [1, 2]


def test_metadata_rejects_non_template_file(tmp_path):
    bad = tmp_path / 'not_a_template.yaml'
    bad.write_text('just_a_key: 5\n')
    with pytest.raises(ValueError):
        load_template_metadata(str(bad))


def test_argument_names():
    names = make_collection().argument_names()
    assert 'planner/cruise_speed' in names
    assert 'planner/planner.mode' in names
    assert 'controller/cruise_speed' in names
    assert 'sensor/enabled' in names
    # none-typed and mapped parameters are not declared
    assert not any('unused_external' in name or '__map_' in name for name in names)


def test_selector_matches():
    assert selector_matches('/veh1/planner', '/veh1/planner')
    assert not selector_matches('/veh1/planner', '/veh2/planner')
    assert selector_matches('/**', '/veh1/planner')
    assert selector_matches('/veh1/**', '/veh1/planner')
    assert not selector_matches('/veh1/**', '/veh2/planner')
    assert selector_matches('/**/planner', '/veh1/planner')
    assert selector_matches('/**/planner', '/planner')
    assert not selector_matches('/**/planner', '/veh1/controller')
    assert selector_matches('/*/planner', '/veh1/planner')
    assert not selector_matches('/*/planner', '/a/b/planner')


def test_normalize_selector():
    # relative, not a vehicle: single-token wildcard for the vehicle id only
    assert _normalize_selector('planner', VEHICLES) == '/*/planner'
    # deeper relative paths address nested node namespaces below the vehicle
    assert _normalize_selector('nav/planner', VEHICLES) == '/*/nav/planner'
    # relative, vehicle-scoped: anchored without wildcards
    assert _normalize_selector('veh1/planner', VEHICLES) == '/veh1/planner'
    # bare vehicle namespace: all of that vehicle's nodes
    assert _normalize_selector('veh1', VEHICLES) == '/veh1/**'
    assert _normalize_selector('/veh1', VEHICLES) == '/veh1/**'
    # explicit wildcards and absolute selectors pass through
    assert _normalize_selector('**/planner', VEHICLES) == '/**/planner'
    assert _normalize_selector('/veh1/planner', VEHICLES) == '/veh1/planner'


def test_resolve_global_scalar():
    overrides = resolve([('**/cruise_speed', '9.0')])
    for fqn in ('/veh1/planner', '/veh1/controller', '/veh2/planner', '/veh2/controller'):
        assert overrides.for_node(fqn) == {'cruise_speed': 9.0}
    # sensor does not declare cruise_speed: dormant, like a yaml file entry
    assert overrides.for_node('/veh1/sensor') == {}


def test_resolve_relative_node_selector():
    overrides = resolve([('planner/cruise_speed', '8.0')])
    assert overrides.for_node('/veh1/planner') == {'cruise_speed': 8.0}
    assert overrides.for_node('/veh2/planner') == {'cruise_speed': 8.0}
    assert overrides.for_node('/veh1/controller') == {}


def test_resolve_vehicle_scoped_scalar():
    overrides = resolve([('veh1/planner/cruise_speed', '7.5')])
    assert overrides.for_node('/veh1/planner') == {'cruise_speed': 7.5}
    assert overrides.for_node('/veh2/planner') == {}


def test_resolve_bare_vehicle_mapping():
    overrides = resolve([('veh1', '{cruise_speed: 3.25}')])
    assert overrides.for_node('/veh1/planner') == {'cruise_speed': 3.25}
    assert overrides.for_node('/veh1/controller') == {'cruise_speed': 3.25}
    assert overrides.for_node('/veh2/planner') == {}


def test_resolve_mapping_with_nested_group():
    overrides = resolve([('/**', '{planner: {mode: graph}}')])
    assert overrides.for_node('/veh1/planner') == {'planner.mode': 'graph'}
    assert overrides.for_node('/veh2/controller') == {'planner.mode': 'graph'}
    assert overrides.for_node('/veh1/sensor') == {}


def test_resolve_mapping_with_explicit_ros_parameters_key():
    overrides = resolve([('/veh2/sensor', '{ros__parameters: {enabled: false}}')])
    assert overrides.for_node('/veh2/sensor') == {'enabled': False}


def test_resolve_entry_order_wins():
    overrides = resolve([
        ('**/cruise_speed', '2.0'),
        ('veh1/planner/cruise_speed', '3.0'),
    ])
    assert overrides.for_node('/veh1/planner') == {'cruise_speed': 3.0}
    assert overrides.for_node('/veh2/planner') == {'cruise_speed': 2.0}

    overrides = resolve([
        ('veh1/planner/cruise_speed', '3.0'),
        ('**/cruise_speed', '2.0'),
    ])
    assert overrides.for_node('/veh1/planner') == {'cruise_speed': 2.0}


def test_resolve_converts_to_declared_types():
    overrides = resolve([('planner/cruise_speed', '9')])
    value = overrides.for_node('/veh1/planner')['cruise_speed']
    assert isinstance(value, float) and value == 9.0
    # string parameters take mapping-form values via str conversion
    overrides = resolve([('planner', '{planner: {mode: graph}}')])
    assert overrides.for_node('/veh1/planner') == {'planner.mode': 'graph'}


def test_resolve_rejects_unknown_parameter():
    with pytest.raises(RuntimeError, match='cruse_speed'):
        resolve([('planner/cruse_speed', '1.0')])


def test_resolve_rejects_unmatched_selector():
    with pytest.raises(RuntimeError, match='matches none'):
        resolve([('ghost/cruise_speed', '1.0')])


def test_resolve_rejects_partial_wildcard_token():
    with pytest.raises(RuntimeError, match='Unsupported wildcard'):
        resolve([('/ve*1/planner/cruise_speed', '1.0')])


def test_resolve_rejects_malformed_key():
    with pytest.raises(RuntimeError, match='Malformed'):
        resolve([('**', '5')])


class FakeContext:
    def __init__(self, configs):
        self.launch_configurations = configs


def test_resolve_cli_overrides_uses_snapshot_only():
    pargs = make_collection()
    # user-provided arguments arrive before declaration: snapshot them
    context = FakeContext(OrderedDict([('**/cruise_speed', '9.0')]))
    pargs._snapshot_cmd_args(context)
    # declaration then injects defaults, which must not be treated as explicit
    context.launch_configurations['planner/cruise_speed'] = '1.5'
    overrides = pargs.resolve_cli_overrides(context, VEHICLES)
    assert overrides.for_node('/veh1/planner') == {'cruise_speed': 9.0}


def test_resolve_cli_overrides_accepts_bare_vehicle_and_node_keys():
    pargs = make_collection()
    context = FakeContext(OrderedDict([
        ('vehicle_ids', '[veh1, veh2]'),  # ordinary argument: ignored
        ('veh1', '{cruise_speed: 3.25}'),
        ('planner', '{planner: {mode: graph}}'),
    ]))
    pargs._snapshot_cmd_args(context)
    overrides = pargs.resolve_cli_overrides(context, VEHICLES)
    assert overrides.for_node('/veh1/controller') == {'cruise_speed': 3.25}
    assert overrides.for_node('/veh1/planner') == {
        'cruise_speed': 3.25, 'planner.mode': 'graph'}
    assert overrides.for_node('/veh2/planner') == {'planner.mode': 'graph'}


def test_relevant_params_files(tmp_path):
    everyone = tmp_path / 'everyone.yaml'
    everyone.write_text('/**:\n  ros__parameters:\n    cruise_speed: 2.5\n')
    veh2_only = tmp_path / 'veh2_only.yaml'
    veh2_only.write_text('/veh2/**:\n  ros__parameters:\n    cruise_speed: 0.5\n')
    nested_style = tmp_path / 'nested_style.yaml'
    nested_style.write_text('veh1:\n  planner:\n    ros__parameters:\n      cruise_speed: 1.0\n')
    files = [str(everyone), str(veh2_only), str(nested_style)]

    assert relevant_params_files(files, '/veh1/planner') == [str(everyone), str(nested_style)]
    assert relevant_params_files(files, '/veh2/planner') == [str(everyone), str(veh2_only)]
    assert relevant_params_files(files, '/veh2/sensor') == [str(everyone), str(veh2_only)]


def test_convert_scalars():
    assert convert_cli_value('9.0', 'double', 'a') == 9.0
    assert convert_cli_value('9', 'double', 'a') == 9.0
    assert convert_cli_value('5', 'int', 'a') == 5
    assert convert_cli_value('true', 'bool', 'a') is True
    # string parameters take the raw text verbatim, even numeric-looking ones
    assert convert_cli_value('5', 'string', 'a') == '5'


def test_convert_arrays():
    assert convert_cli_value('[1.0, 2]', 'double_array', 'a') == [1.0, 2.0]
    assert convert_cli_value('[a, b]', 'string_array', 'a') == ['a', 'b']


def test_convert_type_mismatch():
    with pytest.raises(ValueError):
        convert_cli_value('not_a_number', 'double', 'a')
    with pytest.raises(ValueError):
        convert_cli_value('5', 'bool', 'a')
    with pytest.raises(ValueError):
        convert_cli_value('1.0', 'double_array', 'a')


def test_convert_typed_values():
    assert convert_typed_value(9, 'double', 'a') == 9.0
    assert convert_typed_value([1, 2], 'int_array', 'a') == [1, 2]
    with pytest.raises(ValueError):
        convert_typed_value('nope', 'double', 'a')
