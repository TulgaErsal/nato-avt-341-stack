import os
from collections import OrderedDict
from pathlib import Path

import pytest
from ament_index_python.packages import PackageNotFoundError

from avt_341_param_lib.codegen.generate_cpp_header import run as run_cpp
from avt_341_param_lib.codegen.generate_cpp_mixin_header import run as run_mixin_cpp
from avt_341_param_lib.runtime.launch_params import (
    ParameterCollection,
    _normalize_selector,
    convert_cli_value,
    convert_typed_value,
    load_template_specs,
    relevant_params_files,
    selector_matches,
)

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
TEMPLATE_A = os.path.join(TEST_DIR, 'launch_template_a.yaml')
TEMPLATE_B = os.path.join(TEST_DIR, 'launch_template_b.yaml')

# Multi-level template fixture (with mixins/fixture_geometry_mixin.yaml beside
# it). Owned by these tests -- see the header of the yaml file. A real node
# template must not be substituted here: those change whenever a node gains or
# renames a parameter, which would make these expectations churn.
TEMPLATE_NESTED = os.path.join(TEST_DIR, 'launch_template_nested.yaml')
MIXIN_NESTED = os.path.join(TEST_DIR, 'mixins', 'fixture_geometry_mixin.yaml')

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
    specs = load_template_specs(TEMPLATE_A)
    assert set(specs.keys()) == {'cruise_speed', 'planner.mode', 'ids'}
    assert specs['cruise_speed'].param_type == 'double'
    assert specs['cruise_speed'].default_value == 1.5
    assert specs['planner.mode'].param_type == 'string'
    assert specs['planner.mode'].default_value == 'grid'
    assert specs['ids'].param_type == 'int_array'
    assert specs['ids'].default_value == [1, 2]


def test_multi_template_node_merges_specs():
    # a node may list several templates (executable linking several generated
    # parameter services); overrides validate against the union
    collection = ParameterCollection.from_node_templates({
        'planner': TEMPLATE_A,
        'combined': [TEMPLATE_A, TEMPLATE_B],
    })
    names = set(collection.argument_names())
    assert 'combined/cruise_speed' in names
    assert 'combined/enabled' in names
    assert 'planner/enabled' not in names

    overrides = collection.resolve(OrderedDict([
        ('combined/cruise_speed', '3.5'),
        ('combined/enabled', 'false'),
    ]), VEHICLES)
    assert overrides.for_node('/veh1/combined') == {
        'cruise_speed': 3.5, 'enabled': False}
    assert overrides.for_node('/veh1/planner') == {}

    # a single-template node still rejects the other template's parameters
    with pytest.raises(RuntimeError, match='enabled'):
        collection.resolve(OrderedDict([('planner/enabled', 'false')]), VEHICLES)


def test_multi_template_duplicate_parameter_raises():
    with pytest.raises(ValueError, match='declared by several templates'):
        ParameterCollection.from_node_templates({
            'combined': [TEMPLATE_A, TEMPLATE_A],
        })


# ---------------------------------------------------------------------------
# Whole-template behaviour against the checked-in nested fixture
#
# The tests above cover one feature per one-off template. The tests below run a
# single complete template (several nesting levels, multi-word group names, a
# mixin include and a spread of scalar types) through spec loading, launch
# argument resolution and C++ generation, to check those features compose.
# ---------------------------------------------------------------------------

# Every parameter the fixture declares: dotted name -> (type, default value).
# Names ending in a group that came from mixins/fixture_geometry_mixin.yaml are
# marked; they must appear exactly as if declared inline.
NESTED_EXPECTED_SPECS = {
    'update_rate':                            ('double', 10.0),
    'tracking.target_timeout':                ('double', 1.5),
    'tracking.max_targets':                   ('int', 8),
    'target_selection.formation_vehicle_ids': ('string_array', []),
    'target_selection.enabled':               ('bool', True),
    'fsc.type':                               ('string', 'none'),
    'fsc.oof.threshold':                      ('double', 0.75),
    'costmap.thresholds.thresh':              ('float', 0.5),
    'costmap.terrain_rms.hfov':               ('float', 1.57),
    'costmap.frame_id':                       ('string', 'odom'),   # from mixin
    'costmap.geometry.res':                   ('float', 0.25),      # from mixin
    'costmap.geometry.width':                 ('float', 100.0),     # from mixin
}


def test_nested_template_specs_use_dotted_names_only():
    specs = load_template_specs(TEMPLATE_NESTED)

    # every parameter is addressed by its full dotted path, with the declared
    # type and default carried through
    assert set(specs) == set(NESTED_EXPECTED_SPECS)
    for name, (param_type, default) in NESTED_EXPECTED_SPECS.items():
        assert specs[name].param_type == param_type, name
        assert specs[name].default_value == default, name

    # ...and no bare leaf name leaks in as an alias. `type` also confirms that a
    # parameter literally named `type` is treated as a parameter rather than as
    # the type declaration of its enclosing group.
    for bare in ('target_timeout', 'max_targets', 'formation_vehicle_ids',
                 'enabled', 'threshold', 'res', 'width', 'thresh', 'hfov',
                 'frame_id', 'type', 'oof', 'geometry'):
        assert bare not in specs


def test_nested_template_mixin_content_is_spliced_into_the_group():
    specs = load_template_specs(TEMPLATE_NESTED)

    # a mixin's root-level group mounts under the including group...
    assert specs['costmap.geometry.res'].param_type == 'float'
    assert specs['costmap.geometry.res'].default_value == 0.25
    assert specs['costmap.geometry.width'].default_value == 100.0
    # ...and a mixin's root-level leaf splices in as a plain scalar
    assert specs['costmap.frame_id'].param_type == 'string'
    assert specs['costmap.frame_id'].default_value == 'odom'
    # the mixin mounts under `costmap`, not at the template root
    assert 'geometry.res' not in specs
    assert 'frame_id' not in specs


def test_nested_template_generates_a_struct_per_group(tmp_path):
    dto_output = tmp_path / 'nested_params_dto.hpp'
    service_output = tmp_path / 'nested_params_service.hpp'
    run_cpp(str(dto_output), str(service_output), TEMPLATE_NESTED)
    dto = dto_output.read_text()
    service = service_output.read_text()

    assert 'struct Tracking' in dto
    assert 'struct Costmap' in dto
    assert 'struct Thresholds' in dto
    # multi-word group names become PascalCase
    assert 'struct TargetSelection' in dto
    assert 'struct TerrainRms' in dto
    # third-level group nests inside its parent's struct
    assert 'struct Fsc' in dto
    assert 'struct Oof' in dto

    # the generated field keeps the yaml type
    assert 'std::vector<std::string> formation_vehicle_ids = {};' in dto
    assert 'std::string type = "none";' in dto

    # ROS declares the parameters under their dotted names
    for declared in ('"update_rate"', '"tracking.target_timeout"',
                     '"target_selection.formation_vehicle_ids"',
                     '"fsc.type"', '"fsc.oof.threshold"',
                     '"costmap.geometry.res"', '"costmap.terrain_rms.hfov"'):
        assert declared in service


def test_nested_template_dto_references_shared_mixin_structs(tmp_path):
    dto_output = tmp_path / 'nested_params_dto.hpp'
    service_output = tmp_path / 'nested_params_service.hpp'
    run_cpp(str(dto_output), str(service_output), TEMPLATE_NESTED)
    dto = dto_output.read_text()

    # the mixin group is referenced through the shared DTO rather than being
    # re-defined inline. Only the Params tree is checked: StackParams is a plain
    # value-copy mirror and always inlines its sub-structs (parse_yaml.py adds
    # the ExternalStructMember to the Params tree only).
    params_dto = dto.split('struct StackParams')[0]
    assert '#include "fixture_geometry_mixin_params_dto.hpp"' in dto
    assert 'fixture::params::shared::Geometry geometry;' in params_dto
    assert 'struct Geometry' not in params_dto
    # the mixin's root-level leaf is an ordinary field of the including struct
    assert 'std::string frame_id = "odom";' in params_dto


def test_nested_template_mixin_fragment_header(tmp_path):
    output = tmp_path / 'fixture_geometry_mixin_params_dto.hpp'
    run_mixin_cpp(str(output), MIXIN_NESTED)
    header = output.read_text()

    assert 'namespace fixture::params::shared' in header
    assert 'struct Geometry' in header
    # only groups become shared structs; root-level leaves are not emitted here
    assert 'frame_id' not in header


def test_nested_template_launch_overrides_use_dotted_names():
    collection = ParameterCollection.from_node_templates({
        'nested_node': TEMPLATE_NESTED,
    })
    overrides = collection.resolve(OrderedDict([
        ('nested_node/tracking.target_timeout', '2.5'),
        ('nested_node/target_selection.formation_vehicle_ids', '[veh1, veh2]'),
        ('nested_node/target_selection.enabled', 'false'),
        ('nested_node/fsc.type', 'strict'),
        ('nested_node/fsc.oof.threshold', '0.25'),
        ('nested_node/costmap.geometry.res', '0.5'),
        ('nested_node/costmap.terrain_rms.hfov', '1.0'),
    ]), VEHICLES)

    expected = {
        'tracking.target_timeout': 2.5,
        'target_selection.formation_vehicle_ids': VEHICLES,
        'target_selection.enabled': False,
        'fsc.type': 'strict',
        'fsc.oof.threshold': 0.25,
        'costmap.geometry.res': 0.5,
        'costmap.terrain_rms.hfov': 1.0,
    }
    assert overrides.for_node('/veh1/nested_node') == expected
    assert overrides.for_node('/veh2/nested_node') == expected


@pytest.mark.parametrize(
    'bad_name',
    [
        'target_timeout',          # bare leaf of a nested group
        'threshold',               # bare leaf of a third-level group
        'tracking_target_timeout',  # underscore instead of dot
        'tracking.missing',        # unknown leaf in a known group
        'missing.target_timeout',  # known leaf under an unknown group
    ],
)
def test_nested_template_rejects_names_that_are_not_declared(bad_name):
    collection = ParameterCollection.from_node_templates({
        'nested_node': TEMPLATE_NESTED,
    })
    with pytest.raises(RuntimeError, match=bad_name.split('.')[-1]):
        collection.resolve(
            OrderedDict([(f'nested_node/{bad_name}', '1.0')]), VEHICLES)


MIXIN_TEXT = """
code_namespace: avt_341/params/core
ros__parameters:
  costmap_info:
    geometry:
      res:
        type: float
        default_value: 0.25
        description: "Grid resolution."
        validation:
          gt<>: [ 0.0 ]
    transmission:
      type: string
      default_value: 'window'
      description: "Transmission method."
"""

TEMPLATE_WITH_MIXIN = """
code_namespace: demo
ros__parameters:
  __include_mixins: costmap_info_mixin
  rate:
    type: double
    default_value: 10.0
"""


def write_template_with_mixin(tmp_path, template_text=TEMPLATE_WITH_MIXIN,
                              mixin_text=MIXIN_TEXT, stem='costmap_info_mixin'):
    mixins_dir = tmp_path / 'mixins'
    mixins_dir.mkdir(exist_ok=True)
    (mixins_dir / f'{stem}.yaml').write_text(mixin_text)
    template = tmp_path / 'node.yaml'
    template.write_text(template_text)
    return str(template)


def test_load_template_specs_expands_mixins(tmp_path):
    specs = load_template_specs(write_template_with_mixin(tmp_path))
    assert set(specs) == {
        'rate', 'costmap_info.geometry.res', 'costmap_info.transmission'}
    assert specs['costmap_info.geometry.res'].param_type == 'float'
    assert specs['costmap_info.transmission'].default_value == 'window'


def test_load_template_specs_expands_nested_mixin_list(tmp_path):
    limits_mixin = (
        'code_namespace: avt_341/params/core\n'
        'ros__parameters:\n'
        '  limits:\n'
        '    max:\n'
        '      type: double\n'
        '      default_value: 5.0\n'
    )
    template = (
        'code_namespace: demo\n'
        'ros__parameters:\n'
        '  group:\n'
        '    __include_mixins: [costmap_info_mixin, limits]\n'
    )
    path = write_template_with_mixin(tmp_path, template)
    (tmp_path / 'mixins' / 'limits.yaml').write_text(limits_mixin)
    specs = load_template_specs(path)
    assert set(specs) == {
        'group.costmap_info.geometry.res',
        'group.costmap_info.transmission',
        'group.limits.max',
    }


def test_include_mixins_comma_separated_string(tmp_path):
    limits_mixin = (
        'code_namespace: avt_341/params/core\n'
        'ros__parameters:\n'
        '  limits:\n'
        '    max:\n'
        '      type: double\n'
        '      default_value: 5.0\n'
    )
    template = (
        'code_namespace: demo\n'
        'ros__parameters:\n'
        '  __include_mixins: costmap_info_mixin, limits\n'
    )
    path = write_template_with_mixin(tmp_path, template)
    (tmp_path / 'mixins' / 'limits.yaml').write_text(limits_mixin)
    specs = load_template_specs(path)
    assert set(specs) == {
        'costmap_info.geometry.res',
        'costmap_info.transmission',
        'limits.max',
    }


def test_include_mixin_missing_file_raises(tmp_path):
    template = tmp_path / 'node.yaml'
    template.write_text(
        'code_namespace: demo\nros__parameters:\n  __include_mixins: nope\n')
    with pytest.raises(Exception, match='not found'):
        load_template_specs(str(template))


def test_include_mixin_key_collision_raises(tmp_path):
    template = TEMPLATE_WITH_MIXIN + (
        '  costmap_info:\n'
        '    other:\n'
        '      type: double\n'
        '      default_value: 1.0\n'
    )
    with pytest.raises(Exception, match='already exists'):
        load_template_specs(write_template_with_mixin(tmp_path, template))


def test_mixin_nested_include_mixin_raises(tmp_path):
    nested_mixin = (
        'code_namespace: avt_341/params/core\n'
        'ros__parameters:\n'
        '  __include_mixins: other\n'
    )
    with pytest.raises(Exception, match='cannot include other mixins'):
        load_template_specs(
            write_template_with_mixin(tmp_path, mixin_text=nested_mixin))


def test_generated_cpp_references_shared_mixin_structs(tmp_path):
    template = write_template_with_mixin(tmp_path)
    dto_output = tmp_path / 'node_params_dto.hpp'
    service_output = tmp_path / 'node_params_service.hpp'
    run_cpp(str(dto_output), str(service_output), template)
    dto = dto_output.read_text()
    service = service_output.read_text()
    # the mixin group references the shared struct instead of re-defining it
    # inline; the single remaining inline definition is StackParams' internal
    # copy, which intentionally stays per-template
    assert '#include "costmap_info_mixin_params_dto.hpp"' in dto
    assert 'avt_341::params::core::CostmapInfo costmap_info;' in dto
    assert dto.count('struct CostmapInfo') == 1
    # the parameter fragments still cover the mixin's parameters
    assert '"costmap_info.geometry.res"' in service
    assert '"costmap_info.transmission"' in service


def test_generated_mixin_fragment_header(tmp_path):
    mixins_dir = tmp_path / 'mixins'
    mixins_dir.mkdir(exist_ok=True)
    mixin = mixins_dir / 'costmap_info_mixin.yaml'
    mixin.write_text(MIXIN_TEXT)
    output = tmp_path / 'costmap_info_mixin_params_dto.hpp'
    run_mixin_cpp(str(output), str(mixin))
    header = output.read_text()
    assert 'namespace avt_341::params::core' in header
    assert 'struct CostmapInfo' in header
    assert 'struct Geometry' in header
    assert 'float res = 0.25F;' in header
    # type-only definitions: no member instance, no listener baggage
    assert '} costmap_info;' not in header
    assert '__stamp' not in header


def test_metadata_rejects_non_template_file(tmp_path):
    bad = tmp_path / 'not_a_template.yaml'
    bad.write_text('just_a_key: 5\n')
    with pytest.raises(ValueError):
        load_template_specs(str(bad))


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


def test_resolve_bare_parameter_name():
    # a bare declared parameter name is shorthand for **/<param.name>
    overrides = resolve([('cruise_speed', '9.0')])
    for fqn in ('/veh1/planner', '/veh1/controller', '/veh2/planner', '/veh2/controller'):
        assert overrides.for_node(fqn) == {'cruise_speed': 9.0}
    # sensor does not declare cruise_speed: dormant, like a yaml file entry
    assert overrides.for_node('/veh1/sensor') == {}


def test_resolve_bare_dotted_parameter_name():
    overrides = resolve([('planner.mode', 'graph')])
    assert overrides.for_node('/veh1/planner') == {'planner.mode': 'graph'}
    assert overrides.for_node('/veh2/controller') == {'planner.mode': 'graph'}
    assert overrides.for_node('/veh1/sensor') == {}


def test_resolve_rejects_bare_undeclared_name():
    # names no template declares keep the malformed-key error (the
    # resolve_cli_overrides gate never lets them through in the first place)
    with pytest.raises(RuntimeError, match='Malformed'):
        resolve([('cruse_speed', '1.0')])


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


def test_resolve_cli_overrides_accepts_bare_parameter_names():
    pargs = make_collection()
    context = FakeContext(OrderedDict([
        ('vehicle_ids', '[veh1, veh2]'),  # ordinary argument: ignored
        ('use_sim_time', 'False'),        # ordinary argument: ignored
        ('cruise_speed', '4.5'),          # declared in a template: override
    ]))
    pargs._snapshot_cmd_args(context)
    overrides = pargs.resolve_cli_overrides(context, VEHICLES)
    assert overrides.for_node('/veh1/planner') == {'cruise_speed': 4.5}
    assert overrides.for_node('/veh2/controller') == {'cruise_speed': 4.5}
    assert overrides.for_node('/veh1/sensor') == {}


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
    assert convert_cli_value('9.0', 'float', 'a') == 9.0
    assert convert_cli_value('9', 'float', 'a') == 9.0
    assert convert_cli_value('5', 'int', 'a') == 5
    assert convert_cli_value('true', 'bool', 'a') is True
    # string parameters take the raw text verbatim, even numeric-looking ones
    assert convert_cli_value('5', 'string', 'a') == '5'


def test_convert_arrays():
    assert convert_cli_value('[1.0, 2]', 'double_array', 'a') == [1.0, 2.0]
    assert convert_cli_value('[1.0, 2]', 'float_array', 'a') == [1.0, 2.0]
    assert convert_cli_value('[a, b]', 'string_array', 'a') == ['a', 'b']


def test_convert_type_mismatch():
    with pytest.raises(ValueError):
        convert_cli_value('not_a_number', 'double', 'a')
    with pytest.raises(ValueError):
        convert_cli_value('5', 'bool', 'a')
    with pytest.raises(ValueError):
        convert_cli_value('1.0', 'double_array', 'a')
    with pytest.raises(ValueError):
        convert_cli_value('true', 'float', 'a')
    with pytest.raises(ValueError):
        convert_cli_value('not_a_number', 'float', 'a')
    with pytest.raises(ValueError):
        convert_cli_value('[1.0, nope]', 'float_array', 'a')
    with pytest.raises(ValueError):
        convert_cli_value('1.0', 'float_array', 'a')


def test_convert_typed_values():
    assert convert_typed_value(9, 'double', 'a') == 9.0
    assert convert_typed_value(9, 'float', 'a') == 9.0
    assert convert_typed_value([1, 2.5], 'float_array', 'a') == [1.0, 2.5]
    assert convert_typed_value([1, 2], 'int_array', 'a') == [1, 2]
    with pytest.raises(ValueError):
        convert_typed_value('nope', 'double', 'a')


def test_convert_string_expands_pkg_path(monkeypatch):
    monkeypatch.setattr(
        'avt_341_param_lib.runtime.launch_params.get_package_share_directory',
        lambda pkg: f'/opt/share/{pkg}')
    assert convert_cli_value(
        '$pkg_path{demo}/maps/a.csv', 'string', 'a') == '/opt/share/demo/maps/a.csv'
    assert convert_typed_value('$pkg_path{demo}/m.csv', 'string', 'a') == '/opt/share/demo/m.csv'
    assert convert_typed_value(
        ['$pkg_path{p1}/a', 'plain'], 'string_array', 'a') == ['/opt/share/p1/a', 'plain']


def test_convert_pkg_path_errors(monkeypatch):
    def raise_not_found(pkg):
        raise PackageNotFoundError(pkg)
    monkeypatch.setattr(
        'avt_341_param_lib.runtime.launch_params.get_package_share_directory', raise_not_found)
    with pytest.raises(ValueError, match=r"\$pkg_path\{nope\} in parameter override 'a'"):
        convert_cli_value('$pkg_path{nope}/f.csv', 'string', 'a')
    with pytest.raises(ValueError, match='no package name'):
        convert_cli_value('$pkg_path{}/f.csv', 'string', 'a')
    with pytest.raises(ValueError, match='unterminated'):
        convert_cli_value('$pkg_path{nope/f.csv', 'string', 'a')
