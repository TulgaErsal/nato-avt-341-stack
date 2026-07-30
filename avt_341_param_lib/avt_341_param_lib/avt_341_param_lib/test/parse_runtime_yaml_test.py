import os
import re

import pytest
import yaml
from ament_index_python.packages import PackageNotFoundError

from avt_341_param_lib.runtime.parse_runtime_yaml import (
    _patterns_intersect,
    resolve_params_files,
)

VEHICLES = ['veh1', 'veh2']


def patch_share_dirs(monkeypatch):
    monkeypatch.setattr(
        'avt_341_param_lib.runtime.launch_params.get_package_share_directory',
        lambda pkg: f'/opt/share/{pkg}')


def write(tmp_path, name, text):
    path = tmp_path / name
    path.write_text(text)
    return str(path)


def resolved_doc(paths, vehicles=VEHICLES, work_dir=None, node_fqns=None):
    resolved = resolve_params_files(paths, vehicles, work_dir, node_fqns=node_fqns)
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


def test_pkg_path_expands_in_place(tmp_path, monkeypatch):
    patch_share_dirs(monkeypatch)
    path = write(tmp_path, 'a.yaml', (
        '/**:\n'
        '  ros__parameters:\n'
        '    whole: $pkg_path{avt_341}\n'
        '    joined: $pkg_path{avt_341}/config/zones.csv\n'
        '    doubled: $pkg_path{a}:$pkg_path{b}\n'
    ))
    resolved = resolve_params_files([path], VEHICLES)
    assert resolved[0] != path  # $pkg_path{} alone marks the file templated
    with open(resolved[0]) as f:
        params = yaml.safe_load(f)['/**']['ros__parameters']
    assert params['whole'] == '/opt/share/avt_341'
    assert params['joined'] == '/opt/share/avt_341/config/zones.csv'
    assert params['doubled'] == '/opt/share/a:/opt/share/b'


def test_pkg_path_unknown_package_raises(tmp_path, monkeypatch):
    def raise_not_found(pkg):
        raise PackageNotFoundError(pkg)
    monkeypatch.setattr(
        'avt_341_param_lib.runtime.launch_params.get_package_share_directory', raise_not_found)
    path = write(tmp_path, 'a.yaml', '/**:\n  ros__parameters:\n    x: $pkg_path{nope}/f.csv\n')
    with pytest.raises(RuntimeError, match=r'\$pkg_path\{nope\}'):
        resolve_params_files([path], VEHICLES)


def test_pkg_path_malformed_raises(tmp_path, monkeypatch):
    patch_share_dirs(monkeypatch)
    empty = write(tmp_path, 'a.yaml', '/**:\n  ros__parameters:\n    x: $pkg_path{}/f.csv\n')
    with pytest.raises(RuntimeError, match='no package name'):
        resolve_params_files([empty], VEHICLES)
    unterminated = write(
        tmp_path, 'b.yaml', "/**:\n  ros__parameters:\n    x: '$pkg_path{avt_341/f.csv'\n")
    with pytest.raises(RuntimeError, match='unterminated'):
        resolve_params_files([unterminated], VEHICLES)


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


def test_ref_to_pkg_path_value(tmp_path, monkeypatch):
    patch_share_dirs(monkeypatch)
    path = write(tmp_path, 'a.yaml', (
        '/veh1/planner:\n'
        '  ros__parameters:\n'
        '    map_file: $pkg_path{maps}/grid.csv\n'
        '/**:\n'
        '  ros__parameters:\n'
        '    copied: $ref{veh1/planner/map_file}\n'
    ))
    doc = resolved_doc([path])
    assert doc['/veh1/planner']['ros__parameters']['map_file'] == '/opt/share/maps/grid.csv'
    assert doc['/**']['ros__parameters']['copied'] == '/opt/share/maps/grid.csv'


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


def test_resolved_output_is_valid_params_yaml_without_residue(tmp_path, monkeypatch):
    patch_share_dirs(monkeypatch)
    work_dir = tmp_path / 'out'
    work_dir.mkdir()
    path = write(tmp_path, 'a.yaml', (
        '/**:\n'
        '  ros__parameters:\n'
        "    data_file: $python{os.path.join('base', 'file.csv')}\n"
        '    zones_file: $pkg_path{avt_341}/config/zones.csv\n'
        '    plain: kept\n'
    ))
    resolved = resolve_params_files([path], VEHICLES, str(work_dir))
    assert resolved[0] != path
    assert os.path.basename(resolved[0]) == '00_a.yaml'
    text = open(resolved[0]).read()
    assert '$python{' not in text and '$ref{' not in text and '$pkg_path{' not in text
    doc = yaml.safe_load(text)
    assert doc['/**']['ros__parameters']['plain'] == 'kept'


# --- regex section-key expansion -------------------------------------------

NODE_FQNS = ['/veh1/planner', '/veh1/controller', '/veh2/planner', '/veh2/controller']


def test_regex_section_expands_to_concrete_keys(tmp_path):
    path = write(tmp_path, 'a.yaml', (
        '/**:\n'
        '  ros__parameters:\n'
        '    shared: 1\n'
        '/(veh[12])/planner:\n'
        '  ros__parameters:\n'
        '    cruise_speed: 2.5\n'
    ))
    doc = resolved_doc([path], node_fqns=NODE_FQNS)
    # untouched sections keep their position; the expansions replace the
    # regex section in place, preserving document order
    assert list(doc) == ['/**', '/veh1/planner', '/veh2/planner']
    for key in ('/veh1/planner', '/veh2/planner'):
        assert doc[key]['ros__parameters'] == {'cruise_speed': 2.5}


def test_regex_file_without_value_templates_is_still_rewritten(tmp_path):
    path = write(tmp_path, 'a.yaml', (
        '/(veh[12])/planner:\n  ros__parameters:\n    cruise_speed: 2.5\n'))
    resolved = resolve_params_files([path], VEHICLES, node_fqns=NODE_FQNS)
    assert resolved[0] != path  # regex keys alone mark the file templated


def test_regex_nested_namespace_component(tmp_path):
    path = write(tmp_path, 'a.yaml', (
        '(veh[12]):\n'
        '  planner:\n'
        '    ros__parameters:\n'
        '      cruise_speed: 4.5\n'
    ))
    doc = resolved_doc([path], node_fqns=NODE_FQNS)
    assert list(doc) == ['/veh1/planner', '/veh2/planner']
    assert doc['/veh1/planner']['ros__parameters'] == {'cruise_speed': 4.5}


def test_regex_mixed_subtree_keeps_literal_branches(tmp_path):
    path = write(tmp_path, 'a.yaml', (
        'veh1:\n'
        '  planner:\n'
        '    ros__parameters:\n'
        '      a: 1\n'
        '  (controller|sensor):\n'
        '    ros__parameters:\n'
        '      b: 2\n'
    ))
    doc = resolved_doc([path], node_fqns=NODE_FQNS)
    # the all-literal branch stays nested under its original keys (rcl matches
    # it natively); only the regex branch is hoisted to concrete flat keys
    assert list(doc) == ['veh1', '/veh1/controller']
    assert doc['veh1'] == {'planner': {'ros__parameters': {'a': 1}}}
    assert doc['/veh1/controller']['ros__parameters'] == {'b': 2}


def test_regex_relative_key_gets_implicit_any_namespace_prefix(tmp_path):
    path = write(tmp_path, 'a.yaml', (
        '(planner|controller):\n  ros__parameters:\n    x: 1\n'))
    doc = resolved_doc([path], node_fqns=NODE_FQNS)
    assert list(doc) == NODE_FQNS


def test_regex_composes_with_glob_wildcards(tmp_path):
    path = write(tmp_path, 'a.yaml', (
        '/**/(planner_[0-9]+):\n  ros__parameters:\n    x: 1\n'))
    doc = resolved_doc([path], node_fqns=['/veh1/planner_1', '/veh1/controller'])
    assert list(doc) == ['/veh1/planner_1']


def test_regex_expansion_bodies_resolve_independently(tmp_path):
    path = write(tmp_path, 'a.yaml', (
        '/(veh[12])/planner:\n'
        '  ros__parameters:\n'
        '    computed: $python{2 + 3}\n'
    ))
    resolved = resolve_params_files([path], VEHICLES, node_fqns=NODE_FQNS)
    text = open(resolved[0]).read()
    assert '&id' not in text  # bodies are deep copies: no yaml anchors/aliases
    doc = yaml.safe_load(text)
    assert doc['/veh1/planner']['ros__parameters']['computed'] == 5
    assert doc['/veh2/planner']['ros__parameters']['computed'] == 5


def test_regex_expansion_merges_with_existing_section(tmp_path):
    path = write(tmp_path, 'a.yaml', (
        '/veh1/planner:\n'
        '  ros__parameters:\n'
        '    a: 1\n'
        '    b: 1\n'
        '/(veh[1])/planner:\n'
        '  ros__parameters:\n'
        '    b: 2\n'
    ))
    doc = resolved_doc([path], node_fqns=NODE_FQNS)
    # sections landing on one key merge with later-wins per parameter
    assert doc['/veh1/planner']['ros__parameters'] == {'a': 1, 'b': 2}


def test_regex_zero_match_warns_and_drops(tmp_path):
    path = write(tmp_path, 'a.yaml', (
        '/**:\n  ros__parameters:\n    kept: 1\n'
        '/(veh[45])/planner:\n  ros__parameters:\n    x: 1\n'
    ))
    with pytest.warns(UserWarning, match='matches none'):
        doc = resolved_doc([path], node_fqns=NODE_FQNS)
    assert list(doc) == ['/**']


def test_regex_empty_node_fqns_warns_and_drops(tmp_path):
    path = write(tmp_path, 'a.yaml', (
        '/(veh[12])/planner:\n  ros__parameters:\n    x: 1\n'))
    with pytest.warns(UserWarning, match='matches none'):
        doc = resolved_doc([path], node_fqns=[])
    assert doc == {}


def test_regex_without_node_fqns_raises(tmp_path):
    path = write(tmp_path, 'a.yaml', (
        '/(veh[12])/planner:\n  ros__parameters:\n    x: 1\n'))
    with pytest.raises(RuntimeError, match='node_fqns'):
        resolve_params_files([path], VEHICLES)


def test_ref_resolves_against_expanded_regex_section(tmp_path):
    # regex keys are expanded before value resolution, so $ref{} lookups only
    # ever see concrete or glob section keys
    path = write(tmp_path, 'a.yaml', (
        '/(veh[12])/planner:\n'
        '  ros__parameters:\n'
        '    cruise_speed: 3.3\n'
        '/**:\n'
        '  ros__parameters:\n'
        '    copied: $ref{veh1/planner/cruise_speed}\n'
    ))
    doc = resolved_doc([path], node_fqns=NODE_FQNS)
    assert doc['/**']['ros__parameters']['copied'] == 3.3


def test_ref_rejects_regex_selector_tokens(tmp_path):
    path = write(tmp_path, 'a.yaml', (
        '/veh1/planner:\n'
        '  ros__parameters:\n'
        '    cruise_speed: 3.3\n'
        '/**:\n'
        '  ros__parameters:\n'
        '    x: $ref{(veh[12])/planner/cruise_speed}\n'
    ))
    with pytest.raises(RuntimeError, match='not supported in'):
        resolve_params_files([path], VEHICLES, node_fqns=NODE_FQNS)


def test_invalid_regex_section_key_raises(tmp_path):
    bad = write(tmp_path, 'bad.yaml', (
        '/(veh[12)/planner:\n  ros__parameters:\n    x: 1\n'))
    with pytest.raises(RuntimeError, match='bad.yaml'):
        resolve_params_files([bad], VEHICLES, node_fqns=NODE_FQNS)
    unbalanced = write(tmp_path, 'unbalanced.yaml', (
        '/(veh[12]/planner:\n  ros__parameters:\n    x: 1\n'))
    with pytest.raises(RuntimeError, match='Unbalanced'):
        resolve_params_files([unbalanced], VEHICLES, node_fqns=NODE_FQNS)


def test_patterns_intersect():
    assert _patterns_intersect(['**'], ['veh1', 'planner'])
    assert _patterns_intersect(['**', 'planner'], ['veh1', 'planner'])
    assert _patterns_intersect(['*', 'planner'], ['**', 'planner'])
    assert _patterns_intersect(['**'], [])
    assert not _patterns_intersect(['veh1', 'planner'], ['veh2', 'planner'])
    assert not _patterns_intersect(['*'], [])
    assert not _patterns_intersect(['veh1'], ['veh1', 'planner'])


# --- __overrides file ordering ----------------------------------------------


def load_doc(path):
    with open(path) as f:
        return yaml.safe_load(f)


def names(paths):
    """Basenames of resolved paths, with the output index prefix removed."""
    return [re.sub(r'^\d\d_', '', os.path.basename(path)) for path in paths]


def section(**params):
    """A minimal single-section params file body."""
    body = ''.join(f'    {name}: {value}\n' for name, value in params.items())
    return '/**:\n  ros__parameters:\n' + body


def test_overrides_loads_unlisted_target_before_the_declaring_file(tmp_path):
    base = write(tmp_path, 'base.yaml', section(max_speed=1.0))
    top = write(tmp_path, 'top.yaml', '__overrides: base.yaml\n' + section(max_speed=2.0))
    resolved = resolve_params_files([top], VEHICLES)
    # the target joins the corpus ahead of its declarer; it holds no templating
    # of its own, so it passes through untouched
    assert len(resolved) == 2
    assert resolved[0] == base
    assert resolved[1] != top  # rewritten to drop the directive
    assert load_doc(resolved[1])['/**']['ros__parameters']['max_speed'] == 2.0


def test_overrides_reorders_an_already_listed_target(tmp_path):
    base = write(tmp_path, 'base.yaml', section(max_speed=1.0))
    top = write(tmp_path, 'top.yaml', '__overrides: base.yaml\n' + section(max_speed=2.0))
    # listed in the order that would let the base win: the target is moved, not
    # copied -- a second copy would simply re-apply after the declarer
    resolved = resolve_params_files([top, base], VEHICLES)
    assert names(resolved) == ['base.yaml', 'top.yaml']
    assert resolved[0] == base


def test_overrides_leaves_an_already_correct_order_unchanged(tmp_path):
    base = write(tmp_path, 'base.yaml', section(max_speed=1.0))
    top = write(tmp_path, 'top.yaml', '__overrides: base.yaml\n' + section(max_speed=2.0))
    resolved = resolve_params_files([base, top], VEHICLES)
    assert names(resolved) == ['base.yaml', 'top.yaml']


def test_overrides_preserves_position_of_unconstrained_files(tmp_path):
    base = write(tmp_path, 'base.yaml', section(a=1))
    top = write(tmp_path, 'top.yaml', '__overrides: base.yaml\n' + section(a=2))
    other = write(tmp_path, 'other.yaml', section(b=3))
    resolved = resolve_params_files([top, other, base], VEHICLES)
    # only the constrained pair reorders; other keeps its list position
    assert names(resolved) == ['base.yaml', 'top.yaml', 'other.yaml']


def test_overrides_transitive_chain(tmp_path):
    write(tmp_path, 'c.yaml', section(x=1))
    write(tmp_path, 'b.yaml', '__overrides: c.yaml\n' + section(x=2))
    a = write(tmp_path, 'a.yaml', '__overrides: b.yaml\n' + section(x=3))
    resolved = resolve_params_files([a], VEHICLES)
    assert names(resolved) == ['c.yaml', 'b.yaml', 'a.yaml']


def test_overrides_diamond_loads_the_shared_base_once(tmp_path):
    write(tmp_path, 'd.yaml', section(x=0))
    write(tmp_path, 'b.yaml', '__overrides: d.yaml\n' + section(x=1))
    write(tmp_path, 'c.yaml', '__overrides: d.yaml\n' + section(x=2))
    a = write(tmp_path, 'a.yaml', '__overrides: [b.yaml, c.yaml]\n' + section(x=3))
    resolved = resolve_params_files([a], VEHICLES)
    assert names(resolved) == ['d.yaml', 'b.yaml', 'c.yaml', 'a.yaml']


def test_overrides_list_orders_later_entries_last(tmp_path):
    write(tmp_path, 'b.yaml', section(x=1))
    write(tmp_path, 'c.yaml', section(x=2))
    a = write(tmp_path, 'a.yaml', '__overrides:\n  - b.yaml\n  - c.yaml\n' + section(x=3))
    resolved = resolve_params_files([a], VEHICLES)
    # within a list, later entries win over earlier ones, as in the caller's list
    assert names(resolved) == ['b.yaml', 'c.yaml', 'a.yaml']


def test_duplicate_input_paths_are_collapsed(tmp_path):
    plain = write(tmp_path, 'plain.yaml', section(max_speed=4.0))
    assert resolve_params_files([plain, plain], VEHICLES) == [plain]


def test_overrides_key_is_stripped_from_the_written_file(tmp_path):
    write(tmp_path, 'base.yaml', section(a=1))
    top = write(tmp_path, 'top.yaml', '__overrides: base.yaml\n' + section(a=2))
    resolved = resolve_params_files([top], VEHICLES)
    text = open(resolved[1]).read()
    assert '__overrides' not in text
    assert list(load_doc(resolved[1])) == ['/**']


def test_overrides_forces_rewrite_of_an_untemplated_file(tmp_path):
    write(tmp_path, 'base.yaml', section(a=1))
    top = write(tmp_path, 'top.yaml', '__overrides: base.yaml\n' + section(a=2))
    # no $ markers anywhere: the directive alone forces the rewrite, because the
    # key must not reach rcl
    resolved = resolve_params_files([top], VEHICLES)
    assert resolved[1] != top


def test_overrides_only_file_yields_an_empty_document(tmp_path):
    write(tmp_path, 'base.yaml', section(a=1))
    top = write(tmp_path, 'top.yaml', '__overrides: base.yaml\n')
    resolved = resolve_params_files([top], VEHICLES)
    assert load_doc(resolved[1]) == {}


def test_overrides_output_names_stay_unique_for_equal_basenames(tmp_path):
    (tmp_path / 'x').mkdir()
    (tmp_path / 'y').mkdir()
    write(tmp_path / 'y', 'p.yaml', '/**:\n  ros__parameters:\n    x: $python{2 + 3}\n')
    top = write(tmp_path / 'x', 'p.yaml', '__overrides: ../y/p.yaml\n' + section(a=1))
    resolved = resolve_params_files([top], VEHICLES)
    assert [os.path.basename(path) for path in resolved] == ['00_p.yaml', '01_p.yaml']


def test_overrides_file_with_anchors_emits_no_aliases(tmp_path):
    write(tmp_path, 'base.yaml', section(a=1))
    top = write(tmp_path, 'top.yaml', (
        '__overrides: base.yaml\n'
        '/veh1/planner: &body\n'
        '  ros__parameters:\n'
        '    a: 2\n'
        '/veh1/controller: *body\n'
    ))
    resolved = resolve_params_files([top], VEHICLES)
    text = open(resolved[1]).read()
    # rcl rejects aliases, so the shared body is repeated rather than anchored
    assert '&id' not in text and '*id' not in text
    doc = yaml.safe_load(text)
    assert doc['/veh1/planner']['ros__parameters'] == {'a': 2}
    assert doc['/veh1/controller']['ros__parameters'] == {'a': 2}


def test_overrides_path_relative_to_the_declaring_file(tmp_path):
    base = write(tmp_path, 'base.yaml', section(a=1))
    (tmp_path / 'sub').mkdir()
    top = write(tmp_path / 'sub', 'top.yaml', '__overrides: ../base.yaml\n' + section(a=2))
    resolved = resolve_params_files([top], VEHICLES)
    assert resolved[0] == base


def test_overrides_path_accepts_absolute(tmp_path):
    base = write(tmp_path, 'base.yaml', section(a=1))
    top = write(tmp_path, 'top.yaml', f'__overrides: {base}\n' + section(a=2))
    resolved = resolve_params_files([top], VEHICLES)
    assert resolved[0] == base


def test_overrides_path_accepts_pkg_path(tmp_path, monkeypatch):
    # the shared patch_share_dirs() returns a path that does not exist, which
    # would fail the target existence check
    monkeypatch.setattr(
        'avt_341_param_lib.runtime.launch_params.get_package_share_directory',
        lambda pkg: str(tmp_path))
    base = write(tmp_path, 'base.yaml', section(a=1))
    top = write(tmp_path, 'top.yaml', (
        '__overrides: $pkg_path{avt_341}/base.yaml\n' + section(a=2)))
    resolved = resolve_params_files([top], VEHICLES)
    assert resolved[0] == os.path.normpath(base)
    assert load_doc(resolved[1])['/**']['ros__parameters']['a'] == 2


def test_overrides_unknown_package_raises(tmp_path):
    # expand_pkg_path raises PackageNotFoundError for a package that is not
    # installed; it surfaces as a RuntimeError naming the declaring file
    top = write(tmp_path, 'top.yaml', '__overrides: $pkg_path{no_such_pkg}/base.yaml\n')
    with pytest.raises(RuntimeError, match='top.yaml'):
        resolve_params_files([top], VEHICLES)


def test_overrides_matches_a_listed_target_spelled_differently(tmp_path):
    base = write(tmp_path, 'base.yaml', section(a=1))
    top = write(tmp_path, 'top.yaml', '__overrides: base.yaml\n' + section(a=2))
    # forward slashes on Windows name the same file as the backslash spelling
    # the directive resolves to; the caller's spelling is what is handed back
    listed = base.replace('\\', '/')
    resolved = resolve_params_files([top, listed], VEHICLES)
    assert names(resolved) == ['base.yaml', 'top.yaml']
    assert resolved[0] == listed


def test_overrides_cycle_raises(tmp_path):
    write(tmp_path, 'b.yaml', '__overrides: a.yaml\n' + section(x=1))
    a = write(tmp_path, 'a.yaml', '__overrides: b.yaml\n' + section(x=2))
    with pytest.raises(RuntimeError, match='Circular __overrides'):
        resolve_params_files([a], VEHICLES)


def test_overrides_self_reference_raises(tmp_path):
    a = write(tmp_path, 'a.yaml', '__overrides: a.yaml\n' + section(x=1))
    with pytest.raises(RuntimeError, match='Circular __overrides'):
        resolve_params_files([a], VEHICLES)


def test_overrides_missing_target_raises(tmp_path):
    top = write(tmp_path, 'top.yaml', '__overrides: nope.yaml\n' + section(a=1))
    with pytest.raises(RuntimeError, match='not found at'):
        resolve_params_files([top], VEHICLES)


@pytest.mark.parametrize('value', ['3', '', '{a: b}', '[base.yaml, 3]', "''", '[[base.yaml]]'])
def test_overrides_invalid_value_raises(tmp_path, value):
    write(tmp_path, 'base.yaml', section(a=1))
    top = write(tmp_path, 'top.yaml', f'__overrides: {value}\n' + section(a=2))
    with pytest.raises(RuntimeError, match='Invalid __overrides'):
        resolve_params_files([top], VEHICLES)


@pytest.mark.parametrize('value', ['$ref{**/max_speed}', '$python{"base.yaml"}'])
def test_overrides_rejects_full_value_templates(tmp_path, value):
    top = write(tmp_path, 'top.yaml', f"__overrides: '{value}'\n" + section(a=1))
    with pytest.raises(RuntimeError, match='not supported in an __overrides path'):
        resolve_params_files([top], VEHICLES)


def test_nested_overrides_key_raises(tmp_path):
    write(tmp_path, 'base.yaml', section(a=1))
    under_namespace = write(tmp_path, 'a.yaml', (
        'veh1:\n'
        '  __overrides: base.yaml\n'
        '  planner:\n'
        '    ros__parameters:\n'
        '      a: 2\n'
    ))
    with pytest.raises(RuntimeError, match='only allowed as a top-level key'):
        resolve_params_files([under_namespace], VEHICLES)
    # the check runs before the ros__parameters stop, so a directive sitting
    # beside a section body is caught too
    beside_body = write(tmp_path, 'b.yaml', (
        '/veh1/planner:\n'
        '  __overrides: base.yaml\n'
        '  ros__parameters:\n'
        '    a: 2\n'
    ))
    with pytest.raises(RuntimeError, match='only allowed as a top-level key'):
        resolve_params_files([beside_body], VEHICLES)


def test_overrides_target_that_is_not_a_mapping_raises(tmp_path):
    write(tmp_path, 'base.yaml', '- a\n- b\n')
    top = write(tmp_path, 'top.yaml', '__overrides: base.yaml\n' + section(a=1))
    with pytest.raises(RuntimeError, match='is not a parameter yaml file'):
        resolve_params_files([top], VEHICLES)


def test_overrides_unparseable_target_names_the_target(tmp_path):
    write(tmp_path, 'broken.yaml', '/**:\n  ros__parameters:\n    a: [1, 2\n')
    top = write(tmp_path, 'top.yaml', '__overrides: broken.yaml\n' + section(a=1))
    # the target is a file the caller never listed, so it has to name itself
    with pytest.raises(RuntimeError, match='broken.yaml'):
        resolve_params_files([top], VEHICLES)


def test_overrides_writes_nothing_when_a_target_fails_to_load(tmp_path):
    work_dir = tmp_path / 'out'
    work_dir.mkdir()
    write(tmp_path, 'broken.yaml', '/**:\n  ros__parameters:\n    a: [1, 2\n')
    top = write(tmp_path, 'top.yaml', '__overrides: broken.yaml\n' + section(a=1))
    with pytest.raises(RuntimeError):
        resolve_params_files([top], VEHICLES, str(work_dir))
    # a document whose directive was stripped never reaches disk unless every
    # transitive target loaded: that is what makes the early strip safe
    assert os.listdir(str(work_dir)) == []


def test_overrides_target_templates_are_resolved(tmp_path):
    write(tmp_path, 'base.yaml', '/**:\n  ros__parameters:\n    computed: $python{2 + 3}\n')
    top = write(tmp_path, 'top.yaml', '__overrides: base.yaml\n' + section(a=1))
    resolved = resolve_params_files([top], VEHICLES)
    # a file reached only via __overrides is a full corpus member
    assert resolved[0] != os.path.join(str(tmp_path), 'base.yaml')
    assert load_doc(resolved[0])['/**']['ros__parameters']['computed'] == 5


def test_overrides_composes_with_regex_sections(tmp_path):
    write(tmp_path, 'base.yaml', section(a=1))
    top = write(tmp_path, 'top.yaml', (
        '__overrides: base.yaml\n'
        '/(veh[12])/planner:\n'
        '  ros__parameters:\n'
        '    cruise_speed: 2.5\n'
    ))
    resolved = resolve_params_files([top], VEHICLES, node_fqns=NODE_FQNS)
    doc = load_doc(resolved[1])
    assert list(doc) == ['/veh1/planner', '/veh2/planner']
    assert doc['/veh1/planner']['ros__parameters'] == {'cruise_speed': 2.5}


def test_overrides_target_with_regex_sections_requires_node_fqns(tmp_path):
    write(tmp_path, 'base.yaml', '/(veh[12])/planner:\n  ros__parameters:\n    x: 1\n')
    top = write(tmp_path, 'top.yaml', '__overrides: base.yaml\n' + section(a=1))
    with pytest.raises(RuntimeError, match='base.yaml'):
        resolve_params_files([top], VEHICLES)


def test_overrides_changes_ref_first_match_order(tmp_path):
    b = write(tmp_path, 'b.yaml', section(max_speed=1.0))
    a = write(tmp_path, 'a.yaml', '__overrides: b.yaml\n' + section(max_speed=2.0))
    c = write(tmp_path, 'c.yaml', (
        '/**:\n  ros__parameters:\n    copied: $ref{**/max_speed}\n'))
    resolved = resolve_params_files([a, b, c], VEHICLES)
    # $ref{} first-match follows the effective order: b was hoisted ahead of a,
    # so its value is the one found (a's 2.0 would win on the listed order)
    assert names(resolved) == ['b.yaml', 'a.yaml', 'c.yaml']
    assert load_doc(resolved[2])['/**']['ros__parameters']['copied'] == 1.0
