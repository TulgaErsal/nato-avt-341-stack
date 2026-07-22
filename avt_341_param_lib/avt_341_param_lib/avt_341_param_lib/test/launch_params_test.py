import os

import pytest

from avt_341_param_lib.launch_params import (
    ParameterCollection,
    convert_cli_value,
    load_template_metadata,
)

TEST_DIR = os.path.dirname(os.path.abspath(__file__))


def test_metadata_with_param_namespace():
    partition, specs = load_template_metadata(os.path.join(TEST_DIR, 'launch_template_a.yaml'))
    assert partition == 'alpha'
    assert set(specs.keys()) == {'cruise_speed', 'planner.mode', 'ids'}
    assert specs['cruise_speed'].param_type == 'double'
    assert specs['cruise_speed'].default_value == 1.5
    assert specs['planner.mode'].param_type == 'string'
    assert specs['planner.mode'].default_value == 'grid'
    assert specs['ids'].param_type == 'int_array'
    assert specs['ids'].default_value == [1, 2]


def test_metadata_stem_fallback():
    partition, specs = load_template_metadata(os.path.join(TEST_DIR, 'launch_template_b.yaml'))
    assert partition == 'launch_template_b'
    assert set(specs.keys()) == {'enabled'}
    assert specs['enabled'].param_type == 'bool'


def test_metadata_rejects_non_template_file(tmp_path):
    bad = tmp_path / 'not_a_template.yaml'
    bad.write_text('just_a_key: 5\n')
    with pytest.raises(ValueError):
        load_template_metadata(str(bad))


def test_parameter_collection_from_template_files():
    pargs = ParameterCollection.from_template_files([
        os.path.join(TEST_DIR, 'launch_template_a.yaml'),
        os.path.join(TEST_DIR, 'launch_template_b.yaml'),
    ])
    assert set(pargs.partitions) == {'alpha', 'launch_template_b'}
    assert 'alpha/planner.mode' in pargs.argument_names()
    assert 'launch_template_b/enabled' in pargs.argument_names()


def test_parameter_collection_from_template_folder(tmp_path):
    for source in ('launch_template_a.yaml', 'launch_template_b.yaml'):
        with open(os.path.join(TEST_DIR, source)) as f:
            (tmp_path / source).write_text(f.read())
    (tmp_path / 'notes.txt').write_text('not a yaml file\n')
    pargs = ParameterCollection.from_template_folder(str(tmp_path))
    assert set(pargs.partitions) == {'alpha', 'launch_template_b'}
    with pytest.raises(ValueError):
        ParameterCollection.from_template_folder(str(tmp_path / 'does_not_exist'))


def test_duplicate_partition_rejected():
    path = os.path.join(TEST_DIR, 'launch_template_a.yaml')
    with pytest.raises(ValueError):
        ParameterCollection.from_template_files([path, path])


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
