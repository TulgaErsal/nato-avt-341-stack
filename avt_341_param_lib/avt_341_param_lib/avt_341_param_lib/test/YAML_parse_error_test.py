# Copyright 2022 PickNik, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pytest
from unittest.mock import patch
import sys
import os
import tempfile

from avt_341_param_lib.codegen.generate_cpp_header import run as run_cpp
from avt_341_param_lib.codegen.generate_python_module import run as run_python
from avt_341_param_lib.codegen.generate_markdown import run as run_md
from avt_341_param_lib.common.template_yaml import YAMLSyntaxError
from avt_341_param_lib.codegen.generate_cpp_header import parse_args

# Test yaml files live next to this file, both in the source tree and when
# installed into site-packages
TEST_DIR = os.path.dirname(os.path.abspath(__file__))


def set_up(yaml_test_file):
    full_file_path = os.path.join(TEST_DIR, yaml_test_file)
    output_dir = tempfile.mkdtemp()
    dto_output_file = os.path.join(output_dir, yaml_test_file + '_params_dto.hpp')
    service_output_file = os.path.join(
        output_dir, yaml_test_file + '_params_service.hpp')
    testargs = [
        sys.argv[0],
        dto_output_file,
        service_output_file,
        full_file_path,
    ]

    with patch.object(sys, 'argv', testargs):
        args = parse_args()
        dto_output_file = args.output_cpp_dto_header_file
        service_output_file = args.output_cpp_service_header_file
        yaml_file = args.input_yaml_file
        validate_header = args.validate_header
        run_cpp(
            dto_output_file,
            service_output_file,
            yaml_file,
            validate_header,
        )

    run_python(
        os.path.join(output_dir, yaml_test_file + '.py'),
        full_file_path,
        '',
    )
    run_md(
        full_file_path,
        os.path.join(output_dir, yaml_test_file + '.md'),
        'markdown',
    )
    run_md(
        full_file_path,
        os.path.join(output_dir, yaml_test_file + '.rst'),
        'rst',
    )


# class TestViewValidCodeGen(unittest.TestCase):
@pytest.mark.parametrize(
    'test_input,expected',
    [
        (file_name, YAMLSyntaxError)
        for file_name in [
            'wrong_default_type.yaml',
            'missing_type.yaml',
            'invalid_syntax.yaml',
            'invalid_parameter_type.yaml',
        ]
    ],
)
def test_expected(test_input, expected):
    with pytest.raises(expected) as e:
        yaml_test_file = test_input
        set_up(yaml_test_file)
    print(e.value)


def test_parse_valid_parameter_file():
    try:
        yaml_test_file = 'valid_parameters.yaml'
        set_up(yaml_test_file)
    except Exception as e:
        assert False, f'failed to parse valid file, reason:{e}'


def test_parse_valid_parameter_file_including_none_type():
    try:
        yaml_test_file = 'valid_parameters_with_none_type.yaml'
        set_up(yaml_test_file)
    except Exception as e:
        assert False, f'failed to parse valid file, reason:{e}'
