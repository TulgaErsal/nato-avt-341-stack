from pathlib import Path

import pytest

from avt_341_param_lib.codegen.generate_cpp_header import run as run_cpp
from avt_341_param_lib.codegen.generate_markdown import run as run_documentation
from avt_341_param_lib.codegen.generate_python_module import run as run_python
from avt_341_param_lib.common.template_yaml import YAMLSyntaxError

TEST_DIR = Path(__file__).resolve().parent
FLOAT_TEMPLATE = TEST_DIR / 'float_parameters.yaml'


def generate_cpp(tmp_path):
    dto_output = tmp_path / 'float_params_dto.hpp'
    service_output = tmp_path / 'float_params_service.hpp'
    run_cpp(str(dto_output), str(service_output), str(FLOAT_TEMPLATE))
    return (
        dto_output.read_text(encoding='utf-8'),
        service_output.read_text(encoding='utf-8'),
    )


def test_cpp_float_types_use_ros_double_transport(tmp_path):
    dto, service = generate_cpp(tmp_path)

    assert 'float scalar = 1.25F;' in dto
    assert 'std::vector<float> array = {1.5F, -2.25F};' in dto
    assert 'float required_scalar;' in dto
    assert 'std::vector<float> required_array;' in dto
    assert 'std::numeric_limits<float>::quiet_NaN()' in dto
    assert 'std::numeric_limits<float>::infinity()' in dto

    assert 'rclcpp::ParameterType::PARAMETER_DOUBLE' in service
    assert 'rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY' in service
    assert 'PARAMETER_FLOAT' not in service

    assert 'static_cast<float>(param.as_double())' in service
    assert (
        'std::vector<float>(param.as_double_array().begin(), '
        'param.as_double_array().end())'
    ) in service


def test_cpp_existing_double_generation_is_unchanged(tmp_path):
    dto, service = generate_cpp(tmp_path)

    assert 'double existing_double = 2.5;' in dto
    assert 'updated_params.existing_double = param.as_double();' in service


def test_cpp_float_conversions_cover_regular_and_mapped_updates(tmp_path):
    _, service = generate_cpp(tmp_path)

    # Initial reads and update callbacks use the same conversion expression.
    assert service.count('static_cast<float>(param.as_double())') >= 6
    assert service.count(
        'std::vector<float>(param.as_double_array().begin(), '
        'param.as_double_array().end())') >= 4


def test_cpp_dto_is_ros_free_and_service_includes_it(tmp_path):
    dto, service = generate_cpp(tmp_path)

    for dependency in (
        'rclcpp',
        'rclcpp_lifecycle',
        'fmt/',
        'parameter_validators',
        'std::mutex',
        'ParamsListener',
    ):
        assert dependency not in dto

    assert 'struct ParamsStamp' in dto
    assert 'std::int32_t sec = 0;' in dto
    assert 'std::uint32_t nanosec = 0;' in dto
    assert 'ParamsStamp __stamp;' in dto
    assert '#include "float_params_dto.hpp"' in service
    assert 'class ParamsListener' in service
    assert 'struct Params {' not in service
    assert 'builtin_interfaces::msg::Time now = clock_.now();' in service


def test_cpp_split_respects_custom_class_name(tmp_path):
    template = tmp_path / 'custom.yaml'
    template.write_text(
        'class_name: ControllerConfig\n'
        'code_namespace: custom/config\n'
        'ros__parameters:\n'
        '  enabled:\n'
        '    type: bool\n'
        '    default_value: true\n',
        encoding='utf-8',
    )
    dto_output = tmp_path / 'custom_params_dto.hpp'
    service_output = tmp_path / 'custom_params_service.hpp'

    run_cpp(str(dto_output), str(service_output), str(template))
    dto = dto_output.read_text(encoding='utf-8')
    service = service_output.read_text(encoding='utf-8')

    assert 'namespace custom::config {' in dto
    assert 'struct ControllerConfigStamp' in dto
    assert 'struct ControllerConfig {' in dto
    assert 'struct StackControllerConfig {' in dto
    assert 'class ControllerConfigListener' in service


def test_cpp_user_validation_header_is_service_only(tmp_path):
    dto_output = tmp_path / 'float_params_dto.hpp'
    service_output = tmp_path / 'float_params_service.hpp'

    run_cpp(
        str(dto_output),
        str(service_output),
        str(FLOAT_TEMPLATE),
        'custom_validators.hpp',
    )
    dto = dto_output.read_text(encoding='utf-8')
    service = service_output.read_text(encoding='utf-8')

    assert 'custom_validators.hpp' not in dto
    assert '#include "custom_validators.hpp"' in service
    assert 'bounds<float>' in service
    assert 'element_bounds<float>' in service


def test_python_and_documentation_generators_accept_float_types(tmp_path):
    python_output = tmp_path / 'float_parameters.py'
    markdown_output = tmp_path / 'float_parameters.md'
    rst_output = tmp_path / 'float_parameters.rst'

    run_python(str(python_output), str(FLOAT_TEMPLATE))
    run_documentation(str(FLOAT_TEMPLATE), str(markdown_output), 'markdown')
    run_documentation(str(FLOAT_TEMPLATE), str(rst_output), 'rst')

    python_generated = python_output.read_text(encoding='utf-8')
    assert 'scalar = 1.25' in python_generated
    assert 'array = [1.5, -2.25]' in python_generated
    assert 'Parameter.Type.DOUBLE' in python_generated
    assert 'Parameter.Type.DOUBLE_ARRAY' in python_generated
    assert 'Parameter.Type.FLOAT' not in python_generated

    assert 'float' in markdown_output.read_text(encoding='utf-8')
    assert 'float' in rst_output.read_text(encoding='utf-8')


@pytest.mark.parametrize(
    'defined_type, default_value',
    [
        ('float', '"not numeric"'),
        ('float_array', '[1.0, "not numeric"]'),
    ],
)
def test_float_types_reject_invalid_defaults(
    tmp_path, defined_type, default_value
):
    template = tmp_path / 'invalid_float.yaml'
    template.write_text(
        'code_namespace: invalid_float\n'
        'ros__parameters:\n'
        '  value:\n'
        f'    type: {defined_type}\n'
        f'    default_value: {default_value}\n',
        encoding='utf-8',
    )

    with pytest.raises(YAMLSyntaxError):
        run_cpp(
            str(tmp_path / 'invalid_params_dto.hpp'),
            str(tmp_path / 'invalid_params_service.hpp'),
            str(template),
        )
