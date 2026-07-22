"""Reusable launch-file helpers for hierarchical parameter overriding.

The intended layering per node, encoded as the order of the ``parameters=[...]``
list handed to a ``launch_ros`` Node action (later entries override earlier
ones parameter-by-parameter):

1. runtime parameter yaml files, in the order given (agent-specific sections
   are expressed inside the files with wildcard node keys, e.g. ``/veh2/**:``)
2. globally declared launch arguments that were explicitly provided
3. agent-specific command line overrides (``<vehicle_id>_overrides`` dicts)

Explicitness of launch arguments is detected with a snapshot of the launch
context's configuration store taken *before* the arguments are declared: at
that point the store only contains values that were provided externally (via
the command line or a parent launch file), so membership in the snapshot is an
exact provenance record and never requires comparing values against defaults.
"""

import glob
import os
from typing import Any, Dict, List

import yaml
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from avt_341_param_lib.parse_yaml import (
    PARAM_NAMESPACE_ROOT_KEY,
    PARAMETERS_ROOT_KEY,
)

_SCALAR_TYPES = ('bool', 'int', 'double', 'string')
_ARRAY_SUFFIX = '_array'
_MAP_PREFIX = '__map_'


class ParameterSpec:
    """Metadata of one parameter from a template yaml file."""

    def __init__(self, name: str, param_type: str, default_value: Any, description: str):
        self.name = name
        self.param_type = param_type
        self.default_value = default_value
        self.description = description


def _flatten_parameters(node: dict, prefix: str, out: Dict[str, ParameterSpec]):
    for key, value in node.items():
        if key.startswith(_MAP_PREFIX):
            # dynamically mapped parameters cannot be pre-declared as launch arguments
            continue
        if not isinstance(value, dict):
            raise ValueError(
                f"Unexpected entry '{prefix}{key}' in parameter template: expected "
                'a parameter definition or a nested group'
            )
        if isinstance(value.get('type'), str):
            if value['type'] == 'none':
                continue
            name = prefix + key
            out[name] = ParameterSpec(
                name, value['type'], value.get('default_value'), value.get('description', ''))
        else:
            _flatten_parameters(value, prefix + key + '.', out)


def load_template_metadata(path: str):
    """Read a parameter template yaml file into launch-layer metadata.

    Returns a tuple (partition, specs) where partition is the template's
    param_namespace (or the file stem when unspecified) and specs maps the
    flattened parameter name to its :class:`ParameterSpec`.
    """
    with open(path) as f:
        doc = yaml.safe_load(f)
    if not isinstance(doc, dict) or not isinstance(doc.get(PARAMETERS_ROOT_KEY), dict):
        raise ValueError(
            f"'{path}' is not a parameter template yaml file: missing a "
            f'{PARAMETERS_ROOT_KEY} root element'
        )
    partition = doc.get(PARAM_NAMESPACE_ROOT_KEY) or os.path.splitext(os.path.basename(path))[0]
    specs: Dict[str, ParameterSpec] = {}
    _flatten_parameters(doc[PARAMETERS_ROOT_KEY], '', specs)
    return partition, specs


def _convert_scalar(value: Any, base_type: str, arg_name: str):
    if base_type == 'bool' and isinstance(value, bool):
        return value
    if base_type == 'int' and isinstance(value, int) and not isinstance(value, bool):
        return value
    if base_type == 'double' and isinstance(value, (int, float)) and not isinstance(value, bool):
        return float(value)
    if base_type == 'string':
        return str(value)
    raise ValueError(
        f"Value '{value}' of launch argument '{arg_name}' is not a valid {base_type}")


def convert_cli_value(raw_text: str, param_type: str, arg_name: str):
    """Convert the string value of a launch argument to the template's declared type."""
    if param_type == 'string':
        # the declared type is known, so string parameters take the text
        # verbatim - no quoting needed for values that look numeric
        return raw_text
    parsed = yaml.safe_load(raw_text)
    if param_type.endswith(_ARRAY_SUFFIX):
        base_type = param_type[: -len(_ARRAY_SUFFIX)]
        if not isinstance(parsed, list):
            raise ValueError(
                f"Value '{raw_text}' of launch argument '{arg_name}' is not a "
                f'valid {param_type}; expected e.g. "[a, b, c]"'
            )
        return [_convert_scalar(item, base_type, arg_name) for item in parsed]
    return _convert_scalar(parsed, param_type, arg_name)


class ParameterCollection:
    """Declares launch override arguments for template parameter files and
    resolves which of them were explicitly provided."""

    def __init__(self, partitions: Dict[str, Dict[str, ParameterSpec]]):
        self._partitions = partitions
        self._explicit = set()

    @classmethod
    def from_template_files(cls, paths: List[str]) -> 'ParameterCollection':
        partitions: Dict[str, Dict[str, ParameterSpec]] = {}
        for path in paths:
            partition, specs = load_template_metadata(path)
            if partition in partitions:
                raise ValueError(
                    f"Duplicate parameter partition '{partition}' while loading '{path}'. "
                    f'Set a unique {PARAM_NAMESPACE_ROOT_KEY} in the template files.'
                )
            partitions[partition] = specs
        return cls(partitions)

    @classmethod
    def from_template_folder(cls, folder: str, recursive: bool = False) -> 'ParameterCollection':
        """Load every parameter template yaml file under the given folder."""
        if not os.path.isdir(folder):
            raise ValueError(f"'{folder}' is not a directory")
        pattern = os.path.join(folder, '**/*' if recursive else '*')
        paths = sorted(
            path for path in glob.glob(pattern, recursive=recursive)
            if os.path.isfile(path) and os.path.splitext(path)[1] in ('.yaml', '.yml')
        )
        if not paths:
            raise ValueError(f"No parameter template yaml files found under '{folder}'")
        return cls.from_template_files(paths)

    @property
    def partitions(self) -> List[str]:
        return list(self._partitions.keys())

    def argument_names(self) -> List[str]:
        return [
            self._argument_name(partition, name)
            for partition, specs in self._partitions.items()
            for name in specs
        ]

    def declare_parameters(self) -> list:
        """Snapshot + declaration actions for the launch file owning the arguments."""
        return [OpaqueFunction(function=self._snapshot_cmd_args), *self._declare_arguments()]

    def client_declare_actions(self) -> list:
        """Snapshot + declaration + scrub actions for a client launch file that
        re-declares the arguments before including the owning launch file.

        The scrub removes only this instance's non-provided argument defaults
        from the context, so the included file's explicitness detection stays
        exact while arguments of any other included launch files keep the
        default launch behavior.
        """
        return [
            OpaqueFunction(function=self._snapshot_cmd_args),
            *self._declare_arguments(),
            OpaqueFunction(function=self._scrub),
        ]

    def explicit_overrides(self, context, partition: str) -> Dict[str, Any]:
        """Typed values of this partition's explicitly provided arguments."""
        overrides: Dict[str, Any] = {}
        for name, spec in self._partitions[partition].items():
            arg_name = self._argument_name(partition, name)
            if arg_name in self._explicit:
                raw = LaunchConfiguration(arg_name).perform(context)
                overrides[name] = convert_cli_value(raw, spec.param_type, arg_name)
        return overrides

    def validate_explicit(self, context):
        """Raise on provided arguments that look like partitioned override
        arguments of this instance but do not match any declared parameter."""
        declared = set(self.argument_names())
        prefixes = tuple(f'{partition}/' for partition in self._partitions)
        stray = sorted(
            key for key in context.launch_configurations
            if key.startswith(prefixes) and key not in declared
        )
        if stray:
            raise RuntimeError(
                f'Unknown parameter override argument(s): {stray}. '
                f'Declared arguments are: {sorted(declared)}'
            )

    def _argument_name(self, partition: str, name: str) -> str:
        return f'{partition}/{name}'

    def _declare_arguments(self) -> list:
        return [
            DeclareLaunchArgument(
                self._argument_name(partition, spec.name),
                default_value='' if spec.default_value is None else str(spec.default_value),
                description=spec.description or spec.name,
            )
            for partition, specs in self._partitions.items()
            for spec in specs.values()
        ]

    def _snapshot_cmd_args(self, context, *args, **kwargs):
        self._explicit.update(context.launch_configurations.keys())
        return []

    def _scrub(self, context, *args, **kwargs):
        for arg_name in self.argument_names():
            if arg_name not in self._explicit:
                context.launch_configurations.pop(arg_name, None)
        return []


def perform_yaml(context, arg_name: str):
    """Perform a launch configuration and parse its value as yaml."""
    return yaml.safe_load(LaunchConfiguration(arg_name).perform(context))


def vehicle_overrides(context, vehicle_id: str) -> Dict[str, Any]:
    """Parse the dynamically named ``<vehicle_id>_overrides`` launch argument.

    The argument does not need to be declared; it defaults to an empty mapping.
    """
    raw = LaunchConfiguration(f'{vehicle_id}_overrides', default='{}').perform(context)
    value = yaml.safe_load(raw)
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError(
            f"Launch argument '{vehicle_id}_overrides' must hold a yaml mapping "
            f"of parameter names to values, got '{raw}'"
        )
    return value
