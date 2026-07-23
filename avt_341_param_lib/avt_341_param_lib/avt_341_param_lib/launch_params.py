"""Reusable launch-file helpers for hierarchical parameter overriding.

Command line parameter overrides use the same node-selector convention as ROS 2
parameter override yaml files: a selector addresses nodes by namespace and node
name with ``/`` delimiters, where ``**`` matches any number of tokens and ``*``
exactly one, and parameter names use ``.`` for nesting. Two argument forms are
supported:

* scalar form -- ``<selector>/<param.name>:=<value>``, e.g.
  ``**/cruise_speed:=9.0`` or ``veh1/planner/cruise_speed:=3.3``
* mapping form -- ``<selector>:=<yaml mapping>``, e.g.
  ``/veh1/planner:='{cruise_speed: 3.3}'``; the mapping is the selector
  section's ``ros__parameters`` body (nested mappings express parameter groups)

ROS 2 parameters can never hold mappings, so a mapping value always means
"section body" and a scalar/list value always means "single parameter"; the two
forms are unambiguous.

Relative selectors (no leading ``/`` or ``**``) are resolved against the
vehicle id list: a first segment naming a vehicle scopes the override to that
vehicle (``veh1/planner/... -> /veh1/planner/...``); any other first segment
applies across all vehicles by prefixing a single-token wildcard for the
vehicle id (``planner/... -> /*/planner/...``). Vehicle ids are assumed to be
single namespace tokens; deeper relative paths address nested node namespaces
below the vehicle (``nav/planner/... -> /*/nav/planner/...``). A selector
ending at a bare vehicle namespace targets all of that vehicle's nodes
(``veh1 -> /veh1/**``).

Per-node override priority (later wins, per parameter):

1. runtime parameter yaml files, in the order given
2. command line overrides, in command line order

Explicitness of launch arguments is detected with a snapshot of the launch
context's configuration store taken *before* the arguments are declared: at
that point the store only contains values that were provided externally (via
the command line or a parent launch file), so membership in the snapshot is an
exact provenance record and never requires comparing values against defaults.
The declared arguments themselves exist for documentation (``ros2 launch -s``);
each declared name doubles as valid override syntax (``planner/cruise_speed``).
"""

import functools
from collections import OrderedDict
from typing import Any, Dict, Iterable, List, Tuple

import yaml
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from avt_341_param_lib.node_selectors import selector_matches
from avt_341_param_lib.parse_yaml import PARAMETERS_ROOT_KEY

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


def load_template_metadata(path: str) -> Dict[str, ParameterSpec]:
    """Read a parameter template yaml file into launch-layer metadata.

    Returns a mapping of the flattened (dot-nested) parameter name to its
    :class:`ParameterSpec`.
    """
    with open(path) as f:
        doc = yaml.safe_load(f)
    if not isinstance(doc, dict) or not isinstance(doc.get(PARAMETERS_ROOT_KEY), dict):
        raise ValueError(
            f"'{path}' is not a parameter template yaml file: missing a "
            f'{PARAMETERS_ROOT_KEY} root element'
        )
    specs: Dict[str, ParameterSpec] = {}
    _flatten_parameters(doc[PARAMETERS_ROOT_KEY], '', specs)
    return specs


def _convert_scalar(value: Any, base_type: str, arg_name: str):
    if base_type == 'bool' and isinstance(value, bool):
        return value
    if base_type == 'int' and isinstance(value, int) and not isinstance(value, bool):
        return value
    if base_type in ('double', 'float') and isinstance(
        value, (int, float)
    ) and not isinstance(value, bool):
        return float(value)
    if base_type == 'string':
        return str(value)
    raise ValueError(
        f"Value '{value}' of parameter override '{arg_name}' is not a valid {base_type}")


def convert_typed_value(value: Any, param_type: str, arg_name: str):
    """Convert an already yaml-typed override value to the template's declared type."""
    if param_type.endswith(_ARRAY_SUFFIX):
        base_type = param_type[: -len(_ARRAY_SUFFIX)]
        if not isinstance(value, list):
            raise ValueError(
                f"Value '{value}' of parameter override '{arg_name}' is not a "
                f'valid {param_type}; expected e.g. "[a, b, c]"'
            )
        return [_convert_scalar(item, base_type, arg_name) for item in value]
    return _convert_scalar(value, param_type, arg_name)


def convert_cli_value(raw_text: str, param_type: str, arg_name: str):
    """Convert the string value of a launch argument to the template's declared type."""
    if param_type == 'string':
        # the declared type is known, so string parameters take the text
        # verbatim - no quoting needed for values that look numeric
        return raw_text
    return convert_typed_value(yaml.safe_load(raw_text), param_type, arg_name)


@functools.lru_cache(maxsize=None)
def _load_params_file(path: str) -> dict:
    with open(path) as f:
        doc = yaml.safe_load(f)
    if not isinstance(doc, dict):
        raise ValueError(
            f"'{path}' is not a parameter yaml file: expected a mapping of node sections")
    return doc


def _iter_section_selectors(doc: dict, prefix: str = ''):
    """Yield the node selectors of a parameter override yaml document.

    Supports both flat section keys (``/veh2/**:``) and nested namespace
    mappings (``veh2: {planner: {ros__parameters: ...}}``).
    """
    for key, value in doc.items():
        if not isinstance(value, dict):
            continue
        selector = f"{prefix}/{str(key).strip('/')}"
        if PARAMETERS_ROOT_KEY in value:
            yield selector
        else:
            yield from _iter_section_selectors(value, selector)


def relevant_params_files(paths: Iterable[str], fqn: str) -> List[str]:
    """Filter parameter yaml files to those with a section that can apply to the node.

    Order is preserved. The check is conservative: sections whose keys do not
    start with ``/`` are also tried with an implicit leading ``/**`` so that
    relative section keys are never filtered out incorrectly; rcl still
    performs the exact matching when the file is loaded by the node.
    """
    result = []
    for path in paths:
        selectors = _iter_section_selectors(_load_params_file(path))
        if any(
            selector_matches(selector, fqn) or selector_matches('/**' + selector, fqn)
            for selector in selectors
        ):
            result.append(path)
    return result


def _normalize_selector(selector: str, vehicles: List[str]) -> str:
    """Expand a possibly relative node selector to an absolute one.

    Relative selectors starting with a vehicle id become vehicle-scoped; any
    other relative selector applies across all vehicles via a single-token
    wildcard for the vehicle id (vehicle ids are single namespace tokens, so
    intermediate tokens of a relative selector always address nested node
    namespaces, never a vehicle). A selector that ends at a bare vehicle
    namespace targets all of that vehicle's nodes.
    """
    text = selector.strip().rstrip('/')
    if not text or text.startswith('//'):
        raise RuntimeError(f"Invalid node selector '{selector}' in parameter override")
    if text.startswith('**'):
        text = '/' + text
    elif not text.startswith('/'):
        first_segment = text.split('/', 1)[0]
        text = f'/{text}' if first_segment in vehicles else f'/*/{text}'
    for token in text.split('/'):
        if '*' in token and token not in ('', '*', '**'):
            raise RuntimeError(
                f"Unsupported wildcard token '{token}' in node selector '{selector}'; "
                "only whole-token '*' and '**' wildcards are supported"
            )
    if text.strip('/') in vehicles:
        text += '/**'
    return text


def _flatten_value_mapping(mapping: dict, arg_name: str, prefix: str = '') -> Dict[str, Any]:
    """Flatten a mapping-form override value into dot-nested parameter names."""
    params: Dict[str, Any] = {}
    for key, value in mapping.items():
        if not isinstance(key, str):
            raise RuntimeError(
                f"Parameter override '{arg_name}' holds a non-string parameter name '{key}'")
        if isinstance(value, dict):
            params.update(_flatten_value_mapping(value, arg_name, prefix + key + '.'))
        else:
            params[prefix + key] = value
    if not params and not prefix:
        raise RuntimeError(f"Parameter override '{arg_name}' holds an empty mapping")
    return params




class ResolvedOverrides:
    """Per-node parameter overrides resolved from explicit launch arguments."""

    def __init__(self, params_by_fqn: Dict[str, Dict[str, Any]]):
        self._params_by_fqn = params_by_fqn

    def for_node(self, fqn: str) -> Dict[str, Any]:
        """Merged override parameters applying to the node; empty when none match."""
        return dict(self._params_by_fqn.get(fqn, {}))


class ParameterCollection:
    """Declares launch override arguments for the nodes of a launch file and
    resolves explicitly provided overrides into per-node parameter dicts.

    Construct with a mapping of node name to template yaml file; several node
    names may share one template.
    """

    def __init__(self, node_specs: Dict[str, Dict[str, ParameterSpec]]):
        self._node_specs = node_specs
        self._explicit = set()

    @classmethod
    def from_node_templates(cls, node_templates: Dict[str, str]) -> 'ParameterCollection':
        """Build a collection from a node-name -> template-yaml-path mapping."""
        specs_by_path: Dict[str, Dict[str, ParameterSpec]] = {}
        node_specs: Dict[str, Dict[str, ParameterSpec]] = {}
        for node_name, path in node_templates.items():
            if path not in specs_by_path:
                specs_by_path[path] = load_template_metadata(path)
            node_specs[node_name] = specs_by_path[path]
        return cls(node_specs)

    @property
    def node_names(self) -> List[str]:
        return list(self._node_specs.keys())

    def argument_names(self) -> List[str]:
        return [
            f'{node_name}/{name}'
            for node_name, specs in self._node_specs.items()
            for name in specs
        ]

    def declare_arguments(self) -> list:
        """Snapshot + declaration actions for the launch file owning the arguments."""
        return [OpaqueFunction(function=self._snapshot_cmd_args), *self._declare_arguments()]

    def client_declare_arguments(self) -> list:
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

    def resolve_cli_overrides(self, context, vehicle_ids: List[str]) -> ResolvedOverrides:
        """Resolve the explicitly provided override arguments for the given vehicles.

        Scans the pre-declaration snapshot for override-shaped arguments (any
        name containing ``/``, starting with ``**``, or equal to a vehicle id
        or node name), expands them against the vehicle id list and validates
        them against the node templates. Raises RuntimeError for overrides
        that match no launched node or name no parameter declared by any
        matched node.
        """
        vehicles = [str(vid).strip('/') for vid in vehicle_ids]
        cli_args = OrderedDict(
            (key, value) for key, value in context.launch_configurations.items()
            if key in self._explicit and self._is_override_key(key, vehicles)
        )
        return self.resolve(cli_args, vehicle_ids)

    def _is_override_key(self, key: str, vehicles: List[str]) -> bool:
        return (
            '/' in key or key.startswith('**')
            or key.strip('/') in vehicles or key in self._node_specs
        )

    def resolve(self, cli_args: 'OrderedDict[str, str]', vehicle_ids: List[str]) -> ResolvedOverrides:
        """Pure resolution core of :meth:`resolve_cli_overrides` (testable without a context)."""
        vehicles = [str(vid).strip('/') for vid in vehicle_ids]
        node_by_fqn = {
            f'/{vid}/{node_name}': node_name
            for vid in vehicles
            for node_name in self._node_specs
        }
        params_by_fqn: Dict[str, Dict[str, Any]] = {}
        problems = []
        for key, raw_value in cli_args.items():
            try:
                selector, params, values_are_text = self._parse_override(key, raw_value, vehicles)
            except RuntimeError as error:
                problems.append(str(error))
                continue
            matched = [fqn for fqn in node_by_fqn if selector_matches(selector, fqn)]
            if not matched:
                problems.append(
                    f"'{key}' (node selector '{selector}') matches none of the launched nodes")
                continue
            for name, value in params.items():
                applied = False
                for fqn in matched:
                    spec = self._node_specs[node_by_fqn[fqn]].get(name)
                    if spec is None:
                        # entries a matched node does not declare stay dormant,
                        # mirroring yaml parameter file semantics
                        continue
                    convert = convert_cli_value if values_are_text else convert_typed_value
                    params_by_fqn.setdefault(fqn, {})[name] = convert(
                        value, spec.param_type, key)
                    applied = True
                if not applied:
                    problems.append(
                        f"'{key}': parameter '{name}' is not declared by any matched node")
        if problems:
            raise RuntimeError(
                'Invalid parameter override argument(s): ' + '; '.join(problems)
                + f'. Declared override arguments are: {sorted(self.argument_names())}'
            )
        return ResolvedOverrides(params_by_fqn)

    def _parse_override(
        self, key: str, raw_value: str, vehicles: List[str]
    ) -> Tuple[str, Dict[str, Any], bool]:
        """Split one override argument into (selector, params, values_are_text)."""
        try:
            value = yaml.safe_load(raw_value)
        except yaml.YAMLError:
            value = None  # not yaml: can only be a verbatim string parameter value
        if isinstance(value, dict):
            # mapping form: the whole key is the node selector
            if set(value) == {PARAMETERS_ROOT_KEY}:
                value = value[PARAMETERS_ROOT_KEY]
            selector = _normalize_selector(key, vehicles)
            return selector, _flatten_value_mapping(value, key), False
        # scalar form: the last slash segment is the (possibly dotted) parameter name
        selector_text, sep, param_name = key.rpartition('/')
        if not sep or not selector_text or not param_name or '*' in param_name:
            raise RuntimeError(
                f"Malformed parameter override '{key}:={raw_value}': expected "
                "'<node-selector>/<param.name>:=<value>' or '<node-selector>:=<yaml mapping>'"
            )
        selector = _normalize_selector(selector_text, vehicles)
        return selector, {param_name: raw_value}, True

    def _declare_arguments(self) -> list:
        return [
            DeclareLaunchArgument(
                f'{node_name}/{spec.name}',
                default_value='' if spec.default_value is None else str(spec.default_value),
                description=spec.description or spec.name,
            )
            for node_name, specs in self._node_specs.items()
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
