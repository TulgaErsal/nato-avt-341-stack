"""Load launch-only node metadata selected by ROS 2-style node paths."""

import os
import re
from collections import OrderedDict
from typing import Dict, List, Optional, Tuple

import yaml
from launch.substitutions import EnvironmentVariable

from avt_341_param_lib.node_selectors import selector_matches

_REMAPPINGS_KEY = 'remappings'
_ADDITIONAL_ENV_KEY = 'additional_env'
_METADATA_KEYS = frozenset((_REMAPPINGS_KEY, _ADDITIONAL_ENV_KEY))
_ENV_VAR_RE = re.compile(r'\$env_var\{(?P<expression>[^{}]*)\}')
_ENV_NAME_RE = re.compile(r'^[A-Za-z_][A-Za-z0-9_]*$')


def _config_error(path: str, message: str, selector: Optional[str] = None) -> ValueError:
    location = f", selector '{selector}'" if selector else ''
    return ValueError(f"Invalid node metadata file '{path}'{location}: {message}")


def _join_selector(path: str, prefix: str, key) -> str:
    if not isinstance(key, str):
        raise _config_error(path, f'selector keys must be strings, got {key!r}', prefix or None)
    if not key or key != key.strip():
        raise _config_error(path, f'invalid selector key {key!r}', prefix or None)
    if key in _METADATA_KEYS:
        raise _config_error(
            path, f"'{key}' must be inside a node selector section", prefix or None)

    stripped = key.strip('/')
    if not stripped or '//' in key:
        raise _config_error(path, f"invalid node selector component '{key}'", prefix or None)

    for token in stripped.split('/'):
        if '*' in token and token not in ('*', '**'):
            raise _config_error(
                path,
                f"unsupported wildcard token '{token}'; only whole-token '*' and '**' "
                'wildcards are supported',
                prefix or '/' + stripped,
            )

    if prefix:
        return prefix.rstrip('/') + '/' + stripped
    return '/' + stripped


def _parse_remappings(path: str, selector: str, value) -> List[Tuple[str, str]]:
    if not isinstance(value, dict):
        raise _config_error(path, "'remappings' must be a source-to-target mapping", selector)

    result = []
    for source, target in value.items():
        if not isinstance(source, str) or not source.strip():
            raise _config_error(
                path, f'remapping source names must be non-empty strings, got {source!r}',
                selector)
        if not isinstance(target, str) or not target.strip():
            raise _config_error(
                path,
                f"target for remapping source '{source}' must be a non-empty string, "
                f'got {target!r}',
                selector,
            )
        result.append((source, target))
    return result


def _parse_env_template(
    path: str, selector: str, variable_name: str, text: str
) -> List[object]:
    """Compile embedded $env_var{} expressions to ROS launch substitutions."""
    matches = list(_ENV_VAR_RE.finditer(text))
    if text.count('$env_var{') != len(matches):
        raise _config_error(
            path,
            f"malformed $env_var{{}} expression in additional_env variable "
            f"'{variable_name}': {text!r}",
            selector,
        )
    if not matches:
        return [text]

    result = []
    offset = 0
    for match in matches:
        if match.start() > offset:
            result.append(text[offset:match.start()])

        expression = match.group('expression')
        env_name, has_default, default_value = expression.partition(':-')
        if not _ENV_NAME_RE.fullmatch(env_name):
            raise _config_error(
                path,
                f"invalid environment variable name {env_name!r} in additional_env "
                f"variable '{variable_name}'",
                selector,
            )
        result.append(EnvironmentVariable(
            env_name,
            default_value=default_value if has_default else None,
        ))
        offset = match.end()

    if offset < len(text):
        result.append(text[offset:])
    return result


def _parse_env_assignment(path: str, selector: str, variable_name: str, value):
    if isinstance(value, str):
        return _parse_env_template(path, selector, variable_name, value)

    if not isinstance(value, dict):
        raise _config_error(
            path,
            f"additional_env variable '{variable_name}' must be a string or a "
            "{'separator', 'values'} mapping",
            selector,
        )

    unexpected = [key for key in value if key not in ('separator', 'values')]
    missing = [key for key in ('separator', 'values') if key not in value]
    if unexpected or missing:
        details = []
        if unexpected:
            details.append(f'unexpected field(s): {unexpected}')
        if missing:
            details.append(f'missing field(s): {missing}')
        raise _config_error(
            path,
            f"invalid list assignment for additional_env variable '{variable_name}': "
            + '; '.join(details),
            selector,
        )

    separator = value['separator']
    values = value['values']
    if not isinstance(separator, str):
        raise _config_error(
            path,
            f"separator for additional_env variable '{variable_name}' must be a string",
            selector,
        )
    if not isinstance(values, list) or not values:
        raise _config_error(
            path,
            f"values for additional_env variable '{variable_name}' must be a non-empty list",
            selector,
        )

    result = []
    for index, item in enumerate(values):
        if not isinstance(item, str):
            raise _config_error(
                path,
                f"value {index} for additional_env variable '{variable_name}' "
                f'must be a string, got {item!r}',
                selector,
            )
        if index:
            result.append(separator)
        result.extend(_parse_env_template(path, selector, variable_name, item))
    return result


def _parse_additional_env(path: str, selector: str, value) -> Dict[str, List[object]]:
    if not isinstance(value, dict):
        raise _config_error(
            path, "'additional_env' must be an environment-variable mapping", selector)

    result = OrderedDict()
    for variable_name, assignment in value.items():
        if (
            not isinstance(variable_name, str)
            or not _ENV_NAME_RE.fullmatch(variable_name)
        ):
            raise _config_error(
                path,
                f'environment variable names must be valid non-empty strings, '
                f'got {variable_name!r}',
                selector,
            )
        result[variable_name] = _parse_env_assignment(
            path, selector, variable_name, assignment)
    return result


def _parse_selector_tree(path: str, mapping: dict, prefix: str = ''):
    for key, value in mapping.items():
        selector = _join_selector(path, prefix, key)
        if not isinstance(value, dict):
            raise _config_error(path, 'selector entries must be mappings', selector)

        present_metadata_keys = [name for name in value if name in _METADATA_KEYS]
        if present_metadata_keys:
            unexpected = [name for name in value if name not in _METADATA_KEYS]
            if unexpected:
                raise _config_error(
                    path,
                    f'unexpected field(s) beside node metadata: {unexpected}',
                    selector,
                )
            remappings = (
                _parse_remappings(path, selector, value[_REMAPPINGS_KEY])
                if _REMAPPINGS_KEY in value else []
            )
            additional_env = (
                _parse_additional_env(path, selector, value[_ADDITIONAL_ENV_KEY])
                if _ADDITIONAL_ENV_KEY in value else {}
            )
            yield selector, remappings, additional_env
            continue

        if not value:
            raise _config_error(
                path,
                "selector section is missing a 'remappings' or 'additional_env' mapping",
                selector,
            )
        yield from _parse_selector_tree(path, value, selector)


def _validate_node_fqn(node_fqn: str) -> None:
    if (
        not isinstance(node_fqn, str)
        or not node_fqn.startswith('/')
        or node_fqn == '/'
        or node_fqn.endswith('/')
        or '//' in node_fqn
        or '*' in node_fqn
    ):
        raise ValueError(
            f"Expected an absolute fully qualified node name, got {node_fqn!r}")


class MetadataCollection:
    """Launch metadata selected for nodes by fully qualified node name.

    ``path`` names zero or one launch-only metadata YAML file. ``None`` and
    the empty string create an empty collection.
    """

    def __init__(self, path=None):
        self._sections = []
        if path is None:
            return

        path_text = os.fspath(path)
        if not path_text:
            return

        try:
            with open(path_text, encoding='utf-8') as stream:
                document = yaml.safe_load(stream)
        except yaml.YAMLError as error:
            raise _config_error(path_text, f'invalid YAML: {error}') from error

        if document is None:
            return
        if not isinstance(document, dict):
            raise _config_error(
                path_text, 'expected a mapping of node selector sections')
        if document:
            self._sections = list(_parse_selector_tree(path_text, document))

    def get_remappings(self, node_fqn: str) -> List[Tuple[str, str]]:
        """Return merged topic remappings for an absolute node FQN."""
        _validate_node_fqn(node_fqn)

        merged = OrderedDict()
        for selector, remappings, _ in self._sections:
            if not selector_matches(selector, node_fqn):
                continue
            for source, target in remappings:
                if source in merged:
                    del merged[source]
                merged[source] = target
        return list(merged.items())

    def get_additional_env(self, node_fqn: str) -> Dict[str, List[object]]:
        """Return merged additional process environment for an absolute node FQN.

        Values are lists of strings and ROS launch substitutions suitable for
        passing directly to ``launch_ros.actions.Node(additional_env=...)``.
        """
        _validate_node_fqn(node_fqn)

        merged = OrderedDict()
        for selector, _, additional_env in self._sections:
            if not selector_matches(selector, node_fqn):
                continue
            for variable_name, assignment in additional_env.items():
                if variable_name in merged:
                    del merged[variable_name]
                merged[variable_name] = list(assignment)
        return dict(merged)
