"""Value templating for runtime parameter override yaml files.

Applies only to the runtime override parameter yaml files a launch file layers
onto its nodes (the ``params_files`` mechanism) -- never to the
code-generation template yaml files consumed by
:mod:`avt_341_param_lib.parse_yaml`; templating syntax in those files is not
interpreted.

A ``$python{}``/``$ref{}`` template must be the entire scalar string value of
a parameter; ``$pkg_path{}`` substitutes in place within a string value:

* ``$python{<expression>}`` -- replaced by the result of evaluating
  ``<expression>`` with a small namespace holding ``os``, ``math`` and
  ``get_package_share_directory``. Replaces the retired ``$Python:`` prefix
  and ROS 1 ``$(find pkg)`` syntaxes, e.g.::

      zones_filepath: $python{os.path.join(get_package_share_directory('avt_341'), 'config', 'zones.csv')}

* ``$ref{<selector>/<param.name>}`` -- replaced by the fully resolved value of
  another parameter in the same params_files corpus. The selector uses the
  node-selector convention shared by rcl section keys and the command line
  overrides (``**`` matches any number of tokens, ``*`` exactly one; relative
  selectors are resolved against the vehicle id list); the parameter name
  nests with ``.``. When wildcards could address multiple entries, the first
  match wins (params_files order, then document order). Referenced values are
  resolved before substitution, so ``$ref`` -> ``$ref``/``$python`` chains
  work; circular references raise an error.

* ``$pkg_path{<pkg_name>}`` -- each occurrence is replaced by the package's
  share directory (``get_package_share_directory``). Unlike the other
  templates it may appear anywhere -- and several times -- within a string
  value, e.g.::

      zones_filepath: $pkg_path{avt_341}/config/zones.csv
"""

import math
import os
import re
import tempfile
from typing import Any, List, Optional, Tuple

import yaml
from ament_index_python.packages import get_package_share_directory

from avt_341_param_lib.launch_params import (
    PKG_PATH_MARKER,
    _normalize_selector,
    expand_pkg_path,
)
from avt_341_param_lib.parse_yaml import PARAMETERS_ROOT_KEY

_PYTHON_RE = re.compile(r'^\$python\{(?P<expr>.*)\}$', re.DOTALL)
_REF_RE = re.compile(r'^\$ref\{(?P<target>[^{}]*)\}$')
_FULL_VALUE_MARKERS = ('$python{', '$ref{')
_TEMPLATE_MARKERS = _FULL_VALUE_MARKERS + (PKG_PATH_MARKER,)

_PYTHON_EVAL_NAMESPACE = {
    'os': os,
    'math': math,
    'get_package_share_directory': get_package_share_directory,
}


def resolve_params_files(
    paths: List[str], vehicle_ids: Optional[List[str]] = None, work_dir: Optional[str] = None
) -> List[str]:
    """Resolve ``$python{}``/``$ref{}``/``$pkg_path{}`` templates in runtime parameter yaml files.

    Returns one path per input, in order: files containing no templating
    syntax pass through with their original path; templated files are resolved
    and written to ``work_dir`` (a session temp directory is created when not
    given). Later files may reference values in earlier ones and vice versa.
    """
    resolver = _Resolver(paths, vehicle_ids or [])
    resolver.resolve_all()

    result = []
    for index, entry in enumerate(resolver.files):
        if not entry.templated:
            result.append(entry.path)
            continue
        if work_dir is None:
            work_dir = tempfile.mkdtemp(prefix='avt_341_resolved_params_')
        out_path = os.path.join(
            work_dir, '%02d_%s' % (index, os.path.basename(entry.path)))
        with open(out_path, 'w') as f:
            yaml.safe_dump(entry.doc, f, sort_keys=False)
        result.append(out_path)
    return result


class _ParamsFile:

    def __init__(self, path: str):
        self.path = path
        with open(path) as f:
            text = f.read()
        self.templated = any(marker in text for marker in _TEMPLATE_MARKERS)
        self.doc = yaml.safe_load(text)
        if not isinstance(self.doc, dict):
            raise RuntimeError(
                f"'{path}' is not a parameter yaml file: expected a mapping of node sections")


class _Resolver:

    def __init__(self, paths: List[str], vehicles: List[str]):
        self.files = [_ParamsFile(path) for path in paths]
        self._vehicles = [str(vid).strip('/') for vid in vehicles]
        # (section body id, param key) pairs currently being resolved, for cycle detection
        self._resolving = set()

    def resolve_all(self):
        for entry in self.files:
            if entry.templated:
                self._resolve_container(entry.doc, entry.path)

    def _resolve_container(self, container, path: str):
        items = container.items() if isinstance(container, dict) else enumerate(container)
        for key, value in items:
            if isinstance(value, (dict, list)):
                self._resolve_container(value, path)
            elif isinstance(value, str):
                container[key] = self._resolve_leaf(value, path)

    def _resolve_leaf(self, text: str, path: str):
        python_match = _PYTHON_RE.match(text)
        if python_match:
            return self._eval_python(python_match.group('expr'), path)
        ref_match = _REF_RE.match(text)
        if ref_match:
            return self._resolve_ref(ref_match.group('target'), path)
        if text.startswith(_FULL_VALUE_MARKERS):
            raise RuntimeError(
                f"Malformed template value '{text}' in '{path}': a $python{{}}/$ref{{}} "
                'template must be the entire value'
            )
        try:
            return expand_pkg_path(text, f"'{path}'")
        except ValueError as error:
            raise RuntimeError(str(error)) from error

    def _eval_python(self, expr: str, path: str):
        try:
            return eval(expr, dict(_PYTHON_EVAL_NAMESPACE))
        except Exception as error:
            raise RuntimeError(
                f"Error evaluating $python{{{expr}}} in '{path}': {error}") from error

    def _resolve_ref(self, target: str, path: str):
        selector_text, sep, param_name = target.rpartition('/')
        if not sep or not selector_text or not param_name or '*' in param_name:
            raise RuntimeError(
                f"Malformed reference '$ref{{{target}}}' in '{path}': expected "
                "'$ref{<node-selector>/<param.name>}', e.g. '$ref{**/max_speed}'"
            )
        selector = _normalize_selector(selector_text, self._vehicles)
        ref_tokens = [token for token in selector.split('/') if token]
        for entry in self.files:
            for section_selector, body in _iter_sections(entry.doc):
                section_tokens = [t for t in section_selector.split('/') if t]
                # relative section keys are conservatively also tried with an
                # implicit any-namespace prefix, mirroring relevant_params_files
                if not (
                    _patterns_intersect(ref_tokens, section_tokens)
                    or _patterns_intersect(ref_tokens, ['**'] + section_tokens)
                ):
                    continue
                found = _lookup_param(body, param_name)
                if found is None:
                    continue
                parent, key = found
                value = parent[key]
                if _is_template(value):
                    cycle_key = (id(parent), key)
                    if cycle_key in self._resolving:
                        raise RuntimeError(
                            f"Circular $ref chain detected while resolving "
                            f"'$ref{{{target}}}' in '{path}'"
                        )
                    self._resolving.add(cycle_key)
                    try:
                        value = self._resolve_leaf(value, entry.path)
                    finally:
                        self._resolving.discard(cycle_key)
                    parent[key] = value
                return value
        raise RuntimeError(
            f"Reference '$ref{{{target}}}' in '{path}' (node selector '{selector}') "
            f"matches no parameter in the given params files: "
            f"{[entry.path for entry in self.files]}"
        )


def _is_template(value) -> bool:
    """Whether a leaf value still holds unresolved templating syntax."""
    return isinstance(value, str) and (
        value.startswith(_FULL_VALUE_MARKERS) or PKG_PATH_MARKER in value)


def _iter_sections(doc: dict, prefix: str = ''):
    """Yield (node selector, ros__parameters body) pairs of an override yaml document.

    Supports both flat section keys (``/veh2/**:``) and nested namespace
    mappings (``veh2: {planner: {ros__parameters: ...}}``).
    """
    for key, value in doc.items():
        if not isinstance(value, dict):
            continue
        selector = f"{prefix}/{str(key).strip('/')}"
        if isinstance(value.get(PARAMETERS_ROOT_KEY), dict):
            yield selector, value[PARAMETERS_ROOT_KEY]
        else:
            yield from _iter_sections(value, selector)


def _patterns_intersect(a: List[str], b: List[str]) -> bool:
    """Whether two wildcard selector token patterns can address a common node."""
    if not a:
        return not b or all(token == '**' for token in b)
    if not b:
        return all(token == '**' for token in a)
    if a[0] == '**':
        return _patterns_intersect(a[1:], b) or _patterns_intersect(a, b[1:])
    if b[0] == '**':
        return _patterns_intersect(a, b[1:]) or _patterns_intersect(a[1:], b)
    if a[0] == '*' or b[0] == '*' or a[0] == b[0]:
        return _patterns_intersect(a[1:], b[1:])
    return False


def _lookup_param(body: dict, dotted_name: str) -> Optional[Tuple[dict, Any]]:
    """Locate a possibly dot-nested parameter in a section body.

    Returns the (parent mapping, key) holding the value, or None when absent.
    Both nested mappings and literal dotted keys are supported.
    """
    if not isinstance(body, dict):
        return None
    if dotted_name in body and not isinstance(body[dotted_name], dict):
        return body, dotted_name
    head, sep, rest = dotted_name.partition('.')
    if sep and isinstance(body.get(head), dict):
        return _lookup_param(body[head], rest)
    return None
