# Copyright 2022 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Parameter template yaml format shared by code generation and launch time.

This module owns the vocabulary of the parameter template format -- the root
elements, the mixin include mechanism and the errors raised while reading a
template. The build-time code generator (``avt_341_param_lib.codegen``) and the
launch-time helpers (``avt_341_param_lib.runtime``) both read templates, so both
depend on this module; it deliberately stays free of jinja2 and of any launch
dependency so that neither side pulls in the other's imports.
"""

from typing import List
from typeguard import typechecked
from yaml.parser import ParserError
from yaml.scanner import ScannerError
import os
import re
import yaml


# YAMLSyntaxError standardizes compiler error messages
class YAMLSyntaxError(Exception):
    def __init__(self, msg):
        self.msg = msg

    def __str__(self):
        return self.msg


# helper functions
@typechecked
def compile_error(msg: str):
    return YAMLSyntaxError('\nERROR: ' + msg)


# Root elements currently understood in the parameter template yaml file.
# Extend this tuple when new root elements are added to the template format.
CLASS_NAME_ROOT_KEY = 'class_name'
CODE_NAMESPACE_ROOT_KEY = 'code_namespace'
PARAMETERS_ROOT_KEY = 'ros__parameters'
KNOWN_ROOT_KEYS = (
    CLASS_NAME_ROOT_KEY,
    CODE_NAMESPACE_ROOT_KEY,
    PARAMETERS_ROOT_KEY,
)

DEFAULT_CLASS_NAME = 'Params'

# Mixin entries: a mapping inside the ros__parameters tree may hold an
# __include_mixins entry naming one or more mixin templates by bare file stem
# (comma-separated or a yaml list). The referenced files live in a "mixins"
# folder next to the including template and their ros__parameters content is
# spliced into the including mapping.
INCLUDE_MIXIN_KEY = '__include_mixins'
MIXINS_DIR_NAME = 'mixins'


@typechecked
def is_mapped_parameter(param_name: str):
    return param_name.__contains__('__map_')


@typechecked
def parse_code_namespace_tokens(code_namespace: str, root_key: str = CODE_NAMESPACE_ROOT_KEY) -> List[str]:
    """Split a slash-separated namespace value into validated identifier tokens."""
    tokens = code_namespace.split('/')
    identifier = re.compile(r'^[A-Za-z_][A-Za-z0-9_]*$')
    if not all(identifier.match(token) for token in tokens):
        raise compile_error(
            f"Invalid {root_key} '{code_namespace}'. Expected "
            'slash-separated identifiers, e.g. "my_controller" or "my_controller/variant_a"'
        )
    return tokens


def _contains_mixin_entry(node) -> bool:
    if not isinstance(node, dict):
        return False
    return INCLUDE_MIXIN_KEY in node or any(
        _contains_mixin_entry(value) for value in node.values()
    )


def _contains_mapped_parameter(node) -> bool:
    if not isinstance(node, dict):
        return False
    return any(
        is_mapped_parameter(key) or _contains_mapped_parameter(value)
        for key, value in node.items()
    )


def is_parameter_group(value) -> bool:
    """Whether a template tree value is a nested group (not a parameter definition)."""
    return isinstance(value, dict) and not isinstance(value.get('type'), str)


def read_mixin_file(mixin_path: str):
    """Load and validate one mixin template file.

    Returns ``(parameters, code_namespace)`` where ``parameters`` is the
    mixin's ros__parameters mapping.
    """
    if not os.path.isfile(mixin_path):
        raise compile_error(f"Mixin file '{mixin_path}' not found")
    with open(mixin_path) as f:
        try:
            doc = yaml.safe_load(f)
        except (ParserError, ScannerError) as e:
            raise compile_error(f"Invalid mixin file '{mixin_path}': {e}")
    if not isinstance(doc, dict) or not isinstance(
        doc.get(PARAMETERS_ROOT_KEY), dict
    ):
        raise compile_error(
            f"Mixin file '{mixin_path}' must be a mapping with a "
            f'{PARAMETERS_ROOT_KEY} root element'
        )
    unknown_keys = [key for key in doc if key not in KNOWN_ROOT_KEYS]
    if unknown_keys:
        raise compile_error(
            f"Unknown root element(s) {unknown_keys} in mixin file "
            f"'{mixin_path}'. Supported root elements are: {list(KNOWN_ROOT_KEYS)}"
        )
    code_namespace = doc.get(CODE_NAMESPACE_ROOT_KEY)
    if not isinstance(code_namespace, str):
        raise compile_error(
            f"Mixin file '{mixin_path}' must declare a "
            f'{CODE_NAMESPACE_ROOT_KEY} root element holding a string'
        )
    parameters = doc[PARAMETERS_ROOT_KEY]
    if _contains_mixin_entry(parameters):
        raise compile_error(
            f"Mixin file '{mixin_path}' holds a nested {INCLUDE_MIXIN_KEY} entry; "
            'mixins cannot include other mixins'
        )
    if _contains_mapped_parameter(parameters):
        raise compile_error(
            f"Mixin file '{mixin_path}' holds a dynamically mapped (__map_) "
            'parameter; mapped parameters are not supported in mixins'
        )
    return parameters, code_namespace


def _load_mixin_parameters(stem, template_path: str):
    """Resolve and load the mixin referenced by ``stem``.

    Returns ``(parameters, code_namespace)``.
    """
    if not isinstance(stem, str) or not stem:
        raise compile_error(
            f"Invalid {INCLUDE_MIXIN_KEY} value {stem!r} in '{template_path}': "
            'expected a mixin file stem, e.g. "costmap_geometry_mixin"'
        )
    mixin_path = os.path.join(
        os.path.dirname(os.path.abspath(template_path)), MIXINS_DIR_NAME,
        stem + '.yaml'
    )
    if not os.path.isfile(mixin_path):
        raise compile_error(
            f"Mixin '{stem}' included by '{template_path}' not found at "
            f"'{mixin_path}'"
        )
    return read_mixin_file(mixin_path)


def expand_mixins(parameters: dict, template_path: str,
                  mounts=None, used_stems=None, _path=()) -> dict:
    """Expand ``__include_mixins`` entries in a template's ros__parameters mapping.

    The value of an ``__include_mixins`` entry names one or more mixin file
    stems -- as a single stem, a comma-separated string
    (``costmap_geometry_mixin, costmap_publish_mixin``) or a yaml list -- each
    resolved to ``mixins/<stem>.yaml`` next to the including template file.
    Every mixin's ros__parameters content is merged into the mapping holding
    the ``__include_mixins`` entry; a key provided both by the mapping and a
    mixin (or by two mixins) is an error. The input mapping is modified in
    place and returned.

    When ``mounts`` (a dict) and ``used_stems`` (a list) are given, every
    mixin root *group* records its mount path and the mixin's C++ namespace
    (``mounts[path_tuple] = 'ns::tokens'``) and the contributing mixin stems
    are appended to ``used_stems`` in first-use order. The C++ generator uses
    these records to reference the shared mixin DTO structs instead of
    re-defining them inline.
    """
    stems = parameters.pop(INCLUDE_MIXIN_KEY, None)
    if stems is not None:
        if isinstance(stems, str):
            stems = [stem.strip() for stem in stems.split(',') if stem.strip()]
        if not isinstance(stems, list):
            raise compile_error(
                f"Invalid {INCLUDE_MIXIN_KEY} value {stems!r} in '{template_path}': "
                'expected a mixin file stem, a comma-separated string of stems '
                'or a yaml list of stems'
            )
        for stem in stems:
            mixin_parameters, mixin_namespace = _load_mixin_parameters(
                stem, template_path)
            for key, value in mixin_parameters.items():
                if key in parameters:
                    raise compile_error(
                        f"Mixin '{stem}' included by '{template_path}' defines "
                        f"'{key}', which already exists at the include location"
                    )
                parameters[key] = value
                if mounts is not None and is_parameter_group(value):
                    mounts[_path + (key,)] = '::'.join(
                        parse_code_namespace_tokens(mixin_namespace))
                    if used_stems is not None and stem not in used_stems:
                        used_stems.append(stem)
    for key, value in parameters.items():
        if isinstance(value, dict):
            expand_mixins(value, template_path, mounts, used_stems, _path + (key,))
    return parameters
