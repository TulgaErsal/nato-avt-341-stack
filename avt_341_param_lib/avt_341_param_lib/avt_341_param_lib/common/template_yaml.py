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

from typing import Dict, List, NamedTuple, Tuple
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
#
# __inherit_mixins names mixins the same way but is only allowed directly under
# ros__parameters. Its parameters splice in identically -- so the declared ROS
# parameter names, the launch arguments and the generated documentation are
# exactly those of a mixin written inline at the template root -- but the
# generated C++ class derives from the mixin's own class instead of re-declaring
# the members. See expand_mixins() for the bookkeeping this needs.
INCLUDE_MIXIN_KEY = '__include_mixins'
INHERIT_MIXIN_KEY = '__inherit_mixins'
MIXIN_KEYS = (INCLUDE_MIXIN_KEY, INHERIT_MIXIN_KEY)
MIXINS_DIR_NAME = 'mixins'


class MixinDefinition(NamedTuple):
    """One mixin template file as read from disk."""

    parameters: dict
    code_namespace: str
    class_name: str


class InheritedMixin(NamedTuple):
    """A mixin whose class becomes a base class of the template's class."""

    stem: str
    namespace: str  # '::'-joined C++ namespace, e.g. avt_341_nav::params::core
    class_name: str
    root_keys: Tuple[str, ...]  # keys this mixin spliced into ros__parameters

    @property
    def qualified_name(self) -> str:
        return f'{self.namespace}::{self.class_name}'


class MixinUsage:
    """Build-time record of how one template used mixins.

    Only the code generator needs this. Launch-time callers pass nothing: the
    flattened parameter set is identical with or without the bookkeeping.
    """

    def __init__(self):
        # path tuple below ros__parameters -> C++ namespace of an included group
        self.mounts: Dict[Tuple[str, ...], str] = {}
        # stems whose DTO header must be #included, in first-use order
        self.include_stems: List[str] = []
        # inherited mixins, in declaration order
        self.inherited: List[InheritedMixin] = []
        # stem -> qualified class name, for every referenced mixin
        self.classes: Dict[str, str] = {}

    def base_classes(self) -> str:
        """The C++ base-clause body, e.g. 'public a::A, public b::B'."""
        return ', '.join(
            f'public {mixin.qualified_name}' for mixin in self.inherited)

    def inherited_root_keys(self) -> set:
        return {key for mixin in self.inherited for key in mixin.root_keys}


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


@typechecked
def parse_class_name(class_name, source: str) -> str:
    """Validate a class_name root element value.

    Shared by template files and mixin files so both report the same error.
    """
    if not isinstance(class_name, str) or not re.match(
        r'^[A-Za-z_][A-Za-z0-9_]*$', str(class_name)
    ):
        raise compile_error(
            f"Invalid {CLASS_NAME_ROOT_KEY} '{class_name}' in '{source}'. "
            'Expected a valid class identifier, e.g. "Params"'
        )
    return class_name


def _find_mixin_entry(node):
    """The first mixin key found anywhere in ``node``, or None."""
    if not isinstance(node, dict):
        return None
    for key in MIXIN_KEYS:
        if key in node:
            return key
    for value in node.values():
        found = _find_mixin_entry(value)
        if found is not None:
            return found
    return None


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


def read_mixin_file(mixin_path: str) -> MixinDefinition:
    """Load and validate one mixin template file.

    Returns the mixin's ros__parameters mapping together with the
    ``code_namespace`` and ``class_name`` naming the class that
    ``__inherit_mixins`` derives from.
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
    class_name = parse_class_name(
        doc.get(CLASS_NAME_ROOT_KEY, DEFAULT_CLASS_NAME), mixin_path)
    parameters = doc[PARAMETERS_ROOT_KEY]
    nested_mixin_key = _find_mixin_entry(parameters)
    if nested_mixin_key is not None:
        raise compile_error(
            f"Mixin file '{mixin_path}' holds a nested {nested_mixin_key} entry; "
            'mixins cannot include or inherit other mixins'
        )
    if _contains_mapped_parameter(parameters):
        raise compile_error(
            f"Mixin file '{mixin_path}' holds a dynamically mapped (__map_) "
            'parameter; mapped parameters are not supported in mixins'
        )
    return MixinDefinition(parameters, code_namespace, class_name)


def _load_mixin_parameters(stem, template_path: str,
                           key: str = INCLUDE_MIXIN_KEY) -> MixinDefinition:
    """Resolve and load the mixin referenced by ``stem``."""
    if not isinstance(stem, str) or not stem:
        raise compile_error(
            f"Invalid {key} value {stem!r} in '{template_path}': "
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


def _mixin_stems(value, key: str, template_path: str) -> List[str]:
    """Normalise a mixin key's value into a list of file stems.

    Accepts a single stem, a comma-separated string or a yaml list.
    """
    if value is None:
        return []
    if isinstance(value, str):
        return [stem.strip() for stem in value.split(',') if stem.strip()]
    if not isinstance(value, list):
        raise compile_error(
            f"Invalid {key} value {value!r} in '{template_path}': "
            'expected a mixin file stem, a comma-separated string of stems '
            'or a yaml list of stems'
        )
    return value


def _splice(mixin_parameters: dict, parameters: dict, stem: str,
            template_path: str) -> List[str]:
    """Copy a mixin's parameters into ``parameters``, returning the keys added.

    A key supplied both by the including mapping and a mixin (or by two
    mixins) is an error. For inherited mixins this is also what keeps a
    derived member from silently shadowing a base member.
    """
    added = []
    for key, value in mixin_parameters.items():
        if key in parameters:
            raise compile_error(
                f"Mixin '{stem}' included by '{template_path}' defines "
                f"'{key}', which already exists at the include location"
            )
        parameters[key] = value
        added.append(key)
    return added


def expand_mixins(parameters: dict, template_path: str,
                  usage: MixinUsage = None, _path=()) -> dict:
    """Expand the mixin entries in a template's ros__parameters mapping.

    The value of ``__include_mixins`` or ``__inherit_mixins`` names one or more
    mixin file stems -- as a single stem, a comma-separated string
    (``costmap_geometry_mixin, costmap_publish_mixin``) or a yaml list -- each
    resolved to ``mixins/<stem>.yaml`` next to the including template file.
    Every mixin's ros__parameters content is merged into the mapping holding
    the entry; a key provided both by the mapping and a mixin (or by two
    mixins) is an error. The input mapping is modified in place and returned.

    ``__include_mixins`` may appear at any level; ``__inherit_mixins`` only
    directly under ros__parameters, because inherited members sit at the root
    of the generated class and so carry no parameter-name prefix.

    Both keys splice identically, so callers that only want the flattened
    parameter set -- launch-time argument declaration, documentation -- can
    ignore the distinction entirely and omit ``usage``. When a ``MixinUsage``
    is given it records what the C++ generator needs on top of the splice:
    which groups an include mounted (and under which namespace), which mixin
    headers to ``#include``, and which mixins became base classes.
    """
    inherit_stems = _mixin_stems(
        parameters.pop(INHERIT_MIXIN_KEY, None), INHERIT_MIXIN_KEY,
        template_path)
    if inherit_stems and _path:
        raise compile_error(
            f"{INHERIT_MIXIN_KEY} entry under '{'.'.join(_path)}' in "
            f"'{template_path}': inherited mixins are only allowed directly "
            f'under {PARAMETERS_ROOT_KEY}. Inherited parameters become members '
            'of the generated class itself, so they carry no name prefix and '
            f'cannot be mounted under a group -- use {INCLUDE_MIXIN_KEY} to '
            'nest a mixin.'
        )

    for stem in inherit_stems:
        mixin = _load_mixin_parameters(stem, template_path, INHERIT_MIXIN_KEY)
        root_keys = _splice(mixin.parameters, parameters, stem, template_path)
        if usage is not None:
            namespace = '::'.join(
                parse_code_namespace_tokens(mixin.code_namespace))
            usage.inherited.append(
                InheritedMixin(stem, namespace, mixin.class_name,
                               tuple(root_keys)))
            usage.classes[stem] = f'{namespace}::{mixin.class_name}'
            # the base type is defined in the mixin's own DTO header
            if stem not in usage.include_stems:
                usage.include_stems.append(stem)

    include_stems = _mixin_stems(
        parameters.pop(INCLUDE_MIXIN_KEY, None), INCLUDE_MIXIN_KEY,
        template_path)
    for stem in include_stems:
        if stem in inherit_stems:
            raise compile_error(
                f"Mixin '{stem}' is both inherited and included by "
                f"'{template_path}'; use one or the other"
            )
        mixin = _load_mixin_parameters(stem, template_path, INCLUDE_MIXIN_KEY)
        namespace = ('::'.join(parse_code_namespace_tokens(mixin.code_namespace))
                     if usage is not None else None)
        for key in _splice(mixin.parameters, parameters, stem, template_path):
            # only groups become a shared struct the including template can
            # reference; root-level leaves splice in as plain fields and pull
            # in no header of their own
            if usage is not None and is_parameter_group(parameters[key]):
                usage.mounts[_path + (key,)] = namespace
                if stem not in usage.include_stems:
                    usage.include_stems.append(stem)
        if usage is not None:
            usage.classes.setdefault(stem, f'{namespace}::{mixin.class_name}')

    for key, value in parameters.items():
        if isinstance(value, dict):
            expand_mixins(value, template_path, usage, _path + (key,))
    return parameters
