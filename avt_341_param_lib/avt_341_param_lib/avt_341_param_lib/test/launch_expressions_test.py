"""Tests for the reusable launch Substitution and Condition classes.

The module previously carried an annotation referring to an unimported name,
which made it raise ``NameError`` on import -- undetected because nothing in the
stack imported it. Importing every public class here keeps that class of
regression visible.
"""

import pytest
from launch import LaunchContext
from launch.conditions import IfCondition
from launch.substitutions import TextSubstitution

from avt_341_param_lib.runtime.launch_expressions import (
    AllCondition,
    AnyCondition,
    ArrayIndexSubstitution,
    Concat,
    InListCondition,
    Invert,
    ListSize,
    NotInListCondition,
    TernarySubstitution,
    ToUpper,
)


@pytest.fixture
def context():
    return LaunchContext()


def text(value):
    return TextSubstitution(text=value)


def always(value: bool):
    return IfCondition(text('true' if value else 'false'))


LIST_TEXT = "['a', 'b', 'c']"


@pytest.mark.parametrize('condition, expected', [(True, 'xbox'), (False, 'can')])
def test_ternary_substitution_selects_by_condition(context, condition, expected):
    substitution = TernarySubstitution(
        text('xbox'), text('can'), always(condition))
    assert substitution.perform(context) == expected


def test_to_upper(context):
    assert ToUpper(text('mrzr')).perform(context) == 'MRZR'


@pytest.mark.parametrize(
    'value, expected',
    [('true', 'False'), ('1', 'False'), ('false', 'True'), ('anything', 'True')],
)
def test_invert_negates_boolean_like_values(context, value, expected):
    assert Invert(text(value)).perform(context) == expected


def test_concat_appends_a_literal(context):
    assert Concat(text('mrzr'), '/base_link').perform(context) == 'mrzr/base_link'


@pytest.mark.parametrize('index, expected', [(0, 'a'), (1, 'b'), (2, 'c'), (-1, 'c')])
def test_array_index_substitution(context, index, expected):
    assert ArrayIndexSubstitution(text(LIST_TEXT), index).perform(context) == expected


@pytest.mark.parametrize(
    'value, expected', [(LIST_TEXT, '3'), ("['only']", '1'), ('[]', '1')])
def test_list_size(context, value, expected):
    # note: an empty list still reports 1, since the string is split on commas
    assert ListSize(text(value)).perform(context) == expected


@pytest.mark.parametrize('value, expected', [('b', True), ('z', False)])
def test_in_list_condition(context, value, expected):
    assert InListCondition(text(value), text('abc')).evaluate(context) is expected


@pytest.mark.parametrize('value, expected', [('b', False), ('z', True)])
def test_not_in_list_condition(context, value, expected):
    assert NotInListCondition(text(value), text('abc')).evaluate(context) is expected


@pytest.mark.parametrize(
    'flags, any_expected, all_expected',
    [
        ((True, True), True, True),
        ((True, False), True, False),
        ((False, False), False, False),
    ],
)
def test_any_and_all_conditions(context, flags, any_expected, all_expected):
    conditions = [always(flag) for flag in flags]
    assert AnyCondition(conditions).evaluate(context) is any_expected
    assert AllCondition(conditions).evaluate(context) is all_expected


def test_describe_does_not_raise(context):
    # `describe` is what launch calls when reporting a substitution, so a broken
    # implementation only surfaces during error reporting
    for substitution in (
        TernarySubstitution(text('a'), text('b'), always(True)),
        ToUpper(text('a')),
        Invert(text('true')),
        Concat(text('a'), 'b'),
        ArrayIndexSubstitution(text(LIST_TEXT), 0),
        ListSize(text(LIST_TEXT)),
    ):
        assert isinstance(substitution.describe(), str)

    for condition in (
        AnyCondition([always(True)]),
        AllCondition([always(True)]),
        InListCondition(text('a'), text('abc')),
    ):
        assert isinstance(condition.describe(), str)
