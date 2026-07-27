"""Reusable custom launch ``Substitution`` and ``Condition`` classes shared by launch files."""

from launch.condition import Condition
from launch.launch_context import LaunchContext
from launch.substitution import Substitution
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions


class TernarySubstitution(Substitution):
    """Substitution resolving to true_val if the given condition holds, else false_val."""

    def __init__(self, true_val: SomeSubstitutionsType, false_val: SomeSubstitutionsType, condition: Condition):
        self.__true_val = true_val
        self.__false_val = false_val
        self.__condition = condition

    def describe(self):
        return 'TernarySubstitution(%s %s %s)' % (self.__true_val.describe(), self.__false_val.describe(), self.__condition.describe())

    def perform(self, context: LaunchContext):
        if self.__condition.evaluate(context):
            return self.__true_val.perform(context)
        else:
            return self.__false_val.perform(context)


class ToUpper(Substitution):
    """Substitution resolving to the wrapped substitution's value in uppercase."""

    def __init__(self, sub_val: SomeSubstitutionsType):
        self.__sub_val = sub_val

    def describe(self):
        return 'ToUpper(%s)' % (self.__sub_val.describe())

    def perform(self, context: LaunchContext):
        return self.__sub_val.perform(context).upper()


class Invert(Substitution):
    """Substitution resolving to the logical negation of the wrapped boolean-like value."""

    def __init__(self, sub_val: SomeSubstitutionsType):
        self.__sub_val = sub_val

    def describe(self):
        return 'Invert(%s)' % (self.__sub_val.describe())

    def perform(self, context: LaunchContext):
        val = self.__sub_val.perform(context).lower()
        is_true = val in ['true', '1']
        return str(not is_true)


class Concat(Substitution):
    """Substitution resolving to the wrapped substitution's value with a literal appended."""

    def __init__(self, sub_val: SomeSubstitutionsType, concat_val):
        self.__sub_val = sub_val
        self.__concat_val = concat_val

    def describe(self):
        return 'StringConcat(%s %s)' % (self.__sub_val.describe(), self.__concat_val)

    def perform(self, context: LaunchContext):
        return self.__sub_val.perform(context) + self.__concat_val


class ArrayIndexSubstitution(Substitution):
    """Substitution resolving to the element at the given index of a string-encoded list."""

    def __init__(self, sub_val: SomeSubstitutionsType, idx: int):
        self.__sub_val = sub_val
        self.__idx = idx

    def describe(self):
        return 'ArrayIndexSubstitution(%s %d)' % (self.__sub_val.describe(), self.__idx)

    def perform(self, context: LaunchContext):
        array_val = self.__sub_val.perform(context)
        # array_val is currently a string, need to parse
        array_val = array_val.replace('[', '', 1)[::-1].replace(']', '', 1)[::-1].replace(' ', '').replace("'", "").split(',')
        return array_val[self.__idx]


class InListCondition(Condition):
    """Condition holding when the substitution's value is one of the expected values."""

    def __init__(self, sub_val: SomeSubstitutionsType, expected_values):
        self.__sub_val = sub_val
        self.__expected_values = None
        if expected_values is not None:
            self.__expected_values = normalize_to_list_of_substitutions(expected_values)
        super().__init__(predicate=self._predicate_func)

    def _predicate_func(self, context):
        value = self.__sub_val.perform(context)
        expanded_expected_value = perform_substitutions(context, self.__expected_values)
        return value in expanded_expected_value

    def describe(self):
        return self.__repr__()


class NotInListCondition(InListCondition):
    """Condition holding when the substitution's value is not one of the expected values."""

    def __init__(self, sub_val: SomeSubstitutionsType, expected_values):
        super().__init__(sub_val, expected_values)

    def _predicate_func(self, context):
        return not super()._predicate_func(context)


class AnyCondition(Condition):
    """Condition holding when any of the given conditions holds."""

    def __init__(self, conditions):
        self.__conditions = conditions
        super().__init__(predicate=self._predicate_func)

    def _predicate_func(self, context):
        return any([c._predicate_func(context) for c in self.__conditions])

    def describe(self):
        return f"AnyCondition({','.join([d.describe() for d in self.__conditions])})"


class AllCondition(Condition):
    """Condition holding when all of the given conditions hold."""

    def __init__(self, conditions):
        self.__conditions = conditions
        super().__init__(predicate=self._predicate_func)

    def _predicate_func(self, context):
        return all([c._predicate_func(context) for c in self.__conditions])

    def describe(self):
        return f"AllCondition({','.join([d.describe() for d in self.__conditions])})"


class ListSize(Substitution):
    """Substitution resolving to the element count of a string-encoded list."""

    def __init__(self, sub_val: SomeSubstitutionsType):
        self.__sub_val = sub_val

    def describe(self):
        return 'ListSize %s' % (self.__sub_val.describe())

    def perform(self, context: LaunchContext):
        array_val = self.__sub_val.perform(context)
        array_val = array_val.replace('[', '', 1)[::-1].replace(']', '', 1)[::-1].replace(' ', '').replace("'", "").split(',')
        return str(len(array_val))