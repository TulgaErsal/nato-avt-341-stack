"""ROS 2-style node-selector matching shared by launch configuration helpers."""

import re
from functools import lru_cache
from typing import List, Optional, Tuple, Union

# A compiled selector token: a literal name, '*', '**' or a regular expression.
Token = Union[str, 're.Pattern']


def is_regex_token(token: str) -> bool:
    """Whether a selector token is regex-shaped (wrapped in parentheses)."""
    return len(token) >= 2 and token.startswith('(') and token.endswith(')')


def compile_token(token: str) -> Token:
    """Compile one selector token; regex-shaped tokens become patterns.

    The surrounding parentheses are compiled as part of the pattern (they are
    an ordinary regex group). Raises ValueError for an invalid regex.
    """
    if not is_regex_token(token):
        return token
    try:
        return re.compile(token)
    except re.error as error:
        raise ValueError(
            f"Invalid regular expression token '{token}': {error}") from error


@lru_cache(maxsize=None)
def compile_selector(selector: str) -> Tuple[Token, ...]:
    """Split a selector on ``/`` and compile each non-empty token."""
    return tuple(
        compile_token(token) for token in selector.split('/') if token)


def token_matches(token: Token, text: str) -> bool:
    """Whether one compiled selector token matches one name segment."""
    if isinstance(token, re.Pattern):
        return token.fullmatch(text) is not None
    return token == '*' or token == text


def selector_matches(selector: str, fqn: str) -> bool:
    """Return whether a node selector matches a fully qualified node name.

    A selector is a slash-delimited path where the token ``**`` matches any
    number of tokens (including none), ``*`` matches exactly one token, a
    parenthesized token is a regular expression fully matching exactly one
    token (``/`` can therefore never occur inside a regex) and any other
    token matches literally.
    """
    return _tokens_match(
        compile_selector(selector),
        [token for token in fqn.split('/') if token],
    )


def _tokens_match(pattern: Tuple[Token, ...], tokens: List[str]) -> bool:
    if not pattern:
        return not tokens
    if pattern[0] == '**':
        return _tokens_match(pattern[1:], tokens) or (
            bool(tokens) and _tokens_match(pattern, tokens[1:]))
    return bool(tokens) and token_matches(pattern[0], tokens[0]) and _tokens_match(
        pattern[1:], tokens[1:])


def validate_selector_token(token: str) -> Optional[str]:
    """Validate one selector token; None when valid, else a problem description.

    Stricter than the matcher itself: tokens holding unbalanced parentheses
    or partial ``*`` wildcards are matched leniently (as literals that can
    never equal a real ROS name) by :func:`selector_matches`, but are almost
    certainly authoring mistakes, so surfaces that accept user-written
    selectors report them via this function.
    """
    if token in ('', '*', '**'):
        return None
    if is_regex_token(token):
        try:
            re.compile(token)
        except re.error as error:
            return f"Invalid regular expression token '{token}': {error}"
        return None
    if '(' in token or ')' in token:
        return (
            f"Unbalanced parentheses in token '{token}': a regular-expression "
            "token must start with '(' and end with ')'"
        )
    if '*' in token:
        return (
            f"Unsupported wildcard token '{token}'; only whole-token '*' and "
            "'**' wildcards and parenthesized '(regex)' tokens are supported"
        )
    return None
