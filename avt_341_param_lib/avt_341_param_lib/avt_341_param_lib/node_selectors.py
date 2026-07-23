"""ROS 2-style node-selector matching shared by launch configuration helpers."""

from typing import List


def selector_matches(selector: str, fqn: str) -> bool:
    """Return whether a node selector matches a fully qualified node name.

    A selector is a slash-delimited path where the token ``**`` matches any
    number of tokens (including none), ``*`` matches exactly one token and any
    other token matches literally.
    """
    return _tokens_match(
        [token for token in selector.split('/') if token],
        [token for token in fqn.split('/') if token],
    )


def _tokens_match(pattern: List[str], tokens: List[str]) -> bool:
    if not pattern:
        return not tokens
    if pattern[0] == '**':
        return _tokens_match(pattern[1:], tokens) or (
            bool(tokens) and _tokens_match(pattern, tokens[1:]))
    return bool(tokens) and pattern[0] in ('*', tokens[0]) and _tokens_match(
        pattern[1:], tokens[1:])
