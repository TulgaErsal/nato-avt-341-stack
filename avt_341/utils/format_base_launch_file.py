#!/usr/bin/env python3
"""Reformat the NODES and DeclareLaunchArgument sections of base.launch.py.

Collapses every entry of the module-level NODES dict and every
DeclareLaunchArgument(...) inside generate_launch_description() onto a
single line and aligns the start of each field across entries, so both
sections read as a table. Comment and blank lines between entries are kept
in place; nothing outside the two sections is modified.

Before writing, the rewritten source is verified to parse and to be
semantically identical to the original (same AST up to keyword order).

Usage:
    python format_base_launch_file.py [launch_file]
    python format_base_launch_file.py --check    # exit 1 if a rewrite is needed
"""

import argparse
import ast
import io
import os
import sys
import tokenize

DEFAULT_LAUNCH_FILE = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'launch', 'base.launch.py'))

# Canonical column order per call; the leading names bind positional arguments.
NODESPEC_COLUMNS = ('executable', 'template', 'condition', 'sub_ns',
                    'extra_params', 'output', 'autonomy')
NODESPEC_POSITIONAL = ('executable', 'template')
DECLARE_COLUMNS = ('name', 'default_value', 'description', 'choices')
DECLARE_POSITIONAL = ('name',)


def _cell_source(src, node):
    """Source text of one argument, collapsed onto a single line."""
    if (isinstance(node, ast.Constant) and isinstance(node.value, str)
            and node.end_lineno > node.lineno):
        # implicitly concatenated string spanning lines -> one merged literal
        return repr(node.value)
    out = ''
    for line in ast.get_source_segment(src, node).splitlines():
        line = line.strip()
        if not line:
            continue
        if (not out or out.endswith(('(', '[', '{'))
                or line.startswith((')', ']', '}', ','))):
            out += line
        else:
            out += ' ' + line
    return out


def _call_fields(src, call, positional):
    """Map the arguments of a Call node to {field_name: single_line_source}."""
    if len(call.args) > len(positional):
        raise SystemExit(f'line {call.lineno}: too many positional arguments '
                         f'for {ast.get_source_segment(src, call.func)}(...)')
    fields = {}
    for name, arg in zip(positional, call.args):
        fields[name] = _cell_source(src, arg)
    for kw in call.keywords:
        if kw.arg is None:
            raise SystemExit(f'line {call.lineno}: cannot format **kwargs')
        fields[kw.arg] = f'{kw.arg}={_cell_source(src, kw.value)}'
    return fields


def _tabulate(rows, canonical, indent):
    """Render (head, call_name, fields) rows as aligned single-line entries."""
    columns = []
    for _, _, fields in rows:
        for name in fields:
            if name not in columns:
                columns.append(name)
    columns = ([c for c in canonical if c in columns]
               + [c for c in columns if c not in canonical])
    head_w = max(len(head) for head, _, _ in rows)
    call_w = max(len(name) for _, name, _ in rows) + 1  # includes '('
    widths = {c: 2 + max(len(fields[c]) for _, _, fields in rows if c in fields)
              for c in columns}
    lines = []
    for head, call_name, fields in rows:
        cells = []
        for c in columns:
            text = fields.get(c, '')
            if text:
                text += ','
            cells.append(text.ljust(widths[c]))
        body = ''.join(cells).rstrip()[:-1]  # drop the trailing comma
        prefix = indent + (head.ljust(head_w) + ' ' if head_w else '')
        lines.append(f"{prefix}{(call_name + '(').ljust(call_w)}{body}),")
    return lines


def _guard_no_comments(comment_lines, start, end):
    lost = sorted(set(range(start, end + 1)) & comment_lines)
    if lost:
        raise SystemExit(f'line {lost[0]}: comment inside an entry would be '
                         f'lost; move it onto its own line above the entry')


def _rebuild_lines(src_lines, first, last, replacements):
    """Body lines first..last (1-based, inclusive) with entry spans replaced."""
    repl = {start: (end, new) for start, end, new in replacements}
    out = []
    ln = first
    while ln <= last:
        if ln in repl:
            end, new = repl[ln]
            out.extend(new)
            ln = end + 1
        else:
            text = src_lines[ln - 1]
            if text.strip() != ',':  # orphaned trailing comma of an old entry
                out.append(text)
            ln += 1
    return out


def _nodes_section(src, src_lines, tree, comment_lines):
    assign = next(
        (n for n in tree.body if isinstance(n, ast.Assign)
         and any(isinstance(t, ast.Name) and t.id == 'NODES' for t in n.targets)),
        None)
    if assign is None or not isinstance(assign.value, ast.Dict):
        raise SystemExit('module-level NODES dict not found')
    d = assign.value
    rows, spans = [], []
    for key, value in zip(d.keys, d.values):
        if (key is None or not isinstance(value, ast.Call)
                or key.lineno <= d.lineno or value.end_lineno >= d.end_lineno):
            raise SystemExit(f'line {d.lineno}: unsupported NODES entry layout')
        _guard_no_comments(comment_lines, key.lineno, value.end_lineno)
        rows.append((ast.get_source_segment(src, key) + ':',
                     ast.get_source_segment(src, value.func),
                     _call_fields(src, value, NODESPEC_POSITIONAL)))
        spans.append((key.lineno, value.end_lineno))
    if not rows:
        raise SystemExit('NODES dict is empty')
    indent = ' ' * d.keys[0].col_offset
    lines = _tabulate(rows, NODESPEC_COLUMNS, indent)
    replacements = [(s, e, [line]) for (s, e), line in zip(spans, lines)]
    body = _rebuild_lines(src_lines, d.lineno + 1, d.end_lineno - 1, replacements)
    return (d.lineno + 1, d.end_lineno - 1, body), len(rows)


def _declare_args_section(src, src_lines, tree, comment_lines):
    func = next(
        (n for n in tree.body
         if isinstance(n, ast.FunctionDef) and n.name == 'generate_launch_description'),
        None)
    if func is None:
        raise SystemExit('generate_launch_description() not found')
    lst = next(
        (n.args[0] for n in ast.walk(func)
         if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
         and n.func.id == 'LaunchDescription'
         and n.args and isinstance(n.args[0], ast.List)),
        None)
    if lst is None:
        raise SystemExit('LaunchDescription([...]) list not found')
    rows, spans = [], []  # spans: (start, end, row index or None to keep as-is)
    for elt in lst.elts:
        if elt.lineno <= lst.lineno or elt.end_lineno >= lst.end_lineno:
            raise SystemExit(f'line {elt.lineno}: unsupported list entry layout')
        if (isinstance(elt, ast.Call) and isinstance(elt.func, ast.Name)
                and elt.func.id == 'DeclareLaunchArgument'):
            _guard_no_comments(comment_lines, elt.lineno, elt.end_lineno)
            rows.append(('', elt.func.id, _call_fields(src, elt, DECLARE_POSITIONAL)))
            spans.append((elt.lineno, elt.end_lineno, len(rows) - 1))
        else:
            spans.append((elt.lineno, elt.end_lineno, None))
    if not rows:
        raise SystemExit('no DeclareLaunchArgument entries found')
    indent = ' ' * lst.elts[0].col_offset
    lines = _tabulate(rows, DECLARE_COLUMNS, indent)
    replacements = [
        (s, e, src_lines[s - 1:e] if idx is None else [lines[idx]])
        for s, e, idx in spans]
    body = _rebuild_lines(src_lines, lst.lineno + 1, lst.end_lineno - 1, replacements)
    return (lst.lineno + 1, lst.end_lineno - 1, body), len(rows)


def _splice(src_lines, sections):
    out, ln = [], 1
    for start, end, new in sorted(sections):
        out.extend(src_lines[ln - 1:start - 1])
        out.extend(new)
        ln = end + 1
    out.extend(src_lines[ln - 1:])
    return out


class _SortKeywords(ast.NodeTransformer):
    def visit_Call(self, node):
        self.generic_visit(node)
        node.keywords.sort(key=lambda kw: kw.arg or '')
        return node


def _semantic_dump(source):
    return ast.dump(_SortKeywords().visit(ast.parse(source)))


def format_file(path, check=False):
    with open(path, encoding='utf-8', newline='') as f:
        raw = f.read()
    newline = '\r\n' if '\r\n' in raw else '\n'
    src = raw.replace('\r\n', '\n')
    src_lines = src.split('\n')
    tree = ast.parse(src, filename=path)
    comment_lines = {
        tok.start[0]
        for tok in tokenize.generate_tokens(io.StringIO(src).readline)
        if tok.type == tokenize.COMMENT}
    nodes_section, node_count = _nodes_section(src, src_lines, tree, comment_lines)
    args_section, arg_count = _declare_args_section(src, src_lines, tree, comment_lines)
    new_src = '\n'.join(_splice(src_lines, [nodes_section, args_section]))
    try:
        ast.parse(new_src)
    except SyntaxError as exc:
        raise SystemExit(f'internal error: reformatted file does not parse: {exc}')
    if _semantic_dump(src) != _semantic_dump(new_src):
        raise SystemExit('internal error: reformatting would change the file '
                         'semantics; aborting')
    if new_src == src:
        print(f'{path}: already formatted ({node_count} NODES entries, '
              f'{arg_count} DeclareLaunchArgument entries)')
        return False
    if not check:
        with open(path, 'w', encoding='utf-8', newline='') as f:
            f.write(new_src if newline == '\n' else new_src.replace('\n', newline))
    print(f"{'would reformat' if check else 'reformatted'} {path}: aligned "
          f'{node_count} NODES entries and {arg_count} DeclareLaunchArgument entries')
    return True


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('launch_file', nargs='?', default=DEFAULT_LAUNCH_FILE,
                        help='launch file to reformat (default: %(default)s)')
    parser.add_argument('--check', action='store_true',
                        help='only report whether a rewrite is needed; '
                             'exit code 1 if the file would change')
    args = parser.parse_args(argv)
    changed = format_file(args.launch_file, check=args.check)
    return 1 if (args.check and changed) else 0


if __name__ == '__main__':
    sys.exit(main())
