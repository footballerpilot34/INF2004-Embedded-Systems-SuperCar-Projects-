"""Audit project-owned C; findings are not a BARR-C compliance certificate.

Install tools/barr-requirements.txt for the syntax-tree review. The standard's
semantic requirements still need human review and target measurements.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
EXCLUDED = {'.git', '.codex', '.agents', 'dhcpserver', 'node_modules',
            '.venv', 'venv', 'pico-sdk', 'external', 'third_party'}
FILES = sorted(path for path in ROOT.rglob('*')
               if path.suffix in {'.c', '.h'} and path.is_file()
               and not any(part in EXCLUDED or part.startswith('build')
                           for part in path.relative_to(ROOT).parts))



def audit(syntax=False):
    findings = []

    def add(path, line, rule, message):
        findings.append(dict(file=str(path.relative_to(ROOT)), line=line,
                             rule=rule, message=message))

    if syntax:
        from tree_sitter import Language, Parser
        import tree_sitter_c
        parser = Parser(Language(tree_sitter_c.language()))

    def walk(node):
        yield node
        for child in node.children:
            yield from walk(child)

    for path in FILES:
        data = path.read_bytes()
        text = data.decode()
        for index, line in enumerate(text.splitlines(), 1):
            if len(line) > 80:
                add(path, index, '1.2.a', 'Line exceeds 80 characters.')
            if '\t' in line:
                add(path, index, '3.5.a', 'Literal tab character.')
            if any(ord(c) < 32 and c != '\f' for c in line):
                add(path, index, '3.6.b', 'Nonprinting character.')
        if b'\r' in data:
            add(path, 1, '3.6.a', 'CR line ending.')
        if '@file' not in text[:1800]:
            add(path, 1, '2.2.h', 'Module Doxygen file comment missing.')
        if path.suffix == '.h' and not re.search(r'#ifndef\s+\w+', text):
            add(path, 1, '4.2.b', 'Include guard missing.')
        if not syntax:
            continue
        root = parser.parse(data).root_node
        for node in walk(root):
            row = node.start_point.row + 1
            kind = node.type
            raw = data[node.start_byte:node.end_byte].decode()
            if kind == 'ERROR' or node.is_missing:
                add(path, row, 'parse', 'Parser limitation or syntax error; verify compiler.')
            if kind in ['if_statement', 'for_statement', 'while_statement', 'do_statement']:
                key = 'consequence' if kind == 'if_statement' else 'body'
                body = node.child_by_field_name(key)
                if body and body.type != 'compound_statement':
                    add(path, row, '1.3.a', 'Control body lacks mandatory braces.')
            if kind == 'function_definition':
                if node.end_point.row - node.start_point.row + 1 > 100:
                    add(path, row, '6.2.a', 'Function exceeds 100 lines.')
                prev = node.prev_named_sibling
                if not prev or prev.type != 'comment' or b'@brief' not in prev.text:
                    add(path, row, '2.2.h', 'Function Doxygen brief missing.')
            if kind in ['declaration', 'field_declaration']:
                if len(node.children_by_field_name('declarator')) > 1:
                    add(path, row, '8.1.a', 'Multiple declarators retained.')
            if kind == 'function_definition':
                declarator = node.child_by_field_name('declarator')
                while declarator and declarator.type != 'function_declarator':
                    declarator = declarator.child_by_field_name('declarator')
                if declarator:
                    name = declarator.child_by_field_name('declarator').text.decode()
                    if len(name) > 31 or name.startswith('_') or name.lower() != name:
                        add(path, row, '6.1', 'Function naming deviation: ' + name)
            if kind in ['primitive_type', 'sized_type_specifier'] and raw in ['float', 'double']:
                add(path, row, '5.4.b.i', 'Native floating type; retain tested arithmetic ABI.')
            if kind == 'cast_expression':
                add(path, row, '1.6.a', 'Cast range/comment requires manual review.')
            if kind in ['declaration', 'parameter_declaration', 'field_declaration']:
                for declarator in node.children_by_field_name('declarator'):
                    target = declarator
                    if target.type == 'init_declarator':
                        target = target.child_by_field_name('declarator')
                    while target and target.type not in ['identifier', 'field_identifier']:
                        target = target.child_by_field_name('declarator')
                    if target:
                        name = target.text.decode()
                        if len(name) < 3 or len(name) > 31 or name.startswith('_') or name.lower() != name:
                            add(path, row, '7.1', 'Variable naming deviation: ' + name)
            if kind == 'binary_expression':
                op = node.child_by_field_name('operator')
                if op and op.text in [b'&&', b'||']:
                    for key in ['left', 'right']:
                        child = node.child_by_field_name(key)
                        if child.type not in ['identifier', 'number_literal', 'parenthesized_expression']:
                            add(path, row, '1.4.b', 'Logical operand lacks explicit parentheses.')
            if kind in ['goto_statement', 'continue_statement']:
                add(path, row, '1.7', 'Review preferred avoidance of goto/continue.')
            if kind == 'sized_type_specifier' and re.search(r'\b(short|long)\b', raw):
                add(path, row, '5.2.b', 'Native-width integer retained; review ABI/format cast.')
            if kind == 'switch_statement' and not any(
                    c.type == 'case_statement' and c.text.startswith(b'default') for c in walk(node)):
                add(path, row, '8.3.b', 'Switch has no default block.')
    return findings


if __name__ == '__main__':
    args = argparse.ArgumentParser()
    args.add_argument('--syntax', action='store_true')
    args.add_argument('--report', type=Path)
    opts = args.parse_args()
    result = audit(opts.syntax)
    if opts.report:
        opts.report.write_text(json.dumps(result, indent=2) + '\n')
    print(f'{len(FILES)} project-owned files reviewed; {len(result)} findings.')
    print(dict(sorted(Counter(item['rule'] for item in result).items())))
    # Formatting gates are strict. Other findings remain visible for review.
    raise SystemExit(any(x['rule'] in ['1.2.a', '3.5.a', '3.6.a', '3.6.b',
                                      '1.3.a', '1.4.b', '4.2.b'] for x in result))
