#!/usr/bin/env python3
"""RAM report for the SuperCar firmware (change requirement 1).

Reads a finished build and prints:
  1. the whole image's RAM sections (from supercar.elf), and
  2. RAM (.data + .bss) and code (.text) per application module, and
  3. (with --symbols) every application variable and its size.

Usage, from the repository root, after building:
    python tools/ram_report.py [build-dir] [--symbols] [--csv FILE]

It needs arm-none-eabi-size and arm-none-eabi-nm on the PATH (they come
with the Arm toolchain used for the build).
"""

import csv
import subprocess
import sys
from pathlib import Path

RAM_SECTIONS = ('.ram_vector_table', '.uninitialized_data', '.data', '.bss',
                '.heap', '.stack_dummy', '.scratch_x', '.scratch_y')


def sections(path):
    """Return {section: size} for one ELF or object file."""
    out = subprocess.run(['arm-none-eabi-size', '-A', str(path)],
                         capture_output=True, text=True, check=True).stdout
    sizes = {}
    for line in out.splitlines():
        parts = line.split()
        if (len(parts) >= 2) and parts[0].startswith('.') and \
                parts[1].isdigit():
            sizes[parts[0]] = sizes.get(parts[0], 0) + int(parts[1])
    return sizes


def module_sizes(path):
    """Return (ram, text) for one object file; .data and .bss sub-sections
    (-fdata-sections) are added up."""
    ram = 0
    text = 0
    for name, size in sections(path).items():
        if name.startswith('.data') or name.startswith('.bss'):
            ram += size
        elif name.startswith('.text') or name.startswith('.rodata'):
            text += size
    return ram, text


def symbols(path):
    """Return [(name, size, kind)] for the RAM variables in one object."""
    out = subprocess.run(['arm-none-eabi-nm', '-S', '--size-sort', str(path)],
                         capture_output=True, text=True, check=True).stdout
    rows = []
    for line in out.splitlines():
        parts = line.split()
        if (len(parts) == 4) and (parts[2] in 'bBdD'):
            rows.append((parts[3], int(parts[1], 16),
                         'bss' if parts[2] in 'bB' else 'data'))
    return rows


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    build = Path(args[0] if args else 'build')
    show_symbols = '--symbols' in sys.argv
    csv_path = None
    if '--csv' in sys.argv:
        csv_path = sys.argv[sys.argv.index('--csv') + 1]

    elf = build / 'supercar.elf'
    whole = sections(elf)
    print(f'Image {elf}')
    for name in RAM_SECTIONS:
        if name in whole:
            print(f'  {name:<22}{whole[name]:>8} B')
    static_ram = whole.get('.data', 0) + whole.get('.bss', 0)
    print(f'  {"static RAM (.data+.bss)":<22}{static_ram:>8} B')
    print()

    objs = sorted((build / 'CMakeFiles' / 'supercar.dir' / 'app_program')
                  .rglob('*.obj'))
    rows = []
    for obj in objs:
        name = obj.name.replace('.c.obj', '')
        ram, text = module_sizes(obj)
        rows.append((name, ram, text))
    print(f'{"module":<14}{"RAM B":>8}{"code B":>9}')
    for name, ram, text in rows:
        print(f'{name:<14}{ram:>8}{text:>9}')
    print(f'{"total":<14}{sum(r[1] for r in rows):>8}'
          f'{sum(r[2] for r in rows):>9}')

    if csv_path:
        with open(csv_path, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(['module', 'ram_bytes', 'code_bytes'])
            writer.writerows(rows)

    if show_symbols:
        print()
        for obj in objs:
            for sym, size, kind in symbols(obj):
                print(f'{obj.name.replace(".c.obj", ""):<14}{kind:<5}'
                      f'{size:>6}  {sym}')


if __name__ == '__main__':
    main()
