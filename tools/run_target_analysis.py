"""Compile project-owned translation units with GCC diagnostics and analyzer."""
import json
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
entries = json.loads((ROOT / 'build/compile_commands.json').read_text())
failed = 0
count = 0
with tempfile.TemporaryDirectory(prefix='supercar-analysis-') as temporary:
    with (ROOT / 'testing/review/final-static-analysis.txt').open('w') as report:
        for entry in entries:
            path = Path(entry['file'])
            project = ('app_program', 'config')
            if not any(part in project for part in
                       path.relative_to(ROOT).parts[:1]):
                continue
            if path.suffix != '.c':
                continue
            arguments = []
            skip = False
            for token in shlex.split(entry['command']):
                if skip:
                    skip = False
                    continue
                if token == '-o':
                    skip = True
                    continue
                if token == '-c' or token.startswith('-O'):
                    continue
                arguments.append(token)
            arguments += ['-c', '-o', str(Path(temporary) / (path.stem + '.o')),
                          '-O0', '-Wall', '-Wextra', '-Wformat=2',
                          '-Wconversion', '-Wsign-conversion', '-Wshadow',
                          '-Wstrict-prototypes', '-Wmissing-prototypes',
                          '-fanalyzer']
            result = subprocess.run(arguments, cwd=entry['directory'],
                                    capture_output=True, text=True)
            report.write(f'\nMODULE {path.relative_to(ROOT)}: '
                         f'exit={result.returncode}\n' +
                         result.stdout + result.stderr)
            report.flush()
            count += 1
            failed += result.returncode != 0
print(f'{count} translation units; {failed} compiler failures.')
raise SystemExit(failed != 0)
