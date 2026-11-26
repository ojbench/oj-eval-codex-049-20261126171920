"""Compile the supplied simulator and run every distributed sample."""
import pathlib
import shutil
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
DATA = next(pathlib.Path('/workspace/data/049').glob('*/src')).parent

with tempfile.TemporaryDirectory() as directory:
    work = pathlib.Path(directory)
    for source in (DATA / 'src').iterdir():
        if source.suffix in {'.h', '.cpp'}:
            shutil.copy(source, work)
    shutil.copy(ROOT / 'src.hpp', work)
    subprocess.run(['g++', '-std=c++17', '-O2', 'main.cpp', '-o', 'sim'],
                   cwd=work, check=True, capture_output=True)
    for case in sorted((DATA / 'testcases').glob('*.in'), key=lambda p: int(p.stem)):
        result = subprocess.run([str(work / 'sim')], input=case.read_text(),
                                text=True, capture_output=True, cwd=work, timeout=20)
        assert result.returncode == 0, result.stderr
        assert '[Collision]' not in result.stderr, result.stderr[:1000]
        assert '[Speeding]' not in result.stderr, result.stderr[:1000]
        assert 'Ended at round' in result.stderr, result.stderr
        print(f'{case.name}: {result.stderr.strip()}')
