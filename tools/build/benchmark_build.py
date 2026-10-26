#!/usr/bin/env python3
"""Compare clean builds with empty and warm ccache on a configured Ninja tree."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time


def sensors():
    info = {}
    for line in Path('/proc/meminfo').read_text().splitlines():
        key, value = line.split(':', 1)
        if key in ('MemAvailable', 'SwapTotal', 'SwapFree'):
            info[key] = int(value.split()[0])
    temperatures = []
    for path in Path('/sys/class/thermal').glob('thermal_zone*/temp'):
        try:
            temperatures.append(int(path.read_text()) / 1000)
        except (OSError, ValueError):
            pass
    return info['MemAvailable'], info['SwapTotal'] - info['SwapFree'], max(temperatures, default=0)


def save_text(path, text):
    with path.open('w') as stream:
        stream.write(text)
        stream.flush()
        os.fsync(stream.fileno())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--jobs', nargs='+', type=int, default=[4, 6, 8])
    parser.add_argument('--ccache', default=shutil.which('ccache') or str(Path.home() / '.local/bin/ccache'))
    args = parser.parse_args()
    if any(j < 1 for j in args.jobs):
        parser.error('--jobs must be positive')
    build = args.build_dir.resolve()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    for filename in ('CMakeCache.txt', 'compile_commands.json'):
        shutil.copy2(build / filename, output / filename)
    commands = json.loads((build / 'compile_commands.json').read_text())
    metadata = {'build_dir': str(build), 'compile_units': len(commands), 'unique_sources': len({c['file'] for c in commands}), 'jobs': args.jobs}
    save_text(output / 'metadata.json', json.dumps(metadata, indent=2) + chr(10))
    results = []
    for jobs in args.jobs:
        # Private temporary cache: never clear or reset the developer's normal cache.
        with tempfile.TemporaryDirectory(prefix='ccache-', dir=output) as cache:
            env = dict(os.environ, CCACHE_DIR=cache, CCACHE_MAXSIZE='2G')
            for mode in ('cold', 'warm'):
                subprocess.run(['cmake', '--build', str(build), '--target', 'clean'], env=env, check=True, stdout=subprocess.DEVNULL)
                subprocess.run([args.ccache, '--zero-stats'], env=env, check=True, stdout=subprocess.DEVNULL)
                tag = f'{mode}-j{jobs}'
                # Give each run the same brief idle interval; record heat and memory.
                time.sleep(5)
                available, swap, temp = sensors()
                min_available, max_swap, max_temp = available, swap, temp
                start = time.monotonic()
                with (output / f'{tag}.log').open('w') as log:
                    process = subprocess.Popen(['cmake', '--build', str(build), '--parallel', str(jobs)], env=env, stdout=log, stderr=subprocess.STDOUT)
                    while True:
                        try:
                            returncode = process.wait(timeout=0.5)
                            break
                        except subprocess.TimeoutExpired:
                            available, swap, temp = sensors()
                            min_available = min(min_available, available)
                            max_swap = max(max_swap, swap)
                            max_temp = max(max_temp, temp)
                    log.flush()
                    os.fsync(log.fileno())
                elapsed = time.monotonic() - start
                stats = subprocess.check_output([args.ccache, '--show-stats'], env=env, text=True)
                save_text(output / f'{tag}.ccache.txt', stats)
                result = {'mode': mode, 'jobs': jobs, 'seconds': round(elapsed, 3), 'exit_code': returncode, 'min_available_mib': round(min_available / 1024, 1), 'max_swap_used_mib': round(max_swap / 1024, 1), 'max_temp_c': max_temp}
                results.append(result)
                save_text(output / 'results.json', json.dumps(results, indent=2) + chr(10))
                print(json.dumps(result), flush=True)
                if returncode:
                    raise SystemExit(f'Build failed: {output / (tag + ".log")}')


if __name__ == '__main__':
    main()
