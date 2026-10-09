#!/usr/bin/env python3
"""Start/stop the project preview and its read-only viewing service together."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[2]
LOGS = ROOT / 'telemetry_logs'
BINARY = ROOT / 'build-arm64-telemetry/rescue_upper_host'
BRIDGE = ROOT / 'tools/remote_camera_telemetry/project_bridge.py'


def is_ours(pid, expected):
    try:
        return expected.encode() in Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
    except OSError:
        return False


def tracked():
    result = {}
    for name, expected in [('main', str(BINARY)), ('bridge', str(BRIDGE))]:
        try:
            pid = int((LOGS / f'{name}.pid').read_text())
            if pid > 1 and is_ours(pid, expected):
                result[name] = pid
        except (OSError, ValueError):
            pass
    return result


def stop():
    running = tracked()
    for pid in running.values():
        try: os.kill(pid, signal.SIGTERM)
        except ProcessLookupError: pass
    end = time.monotonic() + 5
    while tracked() and time.monotonic() < end:
        time.sleep(0.1)
    if tracked():
        raise RuntimeError('Some processes have not exited; check telemetry_logs before restarting')
    for name in ('main', 'bridge'):
        (LOGS / f'{name}.pid').unlink(missing_ok=True)
    print('Project preview and viewing service stopped')


def start(extra):
    if tracked():
        raise RuntimeError('Project preview is already running; use status or stop first')
    if not BINARY.is_file():
        raise RuntimeError('Build build-arm64-telemetry with RESCUE_ENABLE_TELEMETRY=ON first')
    # Refuse to replace an existing service, or to open a second copy of the camera.
    for port in (8080, 8765):
        with socket.socket() as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try: sock.bind(('0.0.0.0', port))
            except OSError: raise RuntimeError(f'Port {port} is in use; stop the old standalone camera service first')
    if any(arg in ('--help', '-h', '--detect-image', '--push-replay', '--telemetry-file') for arg in extra):
        raise RuntimeError('Use the main executable directly for help, replay, image tests or a custom snapshot path')
    timestamp = time.strftime('%Y%m%d-%H%M%S')
    commands = {
        'bridge': [sys.executable, str(BRIDGE)],
        'main': ['stdbuf', '-oL', '-eL', str(BINARY), '--dry-run', '--no-show', '--telemetry', *extra],
    }
    try:
        for name, command in commands.items():
            log_path = LOGS / f'{name}-{timestamp}.log'
            with log_path.open('ab') as log:
                process = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.DEVNULL,
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            (LOGS / f'{name}.pid').write_text(str(process.pid) + '\n')
            link = LOGS / f'{name}.log'
            link.unlink(missing_ok=True)
            link.symlink_to(log_path.name)
        for _ in range(30):
            time.sleep(0.5)
            if len(tracked()) != 2:
                raise RuntimeError('A process exited; see telemetry_logs/main.log and bridge.log')
            try:
                with urlopen('http://127.0.0.1:8080/status', timeout=1) as response:
                    status = json.load(response)
                if status['health']['robot_data_connected']:
                    print('Project preview is running. Foxglove: ws://192.168.34.7:8765')
                    print('Browser: http://192.168.34.7:8080/')
                    print('Logs:', LOGS)
                    return
            except (OSError, ValueError, KeyError):
                pass
        raise RuntimeError('No fresh project image received within startup timeout')
    except Exception:
        stop()
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['start', 'stop', 'status'])
    args, extra = parser.parse_known_args()
    LOGS.mkdir(exist_ok=True)
    with (LOGS / 'manager.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if args.action == 'start': start(extra)
        elif args.action == 'stop': stop()
        else:
            print('Processes:', tracked())
            try:
                with urlopen('http://127.0.0.1:8080/status', timeout=2) as response:
                    print(json.dumps(json.load(response), indent=2, ensure_ascii=False))
            except OSError as exc:
                print('Viewing service unavailable:', exc)

if __name__ == '__main__':
    try: main()
    except Exception as exc:
        print('Error:', exc, file=sys.stderr)
        sys.exit(1)
