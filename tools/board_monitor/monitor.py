#!/usr/bin/env python3
"""Read-only local display: reuse telemetry HTTP, tail match logs, never open devices."""
import argparse
import errno
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import build_opener, ProxyHandler

APP = 'rescue-board-monitor-v1'
LOG_NAME = re.compile(r'match-main-\d{8}-\d{6}-\d+\.log$')
EVENT = re.compile(r'CONTROL_CHANGE|VISION_EVENT|MATCH\]|WATCHDOG|ERROR|FAULT|RUN_HEADER|CONTROL_TUNING|DEMO_RESULT')
OPENER = build_opener(ProxyHandler({}))


def read_logs(folder):
    try:
        files = [p for p in folder.glob('match-main-*.log') if LOG_NAME.fullmatch(p.name) and not p.is_symlink()]
        if not files:
            return {'name': '', 'lines': [], 'events': [], 'age_s': None}
        path = max(files, key=lambda p: p.name)
        with path.open('rb') as stream:
            stream.seek(0, 2)
            size = stream.tell()
            start = max(0, size - 192 * 1024)
            stream.seek(start)
            data = stream.read(192 * 1024)
        if start:
            data = data.partition(b'\n')[2]
        # Keep only completed lines; a producer may be in the middle of a UTF-8 write.
        text = data.rsplit(b'\n', 1)[0].decode('utf-8', errors='replace') if b'\n' in data else ''
        lines = text.splitlines()
        return {'name': path.name, 'lines': lines[-180:],
                'events': [line for line in lines if EVENT.search(line)][-100:],
                'age_s': max(0, time.time() - path.stat().st_mtime)}
    except OSError as exc:
        return {'name': '', 'lines': [], 'events': [], 'age_s': None, 'error': str(exc)}


class Source:
    def __init__(self, root, upstream):
        self.root, self.upstream = root, upstream.rstrip('/')
        self.lock, self.stop = threading.Lock(), threading.Event()
        self.data = {'app': APP, 'live': False, 'camera_live': False, 'logs': {}}

    def poll(self):
        result = {'app': APP, 'live': False, 'camera_live': False,
                  'stream_url': self.upstream + '/stream.mjpg', 'logs': read_logs(self.root / 'telemetry_logs')}
        try:
            with OPENER.open(self.upstream + '/status', timeout=.7) as response:
                raw = response.read(1024 * 1024 + 1)
            if len(raw) > 1024 * 1024:
                raise ValueError('telemetry response too large')
            telemetry = json.loads(raw)
            health = telemetry.get('health') or {}
            if health.get('source') != 'rescue_upper_host':
                raise ValueError('unexpected telemetry source')
            result['telemetry'] = telemetry
            result['live'] = bool(health.get('robot_data_connected'))
            result['camera_live'] = result['live'] and bool(health.get('camera_ok'))
        except (OSError, ValueError, TypeError, AttributeError) as exc:
            result['error'] = str(exc)
        with self.lock:
            self.data = result

    def run(self):
        while not self.stop.is_set():
            self.poll()
            self.stop.wait(.5)

    def snapshot(self):
        with self.lock:
            return self.data


def handler(source):
    page = Path(__file__).with_name('monitor.html').read_bytes()
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path == '/':
                body, mime = page, 'text/html; charset=utf-8'
            elif path == '/api':
                body, mime = json.dumps(source.snapshot(), ensure_ascii=False).encode(), 'application/json; charset=utf-8'
            elif path == '/health':
                body, mime = json.dumps({'app': APP}).encode(), 'application/json'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass
        def log_message(self, *_):
            pass
    return Handler


def open_window(url):
    # Browsers are viewers only; this never launches the robot/preview process.
    for name in ('chromium', 'chromium-browser', 'google-chrome', 'firefox', 'xdg-open'):
        binary = shutil.which(name)
        if not binary:
            continue
        if name in ('chromium', 'chromium-browser', 'google-chrome'):
            args = [binary, '--app=' + url, '--start-maximized', '--no-first-run']
        elif name == 'firefox':
            args = [binary, '--new-window', url]
        else:
            args = [binary, url]
        try:
            proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            try:
                code = proc.wait(timeout=1)
            except subprocess.TimeoutExpired:
                return
            if code == 0:
                return
        except OSError:
            continue
    raise RuntimeError('没有可用浏览器；请使用显示器桌面的浏览器打开 ' + url)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path('/home/cat/ds_innovation'))
    parser.add_argument('--port', type=int, default=18080)
    parser.add_argument('--telemetry-url', default='http://127.0.0.1:8080')
    parser.add_argument('--open', action='store_true')
    args = parser.parse_args()
    url = 'http://127.0.0.1:%d/' % args.port
    source = Source(args.root, args.telemetry_url)
    try:
        server = ThreadingHTTPServer(('127.0.0.1', args.port), handler(source))
    except OSError as exc:
        if exc.errno != errno.EADDRINUSE:
            raise
        with OPENER.open(url + 'health', timeout=1) as response:
            if json.load(response).get('app') != APP:
                raise RuntimeError('监看端口已被其他程序占用') from exc
        if args.open:
            open_window(url)
        return
    server.daemon_threads = True
    worker = threading.Thread(target=source.run, daemon=True)
    worker.start()
    print('只读监看窗口：' + url, flush=True)
    if args.open:
        try:
            open_window(url)
        except RuntimeError as exc:
            print(exc, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        source.stop.set()
        server.server_close()
        worker.join(timeout=2)


if __name__ == '__main__':
    main()
