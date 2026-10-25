#!/usr/bin/env python3
"""Standalone USB camera preview and read-only Foxglove WebSocket bridge.

Requires system OpenCV and websockets 10.x. Does not connect to robot controls.
"""
import argparse
import asyncio
import base64
import json
import signal
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
from websockets.server import serve

PAGE = b'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Camera preview</title><style>body{margin:32px;background:#111827;color:#e5e7eb;font:18px system-ui}img{width:100%;max-width:960px;background:#000;border-radius:12px}pre{white-space:pre-wrap}a{color:#93c5fd}</style>
<h1>USB camera live preview</h1><img src="/stream.mjpg" alt="Waiting for camera">
<p>Read-only camera telemetry. Robot detection and state data are not connected.</p>
<pre id="status">Connecting...</pre><script>async function update(){try{const r=await fetch('/status');document.getElementById('status').textContent=JSON.stringify(await r.json(),null,2)}catch(e){document.getElementById('status').textContent='Disconnected'}}setInterval(update,1000);update()</script></html>'''


class Camera:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.jpeg = None
        self.timestamp = 0
        self.sequence = 0
        self.capture_fps = 0.0
        self.error = 'Waiting for camera'

    def snapshot(self):
        with self.lock:
            age = (time.time_ns() - self.timestamp) / 1e9 if self.timestamp else None
            return self.jpeg, self.timestamp, {
                'camera_ok': self.error is None and age is not None and age < 2,
                'error': self.error, 'sequence': self.sequence,
                'capture_fps': self.capture_fps, 'publish_fps_limit': self.args.fps,
                'frame_age_s': round(age, 3) if age is not None else None,
                'device': self.args.device, 'robot_data_connected': False,
            }

    def run(self):
        while not self.stop.is_set():
            cap = cv2.VideoCapture(self.args.device, cv2.CAP_V4L2)
            try:
                if not cap.isOpened():
                    raise RuntimeError('Cannot open camera')
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_FPS, 30)
                start, count, next_encode = time.monotonic(), 0, 0
                while not self.stop.is_set():
                    ok, frame = cap.read()
                    if not ok:
                        raise RuntimeError('Camera read failed')
                    now = time.monotonic()
                    count += 1
                    if now - start >= 1:
                        with self.lock:
                            self.capture_fps = round(count / (now - start), 2)
                        start, count = now, 0
                    if now < next_encode:
                        continue
                    next_encode = now + 1 / self.args.fps
                    timestamp = time.time_ns()
                    if frame.shape[:2] != (480, 640):
                        frame = cv2.resize(frame, (640, 480))
                    ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    if not ok:
                        raise RuntimeError('JPEG encoding failed')
                    with self.lock:
                        self.jpeg, self.timestamp = jpeg.tobytes(), timestamp
                        self.sequence += 1
                        self.error = None
            except Exception as exc:
                with self.lock:
                    self.error, self.jpeg = str(exc), None
                print('Camera:', exc, flush=True)
            finally:
                cap.release()
            self.stop.wait(1)


def http_handler(camera):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/':
                body, mime = PAGE, 'text/html; charset=utf-8'
            elif self.path == '/status':
                body, mime = json.dumps(camera.snapshot()[2]).encode(), 'application/json'
            elif self.path == '/stream.mjpg':
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.connection.settimeout(3)
                previous = 0
                try:
                    while not camera.stop.is_set():
                        jpeg, timestamp, status = camera.snapshot()
                        if jpeg and status['camera_ok'] and timestamp != previous:
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
                            self.wfile.flush()
                            previous = timestamp
                        camera.stop.wait(1 / camera.args.fps)
                except (BrokenPipeError, ConnectionResetError, socket.timeout):
                    pass
                return
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *_):
            pass
    return Handler


IMAGE_SCHEMA = {'type': 'object', 'properties': {
    'timestamp': {'type': 'object', 'properties': {'sec': {'type': 'integer'}, 'nsec': {'type': 'integer'}}, 'required': ['sec', 'nsec']},
    'frame_id': {'type': 'string'}, 'data': {'type': 'string', 'contentEncoding': 'base64'}, 'format': {'type': 'string'},
}, 'required': ['timestamp', 'frame_id', 'data', 'format']}
HEALTH_SCHEMA = {'type': 'object', 'properties': {
    'camera_ok': {'type': 'boolean'}, 'sequence': {'type': 'integer'},
    'capture_fps': {'type': 'number'}, 'publish_fps_limit': {'type': 'number'},
    'frame_age_s': {'type': ['number', 'null']}, 'error': {'type': ['string', 'null']},
    'device': {'type': 'string'}, 'robot_data_connected': {'type': 'boolean'},
}}


def channel(number, topic, name, schema):
    return {'id': number, 'topic': topic, 'encoding': 'json', 'schemaName': name,
            'schemaEncoding': 'jsonschema', 'schema': json.dumps(schema)}


async def client(ws, camera):
    if ws.subprotocol != 'foxglove.websocket.v1':
        await ws.close(1002, 'Foxglove subprotocol required')
        return
    subscriptions = {}
    await ws.send(json.dumps({'op': 'serverInfo', 'name': 'USB camera telemetry',
                             'capabilities': [], 'supportedEncodings': ['json'], 'metadata': {}}))
    await ws.send(json.dumps({'op': 'advertise', 'channels': [
        channel(1, '/camera/image', 'foxglove.CompressedImage', IMAGE_SCHEMA),
        channel(2, '/system/health', 'rescue.CameraHealth', HEALTH_SCHEMA),
    ]}))

    async def send_frames():
        previous = 0
        while True:
            jpeg, timestamp, health = camera.snapshot()
            fresh = timestamp != previous and jpeg and health['camera_ok']
            for sub_id, channel_id in list(subscriptions.items()):
                if channel_id == 1:
                    if not fresh:
                        continue
                    payload = {'timestamp': {'sec': timestamp // 1_000_000_000, 'nsec': timestamp % 1_000_000_000},
                               'frame_id': 'camera_optical', 'format': 'jpeg', 'data': base64.b64encode(jpeg).decode()}
                    stamp = timestamp
                else:
                    payload, stamp = health, time.time_ns()
                packet = struct.pack('<BIQ', 1, sub_id, stamp) + json.dumps(payload).encode()
                await asyncio.wait_for(ws.send(packet), timeout=3)
            previous = timestamp
            await asyncio.sleep(1 / camera.args.fps)

    async def receive():
        async for message in ws:
            try:
                msg = json.loads(message) if isinstance(message, str) else {}
                if msg.get('op') == 'subscribe':
                    for sub in msg['subscriptions']:
                        sid, cid = sub['id'], sub['channelId']
                        if type(sid) is not int or not 0 <= sid <= 0xffffffff or cid not in (1, 2):
                            raise ValueError('Invalid subscription')
                        if len(subscriptions) >= 8 and sid not in subscriptions:
                            raise ValueError('Too many subscriptions')
                        subscriptions[sid] = cid
                elif msg.get('op') == 'unsubscribe':
                    for sid in msg['subscriptionIds']:
                        subscriptions.pop(sid, None)
                else:
                    raise ValueError('Only subscribe/unsubscribe supported')
            except (ValueError, KeyError, TypeError, AttributeError):
                await ws.close(1008, 'Invalid or unsupported request')
                return

    tasks = [asyncio.create_task(send_frames()), asyncio.create_task(receive())]
    try:
        await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        await ws.close()


async def main(args):
    camera = Camera(args)
    http = ThreadingHTTPServer((args.host, args.http_port), http_handler(camera))
    stopped = asyncio.Event()
    http_started = False
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stopped.set)
    try:
        async with serve(lambda ws: client(ws, camera), args.host, args.ws_port,
                         subprotocols=['foxglove.websocket.v1'], compression=None,
                         max_size=65536, max_queue=4, close_timeout=2):
            threading.Thread(target=camera.run, daemon=True).start()
            threading.Thread(target=http.serve_forever, daemon=True).start()
            http_started = True
            print(f'Preview http://{args.host}:{args.http_port} ; Foxglove ws://{args.host}:{args.ws_port}', flush=True)
            await stopped.wait()
    finally:
        camera.stop.set()
        if http_started:
            http.shutdown()
        http.server_close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default='/dev/v4l/by-id/usb-RYS_USB_Camera_200901010001-video-index0')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--http-port', type=int, default=8080)
    parser.add_argument('--ws-port', type=int, default=8765)
    parser.add_argument('--fps', type=float, default=10)
    args = parser.parse_args()
    if not 0 < args.fps <= 30:
        parser.error('--fps must be between 0 and 30')
    asyncio.run(main(args))
