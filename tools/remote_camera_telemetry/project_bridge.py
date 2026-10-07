#!/usr/bin/env python3
"""Read-only Foxglove/HTTP bridge for the C++ main program's atomic snapshots.

This process never opens a camera, serial port, or robot command channel.
"""
import argparse
import faulthandler
import asyncio
import base64
import json
import math
import os
import signal
import socket
import struct
import threading
import time
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from websockets.server import serve
from server import IMAGE_SCHEMA, channel

MAX_METADATA = 1024 * 1024
MAX_JPEG = 8 * 1024 * 1024

def read_packet(path):
    with open(path, 'rb') as f:
        header = f.read(16)
        if len(header) != 16 or header[:8] != b'RSTEL001':
            raise ValueError('Invalid snapshot header')
        meta_size, jpeg_size = struct.unpack('<II', header[8:])
        if not 0 < meta_size <= MAX_METADATA or not 4 <= jpeg_size <= MAX_JPEG:
            raise ValueError('Snapshot exceeds size limits')
        meta_bytes, jpeg = f.read(meta_size), f.read(jpeg_size)
        if len(meta_bytes) != meta_size or len(jpeg) != jpeg_size or f.read(1):
            raise ValueError('Truncated or oversized snapshot')
    def invalid_constant(value):
        raise ValueError('Non-finite JSON value: ' + value)
    data = json.loads(meta_bytes, parse_constant=invalid_constant)
    if data['version'] != 1 or not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'):
        raise ValueError('Invalid snapshot version or JPEG')
    stamp = data['timestamp']
    if type(stamp['sec']) is not int or type(stamp['nsec']) is not int or not 0 <= stamp['nsec'] < 10**9:
        raise ValueError('Invalid timestamp')
    timestamp = stamp['sec'] * 10**9 + stamp['nsec']
    if timestamp <= 0 or not isinstance(data['detections'], list) or len(data['detections']) > 4096:
        raise ValueError('Invalid observations')
    for key in ('state', 'health', 'motion', 'config'):
        if not isinstance(data[key], dict):
            raise ValueError('Missing snapshot section: ' + key)
    if 'decision' in data and not isinstance(data['decision'], dict):
        raise ValueError('Invalid decision section')
    for key in ('source_width', 'source_height', 'image_width', 'image_height'):
        if type(data[key]) is not int or not 0 < data[key] <= 16384:
            raise ValueError('Invalid image size')
    return data, jpeg, timestamp

def imu_view(data, source_live):
    imu = dict(data.get('imu', {})) if data else {}
    stamp = imu.get('received_monotonic_us', 0)
    age = (time.monotonic_ns() / 1000 - stamp) / 1000 if stamp else None
    fresh = bool(source_live and imu.get('enabled') and imu.get('connected') and
                 age is not None and 0 <= age <= imu.get('timeout_ms', 200))
    for key in ('enabled', 'connected', 'measurements_valid'):
        imu[key] = bool(imu.get(key, False))
    imu.update(fresh=fresh, valid=bool(fresh and imu.get('measurements_valid')),
               age_ms=round(age, 2) if age is not None else None,
               attitude_valid_for_control=False)
    return imu

class Source:
    def __init__(self, path, fps):
        self.path, self.fps = path, fps
        self.lock, self.stop = threading.Lock(), threading.Event()
        self.data, self.jpeg, self.timestamp = None, None, 0
        self.error = 'Waiting for the main program'
        self.identity = None

    def poll(self):
        try:
            stat = os.stat(self.path)
            identity = (stat.st_ino, stat.st_mtime_ns, stat.st_size)
            if identity == self.identity:
                return
            data, jpeg, timestamp = read_packet(self.path)
            with self.lock:
                self.data, self.jpeg, self.timestamp = data, jpeg, timestamp
                self.error, self.identity = None, identity
        except (OSError, ValueError, KeyError, TypeError, OverflowError) as exc:
            with self.lock:
                self.error = 'Main program is not publishing' if isinstance(exc, FileNotFoundError) else str(exc)
                self.identity = None

    def run(self):
        while not self.stop.is_set():
            self.poll()
            self.stop.wait(0.05)

    def snapshot(self):
        with self.lock:
            data, jpeg, stamp, error = self.data, self.jpeg, self.timestamp, self.error
        age = (time.time_ns() - stamp) / 1e9 if stamp else None
        live = error is None and age is not None and 0 <= age < 2
        health = dict(data['health']) if data else {}
        health.update(camera_ok=live, robot_data_connected=live, source='rescue_upper_host',
                      frame_age_s=round(age, 3) if age is not None else None,
                      sequence=data['sequence'] if data else 0,
                      error=error or (None if live else 'Main program frames are stale'),
                      hardware_output_enabled=bool(live and data and data['motion'].get('hardware_output_enabled')),
                      imu_connected=imu_view(data, live)['fresh'], tof_connected=False)
        if data:
            health.update(source_width=data['source_width'], source_height=data['source_height'],
                          image_width=data['image_width'], image_height=data['image_height'])
        return data, jpeg, stamp, health


def schema(properties):
    return {'type': 'object', 'properties': properties}

def fields(names, kind='number'):
    return {name: {'type': kind} for name in names.split()}

HEALTH_SCHEMA = schema({**fields('loop_fps inference_ms capture_ms publish_fps_limit dropped_publish_frames publish_errors sequence source_width source_height image_width image_height'),
    **fields('camera_ok robot_data_connected hardware_output_enabled imu_connected tof_connected', 'boolean'),
    'source': {'type': 'string'}, 'error': {'type': ['string', 'null']},
    'frame_age_s': {'type': ['number', 'null']}})
STATE_SCHEMA = schema({**fields('name target_label', 'string'), **fields('valid', 'boolean'),
    **fields('batch_size delivered_total first_ordinary_delivered run_requested target_valid target_id geometry_valid path_safe safety_ok zone_valid')})
DECISION_SCHEMA = schema({
    **fields('mode phase phase_reason match_state match_reason preflight_reason target_label zone_color zone_color_reason geometry_reason zone_estimate_reason', 'string'),
    **fields('match_remaining_ms preflight_ready run_permitted safety_ok target_id target_valid target_geometry_valid target_region_known target_in_own_zone target_distance_m target_heading_error_rad pose_model_ran zone_identity_verified zone_valid zone_own path_safe retreat_safe opponent_zone_clear corridor_complete corridor_occlusion_free corridor_total hold_observable captured held_complete held_total gripper_feedback_confirmed gripper_feedback_open camera_pitch_readback_cdeg camera_protocol_readback_deg camera_protocol_cmd_deg camera_pitch_stable zone_inventory_complete zone_counts_valid zone_supply_count zone_injured_count carry_plan_valid drop_plan_valid drop_locked drop_x_zone_m drop_y_zone_m computed_vx_mps computed_wz_rps frame_offset_cmd_deg camera_pitch_cmd_cdeg hardware_output_enabled'),
    'missing_evidence': {'type': 'array', 'items': {'type': 'string'}},
    'valid': {'type': 'boolean'},
})
MOTION_SCHEMA = schema({**fields('vx_mps wz_rps hardware_output_enabled'), **fields('valid', 'boolean')})
CONFIG_SCHEMA = schema({**fields('camera_index requested_width requested_height requested_fps confidence nms input_size dry_run'),
    **fields('team model imu_port', 'string'), **fields('imu_baud imu_enabled'), **fields('valid', 'boolean')})
DETECTION_SCHEMA = schema({**fields('valid', 'boolean'), **fields('sequence source_width source_height'),
    'items': {'type': 'array', 'items': schema({**fields('track_id class_id confidence'),
        **fields('label model_label', 'string'), 'box': schema(fields('x y width height'))})}})
IMU_SCHEMA = schema({**fields('enabled connected fresh measurements_valid valid attitude_valid_for_control', 'boolean'),
    **fields('sequence device_time_ms received_monotonic_us timeout_ms status_raw temperature_c pressure_pa bytes valid_frames crc_errors invalid_frames duplicate_times backward_times io_errors'),
    'age_ms': {'type': ['number', 'null']}, 'frame_id': {'type': 'string'}, 'body_frame_id': {'type': 'string'},
    **{name: {'type': 'array', 'items': {'type': 'number'}} for name in
       ('acceleration_mps2', 'angular_velocity_rps', 'magnetic_ut', 'rpy_rad', 'quaternion_wxyz',
        'body_acceleration_mps2', 'body_angular_velocity_rps', 'body_rpy_rad', 'body_quaternion_wxyz')}})
CHANNELS = [channel(1, '/camera/image', 'foxglove.CompressedImage', IMAGE_SCHEMA),
            channel(2, '/system/health', 'rescue.RuntimeHealth', HEALTH_SCHEMA),
            channel(3, '/detections', 'rescue.Detections', DETECTION_SCHEMA),
            channel(4, '/fsm/state', 'rescue.PushState', STATE_SCHEMA),
            channel(5, '/cmd/motion', 'rescue.ComputedMotion', MOTION_SCHEMA),
            channel(6, '/runtime/config', 'rescue.RuntimeConfig', CONFIG_SCHEMA),
            channel(7, '/imu/data', 'rescue.Hi91Imu', IMU_SCHEMA),
            channel(8, '/decision/live', 'rescue.DecisionLive', DECISION_SCHEMA)]

async def client(ws, source):
    if ws.subprotocol != 'foxglove.websocket.v1':
        await ws.close(1002, 'Foxglove subprotocol required')
        return
    subscriptions = {}
    await ws.send(json.dumps({'op': 'serverInfo', 'name': 'ds_innovation main program',
                             'capabilities': [], 'supportedEncodings': ['json'], 'metadata': {}}))
    await ws.send(json.dumps({'op': 'advertise', 'channels': CHANNELS}))

    async def send_frames():
        previous = {}
        while not source.stop.is_set():
            data, jpeg, stamp, health = source.snapshot()
            live = health['robot_data_connected']
            payloads = {2: health, 7: imu_view(data, live)}
            if data:
                payloads.update({3: {'items': data['detections'], 'valid': live,
                                    'sequence': data['sequence'], 'source_width': data['source_width'],
                                    'source_height': data['source_height']},
                                 4: dict(data['state'], valid=live),
                                 5: dict(data['motion'], valid=live),
                                 6: dict(data['config'], valid=live),
                                 8: dict(data.get('decision') or {}, valid=bool(live and isinstance(data.get('decision'), dict)))})
            encoded = {}
            for sid, cid in list(subscriptions.items()):
                if cid == 1:
                    if not live or previous.get(sid) == stamp:
                        continue
                    if 1 not in encoded:
                        payload = {'timestamp': data['timestamp'], 'frame_id': data['frame_id'],
                                   'format': 'jpeg', 'data': base64.b64encode(jpeg).decode()}
                        encoded[1] = json.dumps(payload, separators=(',', ':')).encode()
                elif cid not in payloads:
                    continue
                elif cid not in encoded:
                    encoded[cid] = json.dumps(payloads[cid], allow_nan=False, separators=(',', ':')).encode()
                packet_time = time.time_ns() if cid in (2, 7) else stamp
                packet = struct.pack('<BIQ', 1, sid, packet_time) + encoded[cid]
                await asyncio.wait_for(ws.send(packet), timeout=2)
                previous[sid] = stamp
            # Clear removed subscription state, including newly re-used IDs.
            for sid in list(previous):
                if sid not in subscriptions:
                    del previous[sid]
            await asyncio.sleep(1 / source.fps)

    async def receive():
        async for message in ws:
            try:
                msg = json.loads(message) if isinstance(message, str) else {}
                if msg.get('op') == 'subscribe':
                    for sub in msg['subscriptions']:
                        sid, cid = sub['id'], sub['channelId']
                        if type(sid) is not int or not 0 <= sid <= 0xffffffff or type(cid) is not int or cid not in range(1, 9):
                            raise ValueError('Invalid subscription')
                        if len(subscriptions) >= 12 and sid not in subscriptions:
                            raise ValueError('Too many subscriptions')
                        if sid in subscriptions:
                            raise ValueError('Duplicate subscription ID')
                        subscriptions[sid] = cid
                elif msg.get('op') == 'unsubscribe':
                    for sid in msg['subscriptionIds']:
                        if type(sid) is not int:
                            raise ValueError('Invalid subscription ID')
                        subscriptions.pop(sid, None)
                else:
                    raise ValueError('Only viewing subscriptions are supported')
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
        try:
            await asyncio.wait_for(ws.close(), timeout=2)
        except asyncio.TimeoutError:
            ws.transport.abort()

PAGE = Path(__file__).with_name('dashboard.html').read_bytes()

def handler(source):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path == '/':
                body, mime = PAGE, 'text/html; charset=utf-8'
            elif path == '/status':
                data, _, _, health = source.snapshot()
                decision = dict(data.get('decision') or {}, valid=bool(health['robot_data_connected'] and isinstance(data.get('decision'), dict))) if data else None
                body = json.dumps({'health': health, 'state': data['state'] if data else None,
                    'decision': decision,
                    'motion': data['motion'] if data else None, 'config': data['config'] if data else None,
                    'detections': data['detections'] if data else [],
                    'imu': imu_view(data, health['robot_data_connected'])}).encode()
                mime = 'application/json'
            elif path == '/stream.mjpg':
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.connection.settimeout(2)
                previous = 0
                try:
                    while not source.stop.is_set():
                        _, jpeg, stamp, health = source.snapshot()
                        if health['camera_ok'] and stamp != previous:
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
                            self.wfile.flush()
                            previous = stamp
                        source.stop.wait(1 / source.fps)
                except (OSError, socket.timeout):
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
            try:
                self.wfile.write(body)
            except OSError:
                pass
        def log_message(self, *_):
            pass
    return Handler

async def main(args):
    source = Source(args.snapshot, args.fps)
    http = ThreadingHTTPServer((args.host, args.http_port), handler(source))
    stopped = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stopped.set)
    http_thread = None
    try:
        async with serve(lambda ws: client(ws, source), args.host, args.ws_port,
                         subprotocols=['foxglove.websocket.v1'], compression=None,
                         max_size=65536, max_queue=4, close_timeout=2):
            threading.Thread(target=source.run, daemon=True).start()
            http_thread = threading.Thread(target=http.serve_forever, daemon=True)
            http_thread.start()
            print(f'Main-program telemetry: HTTP {args.http_port}, Foxglove {args.ws_port}', flush=True)
            await stopped.wait()
            source.stop.set()
    finally:
        source.stop.set()
        if http_thread:
            http.shutdown()
        http.server_close()

if __name__ == '__main__':
    faulthandler.register(signal.SIGUSR1, all_threads=True)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', default='/dev/shm/rescue-telemetry.bin')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--http-port', type=int, default=8080)
    parser.add_argument('--ws-port', type=int, default=8765)
    parser.add_argument('--fps', type=int, default=10)
    args = parser.parse_args()
    if not 1 <= args.fps <= 30:
        parser.error('--fps must be 1..30')
    asyncio.run(main(args))
