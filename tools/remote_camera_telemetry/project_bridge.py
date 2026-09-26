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
    for key in ('source_width', 'source_height', 'image_width', 'image_height'):
        if type(data[key]) is not int or not 0 < data[key] <= 16384:
            raise ValueError('Invalid image size')
    return data, jpeg, timestamp

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
                      hardware_output_enabled=False, imu_connected=False, tof_connected=False)
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
MOTION_SCHEMA = schema({**fields('vx_mps wz_rps hardware_output_enabled'), **fields('valid', 'boolean')})
CONFIG_SCHEMA = schema({**fields('camera_index requested_width requested_height requested_fps confidence nms input_size dry_run'),
    **fields('team model', 'string'), **fields('valid', 'boolean')})
DETECTION_SCHEMA = schema({**fields('valid', 'boolean'), **fields('sequence source_width source_height'),
    'items': {'type': 'array', 'items': schema({**fields('track_id class_id confidence'),
        **fields('label model_label', 'string'), 'box': schema(fields('x y width height'))})}})
CHANNELS = [channel(1, '/camera/image', 'foxglove.CompressedImage', IMAGE_SCHEMA),
            channel(2, '/system/health', 'rescue.RuntimeHealth', HEALTH_SCHEMA),
            channel(3, '/detections', 'rescue.Detections', DETECTION_SCHEMA),
            channel(4, '/fsm/state', 'rescue.PushState', STATE_SCHEMA),
            channel(5, '/cmd/motion', 'rescue.ComputedMotion', MOTION_SCHEMA),
            channel(6, '/runtime/config', 'rescue.RuntimeConfig', CONFIG_SCHEMA)]

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
            payloads = {2: health}
            if data:
                payloads.update({3: {'items': data['detections'], 'valid': live,
                                    'sequence': data['sequence'], 'source_width': data['source_width'],
                                    'source_height': data['source_height']},
                                 4: dict(data['state'], valid=live),
                                 5: dict(data['motion'], valid=live),
                                 6: dict(data['config'], valid=live)})
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
                packet_time = time.time_ns() if cid == 2 else stamp
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
                        if type(sid) is not int or not 0 <= sid <= 0xffffffff or type(cid) is not int or cid not in range(1, 7):
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

PAGE = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ds_innovation 实时观测</title><style>
body{margin:0;background:#101722;color:#e3eaf3;font:16px system-ui}main{max-width:1150px;margin:auto;padding:24px}h1{font-size:24px;margin:0 0 10px}.sub{color:#9eb0c6;margin-bottom:20px}.grid{display:grid;grid-template-columns:2fr 1fr;gap:20px}.card{background:#1c2736;border-radius:12px;padding:18px;margin-bottom:16px}img{width:100%;display:block;min-height:200px;background:#101722}h2{font-size:17px;margin:0 0 12px}pre{white-space:pre-wrap;overflow-wrap:anywhere;font:14px ui-monospace,monospace}.good{color:#79e2ad}.bad{color:#ffb685}table{width:100%;border-collapse:collapse}td,th{text-align:left;padding:8px;border-bottom:1px solid #334155}th{color:#9eb0c6}@media(max-width:750px){.grid{grid-template-columns:1fr}}
</style><main><h1>ds_innovation 实时观测</h1><div class="sub">来自板子主程序的检测画面与运行状态 · 当前为预览模式，未下发运动指令</div><div id="connection" class="card">正在连接…</div><div class="grid"><section><div class="card"><h2>检测画面</h2><img id="camera" src="/stream.mjpg" alt="等待主程序图像"></div><div class="card"><h2>检测结果</h2><table><thead><tr><th>目标 ID</th><th>类别</th><th>置信度</th></tr></thead><tbody id="detections"></tbody></table></div></section><aside><div class="card"><h2>运行状态</h2><pre id="state"></pre></div><div class="card"><h2>性能与图像</h2><pre id="health"></pre></div><div class="card"><h2>运行参数</h2><pre id="config"></pre></div><div class="card sub">IMU、ToF 尚未接入。速度字段表示程序计算值，不代表底盘实测速度。</div></aside></div></main><script>
const byId=id=>document.getElementById(id);let busy=false;
async function update(){if(busy)return;busy=true;try{const r=await fetch('/status',{signal:AbortSignal.timeout(3000)});const s=await r.json();const h=s.health;const live=h.robot_data_connected;byId('connection').textContent=live?'主程序在线 · 图像持续更新':'主程序未更新：'+h.error;byId('connection').className='card '+(live?'good':'bad');byId('camera').style.visibility=live?'visible':'hidden';const f=s.state||{};byId('state').textContent=`状态：${f.name??'等待数据'}\n批次数量：${f.batch_size??'-'}\n累计交付：${f.delivered_total??'-'}\n目标 ID：${f.target_id??'-'}\n运动输出：未启用`;const n=v=>typeof v==='number'?v.toFixed(2):'-';byId('health').textContent=`主循环：${n(h.loop_fps)} FPS\n推理及跟踪：${n(h.inference_ms)} ms\n采集等待：${n(h.capture_ms)} ms\n图像延迟：${n(h.frame_age_s)} s\n采集尺寸：${h.source_width??'-'} × ${h.source_height??'-'}\n预览尺寸：${h.image_width??'-'} × ${h.image_height??'-'}\n遥测丢帧：${h.dropped_publish_frames??'-'}`;const c=s.config||{};byId('config').textContent=`相机编号：${c.camera_index??'-'}\n请求帧率：${c.requested_fps??'-'}\n检测阈值：${n(c.confidence)}\nNMS 阈值：${n(c.nms)}\n模型输入：${c.input_size??'-'}\n队伍：${c.team??'-'}`;byId('detections').replaceChildren();for(const d of live?(s.detections||[]):[]){const tr=document.createElement('tr');for(const value of [d.track_id,d.label,(d.confidence*100).toFixed(1)+'%']){const td=document.createElement('td');td.textContent=value;tr.append(td)}byId('detections').append(tr)}}catch(e){byId('connection').textContent='连接中断，正在重试';byId('connection').className='card bad';byId('camera').style.visibility='hidden'}finally{busy=false}}setInterval(update,500);update();
byId('camera').onerror=()=>setTimeout(()=>{byId('camera').src='/stream.mjpg?t='+Date.now()},2000);
</script></html>'''.encode()

def handler(source):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path == '/':
                body, mime = PAGE, 'text/html; charset=utf-8'
            elif path == '/status':
                data, _, _, health = source.snapshot()
                body = json.dumps({'health': health, 'state': data['state'] if data else None,
                    'motion': data['motion'] if data else None, 'config': data['config'] if data else None,
                    'detections': data['detections'] if data else []}).encode()
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
