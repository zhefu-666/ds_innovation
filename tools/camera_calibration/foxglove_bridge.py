"""Read-only Foxglove transport sharing the calibration camera's snapshots."""
import asyncio
import base64
import json
import struct
import threading
import time

from websockets.server import serve


def channel(number, topic, name, properties, required=None):
    # JSON schema titles identify the root datatype to Foxglove's JSON decoder.
    schema = dict(title=name, type='object', properties=properties)
    if required is not None: schema['required'] = required
    return dict(id=number, topic=topic, encoding='json', schemaName=name,
                schemaEncoding='jsonschema', schema=json.dumps(schema))


def fields(names, kind):
    return {name: {'type': kind} for name in names.split()}


CHANNELS = [
    channel(1, '/calibration/image', 'foxglove.CompressedImage', {
        # Match https://github.com/foxglove/schemas/blob/main/schemas/jsonschema/CompressedImage.json
        # In particular, "time" tells the decoder this object represents a timestamp.
        'timestamp': {'type': 'object', 'title': 'time', 'properties': {
            'sec': {'type': 'integer', 'minimum': 0},
            'nsec': {'type': 'integer', 'minimum': 0, 'maximum': 999999999}}},
        **fields('frame_id format', 'string'), 'data': {'type': 'string', 'contentEncoding': 'base64'},
    }, required=['timestamp', 'frame_id', 'data', 'format']),
    channel(2, '/calibration/status', 'rescue.CalibrationStatus', {
        **fields('started ready detected busy camera_fresh completed', 'boolean'),
        **fields('count total frame_sequence source_width source_height preview_width preview_height', 'integer'),
        **fields('step error session', 'string'),
        **fields('capture_fps preview_fps detection_fps detection_ms', 'number'),
        'detection_age_ms': {'type': ['number', 'null']}, 'frame_age_ms': {'type': ['number', 'null']},
    }),
    channel(3, '/calibration/result', 'rescue.CameraIntrinsics', {
        **fields('available', 'boolean'), **fields('status intrinsics_file note', 'string'),
        **fields('fx fy cx cy fit_rms_px', 'number'),
        'image_size': {'type': 'array', 'items': {'type': 'integer'}},
        'camera_matrix': {'type': 'array', 'items': {'type': 'array', 'items': {'type': 'number'}}},
        **{key: {'type': 'array', 'items': {'type': 'number'}} for key in ('dist_coeffs', 'holdout_rms_px')},
        'warnings': {'type': 'array', 'items': {'type': 'string'}},
    }),
]


def payloads(snapshot):
    state, jpeg, stamp = snapshot
    status = {key: state[key] for key in ('started', 'ready', 'detected', 'busy', 'count', 'total',
        'step', 'error', 'frame_sequence', 'frame_age_ms', 'camera_fresh',
        'capture_fps', 'preview_fps', 'detection_fps', 'detection_ms', 'detection_age_ms')}
    status.update(session=state['session'] or '', completed=state['result'] is not None,
        source_width=(state['image_size'] or state['requested_size'])[0],
        source_height=(state['image_size'] or state['requested_size'])[1],
        preview_width=state['preview_size'][0], preview_height=state['preview_size'][1])
    result = state['result']
    result_data = {'available': False}
    if result:
        result_data = {key: result[key] for key in ('status', 'intrinsics_file', 'note', 'image_size',
            'camera_matrix', 'dist_coeffs', 'fit_rms_px', 'holdout_rms_px', 'warnings')}
        k = result['camera_matrix']
        result_data.update(available=True, fx=k[0][0], fy=k[1][1], cx=k[0][2], cy=k[1][2])
    messages = {2: status, 3: result_data}
    # Stop sending images when stale; do not restamp the last frame as live.
    if jpeg and stamp and state['camera_fresh']:
        messages[1] = dict(timestamp={'sec': stamp//10**9, 'nsec': stamp % 10**9},
            frame_id='calibration_camera', format='jpeg', data=base64.b64encode(jpeg).decode())
    return messages, stamp, state['frame_sequence']


class FoxgloveBridge:
    def __init__(self, source, host='0.0.0.0', port=8766, fps=30):
        self.source, self.host, self.port, self.fps = source, host, port, fps
        self.stop = threading.Event()
        self.ready = threading.Event()
        self.error = None
        self.clients = set()
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()
        if not self.ready.wait(5):
            self.stop.set()
            raise RuntimeError('Foxglove 服务启动超时')
        if self.error: raise RuntimeError('Foxglove 服务启动失败：' + self.error)

    def close(self):
        self.stop.set()
        if self.thread: self.thread.join(timeout=4)

    def run(self):
        try: asyncio.run(self.listen())
        except Exception as exc:
            self.error = str(exc)
            self.ready.set()

    async def listen(self):
        async with serve(self.client, self.host, self.port, subprotocols=['foxglove.websocket.v1'],
                         compression=None, max_size=65536, max_queue=8, close_timeout=1) as server:
            self.port = server.sockets[0].getsockname()[1]
            self.ready.set()
            while not self.stop.is_set(): await asyncio.sleep(.1)

    async def client(self, ws):
        if ws.subprotocol != 'foxglove.websocket.v1':
            await ws.close(1002, 'Foxglove subprotocol required')
            return
        self.clients.add(ws)
        subscriptions, previous = {}, {}
        try:
            await ws.send(json.dumps(dict(op='serverInfo', name='ds_innovation 相机标定',
                capabilities=[], supportedEncodings=['json'], metadata={})))
            await ws.send(json.dumps(dict(op='advertise', channels=CHANNELS)))

            async def transmit():
                while not self.stop.is_set():
                    messages, stamp, seq = payloads(self.source.foxglove_snapshot())
                    encoded = {}
                    for sid, cid in list(subscriptions.items()):
                        if cid not in messages or (cid == 1 and previous.get(sid) == seq): continue
                        if cid not in encoded:
                            encoded[cid] = json.dumps(messages[cid], ensure_ascii=False, allow_nan=False,
                                                      separators=(',', ':')).encode()
                        packet = struct.pack('<BIQ', 1, sid, stamp if cid == 1 else time.time_ns()) + encoded[cid]
                        await asyncio.wait_for(ws.send(packet), timeout=2)
                        previous[sid] = seq
                    await asyncio.sleep(1/self.fps)

            async def receive():
                async for raw in ws:
                    try:
                        if not isinstance(raw, str): raise ValueError('Read-only server')
                        msg = json.loads(raw)
                        if not isinstance(msg, dict): raise ValueError('Invalid message')
                        if msg.get('op') == 'subscribe':
                            for sub in msg['subscriptions']:
                                sid, cid = sub['id'], sub['channelId']
                                if type(sid) is not int or not 0 <= sid <= 0xffffffff or type(cid) is not int or cid not in (1,2,3):
                                    raise ValueError('Invalid subscription')
                                if len(subscriptions) >= 12 and sid not in subscriptions: raise ValueError('Too many subscriptions')
                                subscriptions[sid] = cid; previous.pop(sid, None)
                        elif msg.get('op') == 'unsubscribe':
                            for sid in msg['subscriptionIds']:
                                subscriptions.pop(sid, None); previous.pop(sid, None)
                        else: raise ValueError('Only subscribe/unsubscribe are supported')
                    except (ValueError, KeyError, TypeError):
                        await ws.send(json.dumps(dict(op='status', level=1, message='Invalid or unsupported request')))

            tasks = [asyncio.create_task(transmit()), asyncio.create_task(receive())]
            try:
                await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            finally:
                for task in tasks: task.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)
        finally:
            self.clients.discard(ws)
