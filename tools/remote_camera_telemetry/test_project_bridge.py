import asyncio
import json
import os
import struct
import tempfile
import time
import unittest
from pathlib import Path
import websockets
from websockets.server import serve
from project_bridge import Source, read_packet, client, imu_view


def packet(stamp=None):
    stamp = time.time_ns() if stamp is None else stamp
    data = {'version': 1, 'sequence': 2, 'timestamp': {'sec': stamp // 10**9, 'nsec': stamp % 10**9},
            'frame_id': 'camera_optical', 'source_width': 1280, 'source_height': 720,
            'image_width': 640, 'image_height': 360, 'detections': [],
            'state': {'name': 'WAIT_START'}, 'motion': {'vx_mps': 0, 'wz_rps': 0, 'hardware_output_enabled': 0},
            'health': {'loop_fps': 20}, 'config': {'confidence': 0.5}}
    meta = json.dumps(data).encode()
    jpeg = b'\xff\xd8\xff\xd9'
    return b'RSTEL001' + struct.pack('<II', len(meta), len(jpeg)) + meta + jpeg

class PacketTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / 'snapshot'
    def tearDown(self):
        self.tmp.cleanup()
    def test_imu_uses_own_freshness_not_camera_freshness(self):
        data = {'imu': {'enabled': 1, 'connected': 1, 'measurements_valid': 1,
                        'received_monotonic_us': time.monotonic_ns() / 1000,
                        'timeout_ms': 200, 'rpy_rad': [0.1, 0.2, 0.3]}}
        view = imu_view(data, True)
        self.assertTrue(view['valid'])
        self.assertFalse(view['attitude_valid_for_control'])
        self.assertFalse(imu_view(data, False)['valid'])
        data['imu']['received_monotonic_us'] -= 300000
        self.assertFalse(imu_view(data, True)['fresh'])
        data['imu']['received_monotonic_us'] = time.monotonic_ns() / 1000
        data['imu']['measurements_valid'] = 0
        self.assertTrue(imu_view(data, True)['fresh'])
        self.assertFalse(imu_view(data, True)['valid'])
        data['imu']['connected'] = 0
        self.assertFalse(imu_view(data, True)['fresh'])
    def test_corruption_and_size_limits(self):
        for raw in [b'', packet()[:-1], packet()+b'X', b'RSTEL001'+struct.pack('<II', 2**31, 4)]:
            self.path.write_bytes(raw)
            with self.assertRaises(ValueError): read_packet(self.path)
    def atomic_write(self, content):
        pending = self.path.with_suffix('.tmp')
        pending.write_bytes(content)
        os.replace(pending, self.path)
    def test_stale_missing_and_recovery(self):
        source = Source(str(self.path), 10)
        source.poll()
        self.assertFalse(source.snapshot()[3]['robot_data_connected'])
        self.atomic_write(packet(time.time_ns()-3*10**9)); source.poll()
        self.assertFalse(source.snapshot()[3]['camera_ok'])
        self.atomic_write(packet()); source.poll()
        self.assertTrue(source.snapshot()[3]['robot_data_connected'])
        self.path.unlink(); source.poll()
        self.assertFalse(source.snapshot()[3]['robot_data_connected'])
        self.atomic_write(packet()); source.poll()
        self.assertTrue(source.snapshot()[3]['camera_ok'])

class ProtocolTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        path = Path(self.tmp.name) / 'snapshot'; path.write_bytes(packet())
        self.source = Source(str(path), 10); self.source.poll()
        self.server = await serve(lambda ws: client(ws, self.source), '127.0.0.1', 0,
                                  subprotocols=['foxglove.websocket.v1'])
        self.url = 'ws://127.0.0.1:' + str(self.server.sockets[0].getsockname()[1])
    async def asyncTearDown(self):
        self.server.close(); await self.server.wait_closed(); self.tmp.cleanup()
    async def connect(self):
        ws = await websockets.connect(self.url, subprotocols=['foxglove.websocket.v1'])
        info = json.loads(await ws.recv()); self.assertEqual(info['capabilities'], [])
        ad = json.loads(await ws.recv()); self.assertEqual(len(ad['channels']), 7)
        return ws
    async def test_all_channels_and_timestamp(self):
        ws = await self.connect()
        try:
            await ws.send(json.dumps({'op':'subscribe','subscriptions':[{'id':x,'channelId':x} for x in range(1,8)]}))
            received = {}
            while len(received) < 7:
                raw = await asyncio.wait_for(ws.recv(), 2)
                op, sid, stamp = struct.unpack('<BIQ', raw[:13]); self.assertEqual(op,1)
                received[sid] = json.loads(raw[13:])
                if sid not in (2,7): self.assertEqual(stamp, self.source.timestamp)
            self.assertTrue(received[2]['robot_data_connected'])
            self.assertEqual(received[4]['name'], 'WAIT_START')
            self.assertTrue(received[4]['valid'])
            self.assertEqual(received[5]['hardware_output_enabled'], 0)
            self.assertFalse(received[7]['valid'])
        finally:
            await ws.close()
    async def test_reject_control_and_reconnect(self):
        ws = await self.connect()
        await ws.send(json.dumps({'op':'setParameters','parameters':[]}))
        await asyncio.wait_for(ws.wait_closed(), 2)
        self.assertEqual(ws.close_code,1008)
        ws = await self.connect(); await ws.close()
    async def test_source_shutdown_closes_active_client(self):
        ws = await self.connect()
        await ws.send(json.dumps({'op':'subscribe','subscriptions':[{'id':1,'channelId':1}]}))
        await asyncio.wait_for(ws.recv(), 2)
        self.source.stop.set()
        await asyncio.wait_for(ws.wait_closed(), 3)
        self.assertEqual(ws.close_code, 1000)
    async def test_reject_invalid_subscription(self):
        ws = await self.connect()
        await ws.send(json.dumps({'op':'subscribe','subscriptions':[{'id':1,'channelId':99}]}))
        await asyncio.wait_for(ws.wait_closed(), 2)
        self.assertEqual(ws.close_code,1008)

if __name__ == '__main__': unittest.main()
