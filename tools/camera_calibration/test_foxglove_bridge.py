import asyncio
import base64
import json
import struct
import time
from types import SimpleNamespace
import unittest

import cv2
import numpy as np
from websockets.client import connect
from guided import Wizard
from foxglove_bridge import FoxgloveBridge, payloads


class BridgeTests(unittest.TestCase):
    def make_source(self):
        source = Wizard(SimpleNamespace(width=1280,height=720))
        source.size=(1280,720);source.preview_size=(960,540)
        source.preview_time=source.frame_time=time.monotonic();source.frame_timestamp_ns=time.time_ns();source.frame_sequence=8
        source.jpeg=cv2.imencode('.jpg',np.full((540,960,3),200,np.uint8))[1].tobytes()
        return source

    def test_stale_images_not_republished_and_result_schema(self):
        source=self.make_source()
        messages, stamp, seq=payloads(source.foxglove_snapshot())
        self.assertIn(1,messages);self.assertEqual(seq,8)
        self.assertEqual(messages[1]['timestamp']['sec']*10**9+messages[1]['timestamp']['nsec'],stamp)
        source.preview_time=source.frame_time=time.monotonic()-3
        messages,_,_=payloads(source.foxglove_snapshot())
        self.assertNotIn(1,messages);self.assertFalse(messages[2]['camera_fresh'])
        source.result=dict(status='candidate_requires_validation',intrinsics_file='/tmp/intrinsics.yaml',note='test',
            image_size=[1280,720],camera_matrix=[[900,0,640],[0,901,360],[0,0,1]],dist_coeffs=[0]*5,
            fit_rms_px=.1,holdout_rms_px=[.1,.2,.3],warnings=[])
        messages,_,_=payloads(source.foxglove_snapshot())
        self.assertTrue(messages[3]['available']);self.assertEqual(messages[3]['fy'],901)

    def test_wire_subscribe_decode_unsubscribe_and_idle(self):
        source=self.make_source();bridge=FoxgloveBridge(source,'127.0.0.1',0,10);bridge.start()
        async def check():
            async with connect(f'ws://127.0.0.1:{bridge.port}',subprotocols=['foxglove.websocket.v1']) as ws:
                self.assertEqual(json.loads(await ws.recv())['op'],'serverInfo')
                channels=json.loads(await ws.recv())['channels']
                self.assertEqual([c['topic'] for c in channels],['/calibration/image','/calibration/status','/calibration/result'])
                for c in channels:
                    schema=json.loads(c['schema'])
                    self.assertEqual(schema['type'],'object')
                    self.assertEqual(schema['title'],c['schemaName'])
                image_schema=json.loads(channels[0]['schema'])
                self.assertEqual(image_schema['properties']['timestamp']['title'],'time')
                self.assertEqual(set(image_schema['required']),{'timestamp','frame_id','data','format'})
                await ws.send(json.dumps(dict(op='subscribe',subscriptions=[{'id':31,'channelId':1},{'id':32,'channelId':2},{'id':33,'channelId':3}])))
                received={}
                while len(received)<3:
                    packet=await asyncio.wait_for(ws.recv(),2)
                    op,sid,stamp=struct.unpack('<BIQ',packet[:13]);self.assertEqual(op,1)
                    received[sid]=json.loads(packet[13:])
                    if sid==31:self.assertEqual(stamp,source.frame_timestamp_ns)
                jpeg=base64.b64decode(received[31]['data'])
                self.assertEqual(cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR).shape[:2],(540,960))
                self.assertTrue(received[32]['camera_fresh']);self.assertFalse(received[33]['available'])
                # Reuse the image subscription ID for status. Verify fresh payload and no image confusion.
                await ws.send(json.dumps(dict(op='unsubscribe',subscriptionIds=[31,32,33])))
                await ws.send(json.dumps(dict(op='subscribe',subscriptions=[{'id':31,'channelId':2}])))
                source.preview_time=source.frame_time=time.monotonic()-5
                for _ in range(20):
                    packet=await asyncio.wait_for(ws.recv(),2)
                    sid=struct.unpack('<BIQ',packet[:13])[1];body=json.loads(packet[13:])
                    if sid==31 and 'camera_fresh' in body:
                        self.assertFalse(body['camera_fresh']);break
                else:self.fail('Did not receive replacement subscription')
                await ws.send(json.dumps(dict(op='subscribe',subscriptions=[{'id':99,'channelId':999}])))
                for _ in range(20):
                    packet=await asyncio.wait_for(ws.recv(),2)
                    if isinstance(packet,str):
                        self.assertEqual(json.loads(packet)['op'],'status');break
                else:self.fail('Invalid subscription was not rejected')
        try:asyncio.run(check())
        finally:bridge.close()
        self.assertFalse(bridge.thread.is_alive())


if __name__=='__main__': unittest.main()
