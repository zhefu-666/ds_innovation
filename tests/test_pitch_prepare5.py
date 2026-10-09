#!/usr/bin/env python3
"""Software checks for the zero-velocity, gripper-preserving pitch preflight."""
import importlib.util
from pathlib import Path
import struct
import unittest
import time
from unittest.mock import patch


path = Path(__file__).resolve().parents[1] / 'tools/camera_pitch/pitch_ctl.py'
spec = importlib.util.spec_from_file_location('pitch_ctl', path)
pitch_ctl = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pitch_ctl)
pitch_ctl.FRAME_MAPPING={"open":0,"close":1} # synthetic only


class FakeLink:
    dry_run = False

    def __init__(self, state=0, action_id=7, pitch=4500):
        self.state, self.action_id, self.pitch = state, action_id, pitch
        self.sent = []
        self.last = None

    def send(self, packet):
        self.sent.append(packet)

    def poll(self):
        body = struct.pack('<BBBh', 0xA6, self.state, self.action_id, self.pitch // 100)
        raw = body + struct.pack('<H', pitch_ctl.crc16(body)) + b'\x0a'
        self.last = (self.state, self.action_id, self.pitch, raw)
        return [self.last]


class PreparePitchTests(unittest.TestCase):
    def test_keeps_closed_gripper_and_sends_only_zero_speed(self):
        link = FakeLink()
        self.assertTrue(pitch_ctl.prepare_pitch_five(link, 0, 7, 1))
        self.assertTrue(link.sent)
        for packet in link.sent:
            self.assertEqual(struct.unpack('<BffbBh', packet[:13]), (0x56, 0.0, 0.0, 0, 7, 5))
            self.assertEqual(struct.unpack('<H', packet[13:])[0], pitch_ctl.crc16(packet[:13]))

    def test_raised_frame_maps_binary_feedback_to_twenty_degrees(self):
        link = FakeLink(state=1)
        self.assertTrue(pitch_ctl.prepare_pitch_five(link, 1, 7, 1))
        self.assertTrue(all(packet[9] == 20 and packet[10] == 7 for packet in link.sent))

    def test_changed_gripper_feedback_fails(self):
        link = FakeLink(state=1)
        self.assertFalse(pitch_ctl.prepare_pitch_five(link, 0, 7, 1))

    def test_zero_action_id_is_preserved(self):
        link = FakeLink(action_id=0)
        self.assertTrue(pitch_ctl.prepare_pitch_five(link, 0, 0, 1))
        self.assertTrue(all(packet[10] == 0 for packet in link.sent))

    def test_missing_or_stale_feedback_prevents_camera_tx(self):
        link=pitch_ctl.Link(-1,False)
        packet=pitch_ctl.motion_packet(0,7,4500)
        with patch.object(pitch_ctl.os,'write') as write:
            with self.assertRaises(TimeoutError):link.send(packet)
            link.last=(0,7,4500,b'');link.last_received=time.monotonic()-1
            with self.assertRaises(TimeoutError):link.send(packet)
            write.assert_not_called()

    def test_changed_state_or_id_prevents_camera_tx(self):
        link=pitch_ctl.Link(-1,False);link.last_received=time.monotonic()
        packet=pitch_ctl.motion_packet(0,7,4500)
        with patch.object(pitch_ctl.os,'write') as write:
            for feedback in [(1,7,4500,b''),(0,8,4500,b'')]:
                link.last=feedback
                with self.assertRaises(ValueError):link.send(packet)
            write.assert_not_called()

    def test_invalid_gripper_state_sends_nothing(self):
        link = FakeLink(state=2)
        with self.assertRaises(ValueError):
            pitch_ctl.prepare_pitch_five(link, 2, 0, 1)
        self.assertFalse(link.sent)


if __name__ == '__main__':
    unittest.main()
