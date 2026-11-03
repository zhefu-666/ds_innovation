#!/usr/bin/env python3
"""Software checks for the zero-velocity, gripper-preserving pitch preflight."""
import importlib.util
from pathlib import Path
import struct
import unittest
import time
from unittest.mock import patch


path = Path(__file__).resolve().parents[2] / 'tools/camera_pitch/pitch_ctl.py'
spec = importlib.util.spec_from_file_location('pitch_ctl', path)
pitch_ctl = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pitch_ctl)
pitch_ctl.FRAME_MAPPING={"open":1,"close":0} # synthetic only


class FakeLink:
    dry_run = False

    def __init__(self, state=0, action_id=7, pitch=12500):
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
            self.assertEqual(len(packet), 15)
            self.assertEqual(struct.unpack('<BffbBh', packet[:13]), (0x56, 0.0, 0.0, -5, 7, 5))
            self.assertEqual(struct.unpack('<H', packet[13:])[0], pitch_ctl.crc16(packet[:13]))

    def test_raised_frame_maps_binary_feedback_to_twenty_degrees(self):
        link = FakeLink(state=1)
        self.assertTrue(pitch_ctl.prepare_pitch_five(link, 1, 7, 1))
        self.assertTrue(all(packet[9] == 5 and packet[10] == 7 for packet in link.sent))

    def test_changed_gripper_feedback_fails(self):
        link = FakeLink(state=1)
        self.assertFalse(pitch_ctl.prepare_pitch_five(link, 0, 7, 1))

    def test_zero_action_id_is_preserved(self):
        link = FakeLink(action_id=0)
        self.assertTrue(pitch_ctl.prepare_pitch_five(link, 0, 0, 1))
        self.assertTrue(all(packet[10] == 0 for packet in link.sent))

    def test_missing_or_stale_feedback_prevents_camera_tx(self):
        link=pitch_ctl.Link(-1,False)
        packet=pitch_ctl.motion_packet(0,7,12500)
        with patch.object(pitch_ctl.os,'write') as write:
            with self.assertRaises(TimeoutError):link.send(packet)
            link.last=(0,7,12500,b'');link.last_received=time.monotonic()-1
            with self.assertRaises(TimeoutError):link.send(packet)
            write.assert_not_called()

    def test_changed_state_or_id_prevents_camera_tx(self):
        link=pitch_ctl.Link(-1,False);link.last_received=time.monotonic()
        packet=pitch_ctl.motion_packet(0,7,12500)
        with patch.object(pitch_ctl.os,'write') as write:
            for feedback in [(1,7,12500,b''),(0,8,12500,b'')]:
                link.last=feedback
                with self.assertRaises(ValueError):link.send(packet)
            write.assert_not_called()

    def test_fresh_binary_feedback_allows_matching_signed_target(self):
        for state, offset in [(0, -5), (1, 5)]:
            link=pitch_ctl.Link(-1,False)
            link.last=(state,7,12500,b'')
            link.last_received=time.monotonic()
            packet=pitch_ctl.motion_packet(state,7,12500)
            self.assertEqual(struct.unpack_from('<b',packet,9)[0], offset)
            with patch.object(pitch_ctl.os,'write') as write:
                link.send(packet)
                write.assert_called_once_with(-1,packet)

    def test_status_watch_and_dry_run_open_mcu_readonly(self):
        mapping = {"action_mapping_verified": 1, "a6_semantics": "done_flag", "a6_done_settle_ms": 100}
        saved = (pitch_ctl.FRAME_MAPPING, pitch_ctl.FRAME_KNOWN, pitch_ctl.FRAME_HOLD_OPEN)
        try:
            with patch.object(pitch_ctl, 'open_port', return_value=9) as opened, \
                 patch.object(pitch_ctl, 'wait_feedback', return_value=None), \
                 patch.object(pitch_ctl, 'close_port'), \
                 patch.object(pitch_ctl, 'load_mapping', return_value=mapping):
                cases = [
                    (['pitch_ctl.py', 'status', '--port', 'fake'], True),
                    (['pitch_ctl.py', 'watch', '--port', 'fake'], True),
                    (['pitch_ctl.py', 'set', '125', '--dry-run', '--port', 'fake',
                      '--known-frame-angle', '0', '--known-frame-id', '1'], True),
                    (['pitch_ctl.py', 'set', '125', '--port', 'fake',
                      '--known-frame-angle', '0', '--known-frame-id', '1'], False),
                ]
                for argv, expected in cases:
                    pitch_ctl.sys.argv = argv
                    with self.assertRaises(SystemExit):
                        pitch_ctl.main()
                    self.assertEqual(opened.call_args.args[0], 'fake')
                    self.assertIs(opened.call_args.args[2], expected)
        finally:
            pitch_ctl.FRAME_MAPPING, pitch_ctl.FRAME_KNOWN, pitch_ctl.FRAME_HOLD_OPEN = saved

    def test_invalid_gripper_state_sends_nothing(self):
        link = FakeLink(state=2)
        with self.assertRaises(ValueError):
            pitch_ctl.prepare_pitch_five(link, 2, 0, 1)
        self.assertFalse(link.sent)


class PitchHoldSemanticsTests(unittest.TestCase):
    """Exercise CLI setup, packet generation and Link.send without a serial device."""

    def setUp(self):
        for name, value in [('FRAME_MAPPING', {'semantics': 'done_flag', 'settle_ms': 100}),
                            ('FRAME_KNOWN', None), ('FRAME_HOLD_OPEN', None)]:
            patcher = patch.object(pitch_ctl, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)

    @staticmethod
    def feedback(state=1, action_id=2, pitch=90):
        body = struct.pack('<BBBh', 0xA6, state, action_id, pitch)
        raw = body + struct.pack('<H', pitch_ctl.crc16(body)) + b'\x0a'
        return pitch_ctl.Feedback().feed(raw)[0]

    def run_cli(self, angle=20, action='set', observed_id=2, observed_state=1, mapping=None):
        if mapping is None:
            mapping = {'semantics': 'done_flag', 'settle_ms': 100}
        target = 125 if action in ('prepare5', 'prepare45') else 90
        initial_state = 1 if pitch_ctl.done_flag(mapping) else mapping['open' if angle == 0 else 'close']
        argv = ['pitch_ctl.py', action]
        if action == 'set':
            argv += ['90', '--watch', '0.1']
        if pitch_ctl.done_flag(mapping):
            argv += ['--known-frame-angle', str(angle), '--known-frame-id', '2']
        self.sent = []
        clock = [10.0]
        def advance(seconds):
            clock[0] += seconds
        def capture(fd, packet):
            self.sent.append(packet)
            return len(packet)
        with patch.object(pitch_ctl.sys, 'argv', argv), \
             patch.object(pitch_ctl, 'load_mapping', return_value=mapping), \
             patch.object(pitch_ctl, 'open_port', return_value=-1), \
             patch.object(pitch_ctl, 'close_port'), \
             patch.object(pitch_ctl, 'wait_feedback', return_value=self.feedback(initial_state, 2, 120)), \
             patch.object(pitch_ctl, 'read_frames', return_value=[self.feedback(observed_state, observed_id, target)]), \
             patch.object(pitch_ctl.os, 'write', side_effect=capture), \
             patch.object(pitch_ctl.time, 'time', side_effect=lambda: clock[0]), \
             patch.object(pitch_ctl.time, 'monotonic', side_effect=lambda: clock[0]), \
             patch.object(pitch_ctl.time, 'sleep', side_effect=advance):
            pitch_ctl.main()
        return self.sent

    def assert_packets_hold(self, packets, offset, pitch):
        self.assertGreater(len(packets), 1)  # includes holding after reaching the target
        for packet in packets:
            self.assertEqual(struct.unpack('<BffbBh', packet[:13]), (0x56, 0.0, 0.0, offset, 2, pitch - 120))
            self.assertEqual(int.from_bytes(packet[13:], 'little'), pitch_ctl.crc16(packet[:13]))

    def test_set_preserves_known_closed_target_through_settle_and_watch(self):
        self.assert_packets_hold(self.run_cli(angle=20), -5, 90)

    def test_set_preserves_known_open_target(self):
        self.assert_packets_hold(self.run_cli(angle=0), 5, 90)

    def test_prepare_aliases_preserve_known_closed_target(self):
        for action in ('prepare45', 'prepare5'):
            with self.subTest(action=action):
                self.assert_packets_hold(self.run_cli(action=action), -5, 125)

    def test_explicit_state_mapping_is_used_instead_of_binary_open_assumption(self):
        self.assert_packets_hold(self.run_cli(angle=20, mapping={'open': 0, 'close': 1}), -5, 90)

    def test_unexpected_id_change_still_stops_before_next_write(self):
        with self.assertRaises(ValueError):
            self.run_cli(observed_id=3)
        self.assertEqual(len(self.sent), 1)
        self.assertEqual(struct.unpack_from('<b', self.sent[0], 9)[0], -5)

    def test_incomplete_done_flag_still_stops_before_next_write(self):
        with self.assertRaises(ValueError):
            self.run_cli(observed_state=0)
        self.assertEqual(len(self.sent), 1)

    def test_send_rejects_open_packet_when_known_target_is_closed(self):
        pitch_ctl.FRAME_KNOWN = (20, 2)
        link = pitch_ctl.Link(-1, False)
        link.last = self.feedback()
        link.last_received = time.monotonic()
        with patch.object(pitch_ctl.os, 'write') as write:
            with self.assertRaises(ValueError):
                link.send(pitch_ctl.motion_packet(1, 2, 9000))
            write.assert_not_called()


if __name__ == '__main__':
    unittest.main()
