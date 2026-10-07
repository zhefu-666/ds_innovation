#!/usr/bin/env python3
"""Virtual-serial integration tests; never opens a hardware port."""
import errno
import importlib.util
import os
from pathlib import Path
import pty
import select
import struct
import subprocess
import sys
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'tools/frame/frame_ctl.py'
spec = importlib.util.spec_from_file_location('frame_ctl', SCRIPT)
ctl = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ctl)


def feedback(state, action_id, pitch):
    b = struct.pack('<BBBh', 0xa6, state, action_id, pitch)
    return b + struct.pack('<H', ctl.crc16(b)) + b'\x0a'


class FrameTests(unittest.TestCase):
    def exercise(self, action, state=0, action_id=0, pitch=40, dry=False, reply=True, legacy=False, emit=True):
        master, slave = pty.openpty()
        script = ROOT / 'tools/gripper/gripper_ctl.py' if legacy else SCRIPT
        cmd = [sys.executable, str(script), action, '--port', os.ttyname(slave), '--timeout', '.25', '--frame-config', str(ROOT / 'tests/fixtures/frame_mapping.synthetic.json')]
        if dry:
            cmd.append('--dry-run')
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        received = bytearray()
        packets = []
        start = time.monotonic()
        last_emit = 0
        response_state, response_id = state, action_id
        try:
            while process.poll() is None and time.monotonic()-start < 3:
                now = time.monotonic()
                if emit and now-last_emit > .02:
                    os.write(master, feedback(response_state, response_id, pitch))
                    last_emit = now
                if select.select([master], [], [], .005)[0]:
                    data = os.read(master, 4096)
                    # PTY can echo feedback before child switches terminal to raw.
                    received.extend(data)
                    while received:
                        if received[0] != 0x56:
                            del received[0]
                            continue
                        if len(received) < 15:
                            break
                        packet = bytes(received[:15]);del received[:15]
                        packets.append(packet)
                        if reply:
                            response_state = 1 if packet[9] == 20 else 0
                            response_id = packet[10]
            if process.poll() is None:
                process.kill()
                self.fail('script hung')
            out, err = process.communicate(timeout=1)
            return process.returncode, packets, out+err
        finally:
            if process.poll() is None:
                process.kill();process.wait()
            os.close(master);os.close(slave)

    def assert_packet(self, packet, offset, action_id, pitch):
        self.assertEqual(struct.unpack('<BffbBh', packet[:13]), (0x56, 0., 0., offset, action_id, pitch-40))
        self.assertEqual(int.from_bytes(packet[13:], 'little'), ctl.crc16(packet[:13]))

    def test_up(self):
        code, packets, text = self.exercise('up')
        self.assertEqual(code, 0, text);self.assertTrue(packets)
        for packet in packets:self.assert_packet(packet, 0, 1, 40)

    def test_down_and_id_wrap(self):
        code, packets, text = self.exercise('down', state=1, action_id=255, pitch=5)
        self.assertEqual(code, 0, text);self.assertTrue(packets)
        for packet in packets:self.assert_packet(packet, 20, 1, 5)

    def test_dry_run_has_no_tx(self):
        code, packets, text = self.exercise('up', dry=True)
        self.assertEqual(code, 0, text);self.assertEqual(packets, [])
        self.assertIn('TX=0', text)

    def test_status_has_no_tx(self):
        code, packets, text = self.exercise('status')
        self.assertEqual(code, 0, text);self.assertEqual(packets, [])

    def test_matching_state_still_sends_fresh_command(self):
        for action, state, offset in [('down', 1, 20), ('up', 0, 0)]:
            code, packets, text = self.exercise(action, state=state, action_id=2)
            self.assertEqual(code, 0, text);self.assertTrue(packets)
            for packet in packets:self.assert_packet(packet, offset, 3, 40)

    def test_same_state_old_id_cannot_acknowledge_new_command(self):
        code, packets, text = self.exercise('up', state=1, action_id=2, reply=False)
        self.assertEqual(code, 1, text);self.assertTrue(packets)
        for packet in packets:self.assert_packet(packet, 0, 3, 40)
        self.assertIn('反馈超时', text)

    def test_missing_ack_times_out_without_reverse(self):
        code, packets, text = self.exercise('up', reply=False)
        self.assertEqual(code, 1, text);self.assertGreater(len(packets), 1)
        self.assertEqual(len(set(packets)), 1)
        self.assertIn('反馈超时', text)

    def test_eighty_degree_pitch_is_preserved(self):
        code, packets, text = self.exercise('up', pitch=80)
        self.assertEqual(code, 0, text);self.assertTrue(packets)
        for packet in packets:self.assert_packet(packet, 0, 1, 80)

    def test_out_of_range_pitch_no_tx(self):
        for pitch in [-1, 81]:
            code, packets, text = self.exercise('up', pitch=pitch)
            self.assertEqual(code, 1, text);self.assertEqual(packets, [])

    def test_invalid_pitch_no_tx(self):
        code, packets, text = self.exercise('up', pitch=-32768)
        self.assertEqual(code, 1, text);self.assertEqual(packets, [])

    def test_no_feedback_no_tx(self):
        code, packets, text = self.exercise('up', emit=False)
        self.assertEqual(code, 1, text);self.assertEqual(packets, [])

    def test_old_open_entry(self):
        code, packets, text = self.exercise('open', legacy=True)
        self.assertEqual(code, 0, text);self.assertTrue(packets)
        self.assert_packet(packets[0], 0, 1, 40)

    def test_parser_split_noise_and_crc(self):
        parser = ctl.Feedback();f = feedback(1, 10, 40)
        self.assertEqual(parser.feed(b'noise'+f[:3]), [])
        self.assertEqual(parser.feed(f[3:])[0][:3], (1, 10, 40))
        bad = bytearray(f);bad[5] ^= 1
        self.assertEqual(len(parser.feed(bad+f)), 1)


if __name__ == '__main__':
    unittest.main()
