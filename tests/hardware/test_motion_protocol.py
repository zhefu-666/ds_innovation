#!/usr/bin/env python3
"""Offline contract checks against the MCU MotionPacket supplied on 2026-10-24."""
import importlib.util
from pathlib import Path
import struct
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/demo'))
import demo_serial

spec = importlib.util.spec_from_file_location('send_velocity', ROOT / 'tools/send_velocity.py')
velocity = importlib.util.module_from_spec(spec)
spec.loader.exec_module(velocity)


class MotionProtocolTests(unittest.TestCase):
    def test_manual_and_preview_packers_match_firmware_golden_frames(self):
        for logical, offset, ident, expected in [
            (0, 5, 1, '56 00 00 00 00 00 00 00 00 05 01 00 00 9B 71'),
            (20, -5, 2, '56 00 00 00 00 00 00 00 00 FB 02 00 00 5A 99'),
        ]:
            golden = bytes.fromhex(expected)
            self.assertEqual(demo_serial.packet(0, 0, logical, ident, 120), golden)
            self.assertEqual(velocity.build_latest_packet(0, 0, offset, ident, 120), golden)

    def test_preview_raw_offsets_and_camera_boundaries(self):
        for offset in range(-5, 6):
            for pitch, wire in [(90, -30), (120, 0), (150, 30)]:
                packet = velocity.build_latest_packet(.1, -.25, offset, 7, pitch)
                self.assertEqual(len(packet), 15)
                self.assertEqual(struct.unpack_from('<b', packet, 9)[0], offset)
                self.assertEqual(struct.unpack_from('<h', packet, 11)[0], wire)
                self.assertEqual(packet[10], 7)
                self.assertEqual(int.from_bytes(packet[13:], 'little'), velocity.crc16_modbus(packet[:13]))

    def test_preview_rejects_old_logical_values_as_raw_wire_offsets(self):
        for offset in [-6, 6, 20]:
            with self.assertRaises(ValueError):
                velocity.build_latest_packet(0, 0, offset, 0, 120)

    def test_named_targets_and_neutral_are_distinct(self):
        for flags, offset in [(['--gripper', 'open'], 5), (['--gripper', 'closed'], -5),
                              (['--frame-offset', '0'], 0), ([], -5)]:
            args = velocity.make_parser().parse_args(flags)
            self.assertEqual(struct.unpack_from('<b', velocity.build_packet(args, 0, 0), 9)[0], offset)


    def test_removed_fields_and_old_packet_versions_are_rejected(self):
        import contextlib
        import io
        for flags in [['--yaw', '90'], ['--target-angle', '90'],
                      ['--protocol', 'latest19'], ['--protocol', 'latest17']]:
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                velocity.make_parser().parse_args(flags)

    def test_stop_and_nonfinite_speed_keep_fifteen_byte_layout(self):
        args = velocity.make_parser().parse_args(['--vx', '.1', '--wz', '.2'])
        self.assertEqual(args.protocol, 'latest15')
        for packet in [velocity.build_packet(args, 0, 0, stop=True),
                       velocity.build_latest_packet(float('nan'), .2, 5, 1, 120)]:
            self.assertEqual(len(packet), 15)
            self.assertEqual(struct.unpack_from('<ff', packet, 1), (0., 0.))
            self.assertEqual(int.from_bytes(packet[13:], 'little'), velocity.crc16_modbus(packet[:13]))

    def test_other_manual_senders_use_same_layout(self):
        for name, rel, args in [
            ('camera_console', 'tools/remote_debug/camera_angle_console.py', (120, 0, 0)),
        ]:
            spec = importlib.util.spec_from_file_location(name, ROOT / rel)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            result = module.packet(*args)
            packet = result[0] if isinstance(result, tuple) else result
            self.assertEqual(len(packet), 15)
            self.assertEqual(int.from_bytes(packet[13:], 'little'), velocity.crc16_modbus(packet[:13]))


if __name__ == '__main__':
    unittest.main()
