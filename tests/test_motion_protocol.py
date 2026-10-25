#!/usr/bin/env python3
"""Offline contract checks against the MCU MotionPacket supplied on 2026-10-24."""
import importlib.util
from pathlib import Path
import struct
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/demo'))
import demo_serial

spec = importlib.util.spec_from_file_location('send_velocity', ROOT / 'tools/send_velocity.py')
velocity = importlib.util.module_from_spec(spec)
spec.loader.exec_module(velocity)


class MotionProtocolTests(unittest.TestCase):
    def test_manual_and_preview_packers_match_firmware_golden_frames(self):
        for logical, offset, ident, expected in [
            (0, 5, 1, '56 00 00 00 00 00 00 00 00 05 01 00 00 00 80 AA B4'),
            (20, -5, 2, '56 00 00 00 00 00 00 00 00 FB 02 00 00 00 80 FB 6A'),
        ]:
            golden = bytes.fromhex(expected)
            self.assertEqual(demo_serial.packet(0, 0, logical, ident, 120), golden)
            self.assertEqual(velocity.build_latest_packet(0, 0, offset, ident, 120), golden)

    def test_preview_raw_offsets_and_camera_boundaries(self):
        for offset in range(-5, 6):
            for pitch, wire in [(90, -30), (120, 0), (150, 30)]:
                packet = velocity.build_latest_packet(.1, -.25, offset, 7, pitch)
                self.assertEqual(len(packet), 17)
                self.assertEqual(struct.unpack_from('<b', packet, 9)[0], offset)
                self.assertEqual(struct.unpack_from('<h', packet, 11)[0], wire)
                self.assertEqual(packet[10], 7)
                self.assertEqual(struct.unpack_from('<h', packet, 13)[0], velocity.IMU_YAW_INVALID)
                self.assertEqual(int.from_bytes(packet[15:], 'little'), velocity.crc16_modbus(packet[:15]))

    def test_preview_yaw_field_is_encoded_or_explicitly_invalid(self):
        packet = velocity.build_latest_packet(0, 0, 5, 1, 120, yaw_deg=90.0)
        self.assertEqual(struct.unpack_from('<h', packet, 13)[0], 9000)
        self.assertEqual(int.from_bytes(packet[15:], 'little'), velocity.crc16_modbus(packet[:15]))
        packet = velocity.build_latest_packet(0, 0, 5, 1, 120, yaw_deg=-45.5)
        self.assertEqual(struct.unpack_from('<h', packet, 13)[0], -4550)
        packet = velocity.build_latest_packet(0, 0, 5, 1, 120, yaw_deg=190.0)  # 归一到 -170°
        self.assertEqual(struct.unpack_from('<h', packet, 13)[0], -17000)

    def test_preview_rejects_old_logical_values_as_raw_wire_offsets(self):
        for offset in [-6, 6, 20]:
            with self.assertRaises(ValueError):
                velocity.build_latest_packet(0, 0, offset, 0, 120)

    def test_named_targets_and_neutral_are_distinct(self):
        for flags, offset in [(['--gripper', 'open'], 5), (['--gripper', 'closed'], -5),
                              (['--frame-offset', '0'], 0), ([], -5)]:
            args = velocity.make_parser().parse_args(flags)
            self.assertEqual(struct.unpack_from('<b', velocity.build_packet(args, 0, 0), 9)[0], offset)


if __name__ == '__main__':
    unittest.main()
