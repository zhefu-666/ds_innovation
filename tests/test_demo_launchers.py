#!/usr/bin/env python3
import argparse
import contextlib
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import pty
import select
import struct
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools/demo'))
import demo
import demo_serial


class DemoTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.profile = demo.load_profile(ROOT/'config/demo_20261007.json')
        for key in ('binary', 'model', 'pose_model', 'rknn_library', 'camera_calibration', 'zone_geometry'):
            path = self.root/key;path.write_text('test fixture, not a live resource')
            self.profile[key] = str(path)
        frame = self.root/'frame.json'
        frame.write_text(json.dumps({'action_mapping_verified': 1, 'a6_open_state': 1,
                                     'a6_close_state': 0, 'box_area_accepted': 1}))
        accept = self.root/'acceptance.json'
        accept.write_text(json.dumps({'stationary_mapping_trials_passed': True, 'watchdog_verified': True,
                                     'watchdog_max_ms': 100, 'manual_wheel_test_passed': True,
                                     'ground_geometry_verified': True, 'multi_view_dataset_passed': True,
                                     'operator': 'synthetic', 'verified_date': 'synthetic', 'evidence_record': 'test_only'}))
        self.profile['frame_config'] = str(frame);self.profile['acceptance'] = str(accept)

    def tearDown(self):
        self.tmp.cleanup()

    def plan(self, action, *args):
        parsed = demo.parser().parse_args([action, *args])
        return parsed, demo.make_plan(parsed, self.profile)

    def test_recognition_has_no_serial_or_start_flags(self):
        _, plan = self.plan('recognize')
        command = plan['command']
        self.assertIn('--dry-run', command)
        for flag in ('--hardware', '--imu', '--pitch-feedback', '--auto-run', '--controlled-empty-field'):
            self.assertNotIn(flag, command)
        self.assertFalse(plan['issues'])

    def test_search_preview_has_only_receive_ports_and_no_motion(self):
        _, plan = self.plan('search', '--preview')
        self.assertIn('--pitch-feedback', plan['command'])
        self.assertIn('--dry-run', plan['command'])
        self.assertNotIn('--hardware', plan['command'])
        self.assertNotIn('--auto-run', plan['command'])
        self.assertNotIn('--task-calibration', plan['command'])
        self.assertIsNone(plan['native_check'])

    def test_carry_and_search_have_different_native_modes(self):
        for action, mode in [('search', 'search'), ('carry', 'carry_once')]:
            _, plan = self.plan(action)
            command = plan['command'];self.assertEqual(command[command.index('--demo-mode')+1], mode)
            self.assertEqual(command[command.index('--startup-advance-ms')+1], '0')
            self.assertEqual(command[command.index('--scan-wz')+1], '.25')
            self.assertIn('--check-config', plan['native_check'])
            self.assertNotIn('--no-match-time-limit', command)
            self.assertNotIn('--controlled-ignore-clearance', command)

    def test_camera_converts_at_tool_boundary(self):
        _, plan = self.plan('camera', '--offset-deg', '5')
        command = plan['command'];self.assertEqual(command[command.index('set')+1], '45')
        self.assertIn('--frame-config', command)

    def test_reject_limits_and_invalid_mode_combinations(self):
        for action, flags in [('move-forward', ['--seconds', '1.1']), ('carry', ['--seconds', '61']),
                              ('move-forward', ['--speed', 'nan']), ('turn-left', ['--wz', '.3']),
                              ('carry', ['--preview']), ('recognize', ['--seconds', '1.5'])]:
            with self.assertRaises(ValueError):self.plan(action, *flags)

    def test_pending_acceptance_blocks_motion_but_not_recognition(self):
        Path(self.profile['acceptance']).write_text('{}')
        for action in ('move-forward', 'search', 'carry'):
            self.assertTrue(self.plan(action)[1]['issues'])
        self.assertFalse(self.plan('recognize')[1]['issues'])

    def test_missing_site_attestation_rejects_before_native_check(self):
        args, plan = self.plan('carry', '--run')
        with patch.object(demo.subprocess, 'run') as call:
            with self.assertRaises(ValueError):demo.run(args, self.profile, plan)
            call.assert_not_called()

    def test_commission_requires_actual_wheel_disable_attestation(self):
        args, plan = self.plan('commission-open', '--run', '--site-clear')
        with patch.object(demo_serial, 'commission') as call:
            with self.assertRaises(ValueError):demo.run(args, self.profile, plan)
            call.assert_not_called()

    def test_default_checks_never_start_processes_or_open_serial(self):
        with patch.object(demo, 'load_profile', return_value=self.profile), patch.object(demo.subprocess, 'Popen') as proc, patch.object(demo_serial.frame, 'serial_port') as port, contextlib.redirect_stdout(io.StringIO()):
            for action in ('recognize', 'camera', 'open', 'move-forward', 'commission-open', 'web'):
                self.assertEqual(demo.main([action]), 0)
            proc.assert_not_called();port.assert_not_called()

    def test_stop_ignores_reused_or_invalid_pid(self):
        Path(self.tmp.name, 'session.json').write_text(json.dumps({'pid': 555, 'start_ticks': 'old'}))
        with patch.object(demo, 'private_runtime', return_value=Path(self.tmp.name)), patch.object(demo, 'start_ticks', return_value='new'), patch.object(demo.os, 'kill') as kill, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(demo.control(True), 2);kill.assert_not_called()

    def done_profile(self):
        Path(self.profile['frame_config']).write_text(json.dumps({'action_mapping_verified': 1, 'a6_semantics': 'done_flag',
                                                                   'a6_done_settle_ms': 900, 'box_area_accepted': 1}))

    def test_done_flag_requires_known_frame_and_passes_it_to_native(self):
        self.done_profile()
        with patch.object(demo, 'KNOWN_FILE', self.root/'missing.json'):
            _, plan = self.plan('carry')
            self.assertTrue(any('done_flag' in i for i in plan['issues']))
            _, plan = self.plan('carry', '--known-frame-angle', '20', '--known-frame-id', '25')
        self.assertEqual(plan['issues'], [])
        cmd = plan['command'];i = cmd.index('--known-frame-angle')
        self.assertEqual(cmd[i:i+4], ['--known-frame-angle', '20', '--known-frame-id', '25'])
        self.assertEqual(plan['native_check'][-1], '--check-config');self.assertIn('--known-frame-id', plan['native_check'])
        with self.assertRaises(ValueError):self.plan('carry', '--known-frame-angle', '20')
        record = self.root/'known.json';record.write_text(json.dumps({'angle': 0, 'id': 7}))
        with patch.object(demo, 'KNOWN_FILE', record):
            _, plan = self.plan('search')
        self.assertIn('--known-frame-id', plan['command']);self.assertEqual(plan['command'][plan['command'].index('--known-frame-id')+1], '7')
        _, plan = self.plan('open');self.assertEqual(plan['issues'], [])

    def test_done_flag_manual_motion_needs_flag_one_and_matching_id(self):
        import frame_semantics as fs
        m = {'semantics': 'done_flag', 'settle_ms': 900}
        self.assertEqual(fs.hold_angle(1, m, (20, 5)), 20)
        for state, known in [(0, (20, 5)), (1, None)]:
            with self.assertRaises(ValueError):fs.hold_angle(state, m, known)
        with self.assertRaises(ValueError):fs.check_known_id(6, (20, 5))

    def test_packet_limits_crc_and_angle(self):
        for vx, wz in [(float('nan'), 0), (.11, 0), (0, .26)]:
            with self.assertRaises(ValueError):demo_serial.packet(vx, wz, 0, 7, 45)
        packet = demo_serial.packet(.05, -.2, 20, 7, 45)
        self.assertEqual(len(packet), 15);self.assertEqual(packet[9:11], bytes([20, 7]))
        self.assertEqual(struct.unpack('<h', packet[11:13])[0], 5)
        self.assertEqual(int.from_bytes(packet[13:], 'little'), demo_serial.frame.crc16(packet[:13]))

    def exercise_motion(self, stop_feedback=False, wrong_id=False):
        master, slave = pty.openpty();stop = threading.Event();packets = []
        def feedback():
            buf = bytearray();started = time.monotonic()
            while not stop.is_set():
                if not stop_feedback or not packets:
                    ident = 8 if wrong_id and packets else 7
                    raw = struct.pack('<BBBh', 0xa6, 1, ident, 45)
                    raw += struct.pack('<H', demo_serial.frame.crc16(raw))+b'\n'
                    os.write(master, raw)
                if select.select([master], [], [], .01)[0]:
                    try:buf.extend(os.read(master, 4096))
                    except OSError:return
                    while buf:
                        if buf[0] != 0x56:del buf[0];continue
                        if len(buf)<15:break
                        packet=bytes(buf[:15]);del buf[:15]
                        if demo_serial.frame.crc16(packet[:13])==int.from_bytes(packet[13:],'little'):packets.append(packet)
                if time.monotonic()-started>3:return
        worker = threading.Thread(target=feedback);worker.start()
        error = None
        try:
            try:demo_serial.manual_motion(os.ttyname(slave),115200,{'open':1,'close':0},.05,0,.35,lambda _:None)
            except (ValueError,TimeoutError) as exc:error=exc
            time.sleep(.05)
        finally:
            stop.set();worker.join();os.close(master);os.close(slave)
        self.assertTrue(packets)
        self.assertTrue(any(struct.unpack('<f', p[1:5])[0]>.01 for p in packets))
        self.assertEqual(struct.unpack('<ff',packets[-1][1:9]),(0.,0.))
        self.assertTrue(all(p[9]==0 and p[10]==7 for p in packets))
        return error

    def test_virtual_serial_motion_ends_with_zero_and_preserves_open(self):
        self.assertIsNone(self.exercise_motion())

    def test_virtual_serial_stale_feedback_stops_early(self):
        self.assertIsInstance(self.exercise_motion(stop_feedback=True),TimeoutError)

    def test_virtual_serial_changed_action_id_stops_early(self):
        self.assertIsInstance(self.exercise_motion(wrong_id=True),ValueError)


if __name__ == '__main__':
    unittest.main()
