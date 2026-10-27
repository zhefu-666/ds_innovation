"""Rotation hand-eye recovery, degeneracy, timestamps, recording and guided flow."""
import json
import math
import os
from pathlib import Path
import pty
import select
import struct
import subprocess
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import calibrate
import guided
import handeye as h

ROOT = Path(__file__).resolve().parents[2]


def synthetic_session(root, single_axis=False):
    k = np.array([[800., 0, 640], [0, 800, 360], [0, 0, 1]])
    d = np.zeros(5)
    calibrate.write_yaml(root/'intrinsics.yaml', k, d, (1280, 720))
    x = h.exp([.3, -.2, .7])
    p0 = h.exp([.12, -.16, .03])
    axes = [[0, 0, 0], [0, 0, 15], [0, 0, -15], [0, 0, 0],
            [0, 15, 0], [0, -15, 0], [0, 0, 0], [15, 0, 0], [-15, 0, 0],
            [0, 0, 0], [0, 10, 15], [0, -10, -15], [0, 0, 0], [10, 10, 0],
            [-10, -10, 0], [0, 0, 0], [0, 0, 20], [0, 0, 0], [0, 0, -20],
            [0, 0, 0], [0, 20, 0], [0, -20, 0], [15, 0, 10], [-15, 0, -10], [0, 0, 0]]
    if single_axis:
        axes = [[0, 0, (i % 3-1)*20] for i in range(25)]
    qs = [h.exp(np.radians(axis)) for axis in axes]
    stamps = np.arange(25)*5.+6.
    bias = np.array([.001, -.002, .0015])
    t = np.arange(0, stamps[-1]+1, .01)
    gyro = np.tile(bias, (len(t), 1))
    for j in range(1, 25):
        # Smooth unit-area cosine rate pulse on a 2-second interval.
        start = stamps[j-1]+.5
        mask = (t >= start) & (t <= start+2)
        pulse = (1-np.cos(math.pi*(t[mask]-start)))/2
        gyro[mask] += pulse[:, None]*h.log(qs[j-1].T @ qs[j])
    rows = [dict(received_us=int(round((1000+ts)*1e6)), device_time_ms=int(round(ts*1000)),
                 sequence=i+1, measurements_valid=True, gyro_rad_s=w.tolist(),
                 accel_m_s2=[0., 0., 9.8], quaternion_wxyz=[1, 0, 0, 0])
            for i, (ts, w) in enumerate(zip(t, gyro))]
    (root/'imu.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    obj = calibrate.object_grid((9, 6), .035)
    observations = []
    for i, (q, stamp) in enumerate(zip(qs, stamps)):
        p = x.T @ q.T @ x @ p0
        centers = cv2.projectPoints(obj, h.log(p), np.array([-.14, -.09, 1.]), k, d)[0]
        if i % 2: centers = centers[::-1].copy()
        observations.append(centers)
        (root/f'{i+1:03d}.json').write_text(json.dumps(dict(detection_frame_monotonic_s=1000+stamp)))
    measurement = dict(mode='handeye', cols=9, rows=6, spacing_mm=35., image_size=[1280,720], camera_time_offset_ms=0.)
    (root/'session.json').write_text(json.dumps(dict(measurement=measurement,captured=25)))
    return x, observations, measurement, rows


class HandeyeTests(unittest.TestCase):
    def test_recover_nontrivial_rotation_and_reject_single_axis(self):
        x = h.exp([.4, -.3, 1.2])
        vectors = [[.3, 0, 0], [0, -.4, 0], [0, 0, .5], [.2, .3, 0], [0, -.3, .2], [-.3, 0, .3]]
        pairs = [(h.exp(v), x.T @ h.exp(v) @ x) for v in vectors]
        result, ratios = h.solve_rotation(pairs)
        self.assertLess(h.angle(result @ x.T), 1e-5)
        self.assertGreater(min(ratios), .15)
        with self.assertRaisesRegex(ValueError, '单轴'):
            h.solve_rotation([(h.exp([0,0,v]),x.T @ h.exp([0,0,v]) @ x) for v in np.linspace(.1,.8,8)])

    def test_full_pipeline_with_reversed_circle_order_and_independent_holdout(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            x, observations, measurement, _ = synthetic_session(root)
            report = h.calculate(root, observations, (1280,720), measurement)
            self.assertLess(h.angle(np.array(report['R_imu_from_camera']) @ x.T), .02)
            self.assertLess(max(report['holdout_errors_deg']), .02)
            self.assertEqual(report['warnings'], [])
            self.assertEqual(report['holdout_pair_count'], 7)
            self.assertFalse(report['translation_estimated'])
            fs = cv2.FileStorage(str(root/'camera_imu_rotation.yaml'), cv2.FILE_STORAGE_READ)
            self.assertTrue(fs.getNode('T_robot_from_camera').empty())
            self.assertEqual(fs.getNode('extrinsics_validated').real(), 0)
            fs.release()
            changed = list(observations)
            k, d, _ = calibrate.load_intrinsics(root/'intrinsics.yaml')
            # Perturb a held-out camera pose: fitted rotation must stay unchanged.
            p, _ = h.board_pose(changed[-2], k, d, (9,6), .035)
            changed[-2] = cv2.projectPoints(calibrate.object_grid((9,6),.035), h.log(h.exp([.01,0,0]) @ p), np.array([-.14,-.09,1.]),k,d)[0]
            other = h.calculate(root, changed, (1280,720), measurement)
            np.testing.assert_allclose(other['R_imu_from_camera'],report['R_imu_from_camera'],atol=1e-10)

    def test_full_pipeline_rejects_planar_yaw_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); _, obs, meta, _ = synthetic_session(root, True)
            with self.assertRaisesRegex(ValueError, '单轴'):
                h.calculate(root, obs, (1280,720), meta)
            self.assertFalse((root/'camera_imu_rotation.yaml').exists())

    def test_device_wrap_drops_staleness_and_motion(self):
        rows=[dict(received_us=int((10+i*.01)*1e6),device_time_ms=(2**32-100+i*10)%(2**32),
                   sequence=i+1,measurements_valid=True,gyro_rad_s=[0,0,0],accel_m_s2=[0,0,9.8]) for i in range(700)]
        t,g,a,_=h.prepare_imu(rows)
        np.testing.assert_allclose(h.stationary(t,g,a,16.,5),[0,0,0])
        self.assertAlmostEqual(t[-1]-t[0],6.99)
        with self.assertRaises(ValueError):h.prepare_imu(rows[:20]+rows[21:])
        with self.assertRaises(ValueError):h.stationary(t,g,a,17.5,1.5)
        with self.assertRaises(ValueError):h.stationary(t,g+np.array([0,0,.1]),a,16.,1.5,np.zeros(3))
        with self.assertRaises(ValueError):h.integrate(t,g,8,16,np.zeros(3))
        bad=[dict(r) for r in rows];bad[20]['measurements_valid']=False
        with self.assertRaises(ValueError):h.prepare_imu(bad)

    def test_gyro_integration_order(self):
        t=np.arange(0,4.001,.001);g=np.zeros((len(t),3))
        g[t<2,0]=.2;g[t>=2,1]=-.3
        got=h.integrate(t,g,0,4,np.zeros(3))
        self.assertLess(h.angle(got @ (h.exp([.4,0,0]) @ h.exp([0,-.6,0])).T),.02)

    def test_guided_capture_accepts_return_pose_and_checks_imu(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); _, obs, meta, _ = synthetic_session(root)
            (root/'samples').mkdir();(root/'detected').mkdir()
            args=SimpleNamespace(mode='handeye',intrinsics=root/'intrinsics.yaml',width=1280,height=720,camera_time_offset_ms=0)
            w=h.make_wizard_class()(args);w.session=root;w.measurement=meta;w.size=(1280,720)
            class FakeRecorder:
                def check(self,*args):return np.zeros(3)
            w.recorder=FakeRecorder();w.frame=np.zeros((720,1280,3),np.uint8)
            for i in [0,1,3]:
                w.centers=obs[i];w.frame_time=time.monotonic();w.capture(len(w.observations))
            self.assertEqual(len(w.observations),3)
            w.undo(3);self.assertEqual(len(w.poses),2)
            w.recorder.check=lambda *a: (_ for _ in ()).throw(ValueError('IMU stale'))
            with self.assertRaisesRegex(ValueError,'stale'):w.capture(2)
            self.assertEqual(len(w.observations),2)

    def test_recorder_real_parser_over_pseudo_terminal(self):
        binary=ROOT/'build-release/rescue_imu_record'
        self.assertTrue(binary.is_file(),'Build rescue_imu_record before CTest')
        master,slave=pty.openpty()
        fixture=bytearray((ROOT/'tests/fixtures/hi91-capture.bin').read_bytes())
        stop=threading.Event()
        def feed():
            tick=0
            while not stop.wait(.01):
                tick+=10;frame=bytearray(fixture);struct.pack_into('<I',frame,14,tick)
                crc=0
                for value in frame[:4]+frame[6:]:
                    crc^=value<<8
                    for _ in range(8):crc=((crc<<1)^0x1021)&65535 if crc&0x8000 else (crc<<1)&65535
                struct.pack_into('<H',frame,4,crc);os.write(master,frame)
        process=subprocess.Popen([str(binary),os.ttyname(slave),'115200','1'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        worker=threading.Thread(target=feed);worker.start()
        try:
            out,err=process.communicate(timeout=5)
            self.assertEqual(process.returncode,0,err)
            rows=[json.loads(line) for line in out.splitlines()]
            self.assertGreater(len(rows),50)
            self.assertTrue(all(s['measurements_valid'] for s in rows))
            np.testing.assert_allclose(rows[0]['gyro_rad_s'],np.radians(struct.unpack_from('<fff',fixture,30)),atol=1e-6)
            self.assertTrue(all(b['sequence']==a['sequence']+1 for a,b in zip(rows,rows[1:])))
            ready,_,_=select.select([master],[],[],0)
            self.assertEqual(ready,[],'Recorder must not write any bytes to the serial port')
        finally:
            stop.set();worker.join();os.close(master);os.close(slave)
            if process.poll() is None:process.kill();process.wait()

    def test_cli_help_and_mode_resolution_validation(self):
        script=ROOT/'tools/camera_calibration/guided.py'
        help_result=subprocess.run(['python3',str(script),'--help'],capture_output=True,text=True)
        self.assertEqual(help_result.returncode,0,help_result.stderr)
        self.assertIn('--mode {intrinsics,handeye}',help_result.stdout)
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'intrinsics.yaml'
            calibrate.write_yaml(path,np.eye(3),np.zeros(5),(640,480))
            result=subprocess.run(['python3',str(script),'--mode','handeye','--intrinsics',str(path)],capture_output=True,text=True)
            self.assertNotEqual(result.returncode,0)
            self.assertIn('分辨率',result.stderr)


if __name__=='__main__':unittest.main()
