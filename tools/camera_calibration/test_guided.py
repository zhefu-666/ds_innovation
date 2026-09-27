"""Circle detection, holdout calibration, and guided capture regression tests."""
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from urllib.request import Request, urlopen
from urllib.error import HTTPError
from http.server import ThreadingHTTPServer
from unittest.mock import patch

import cv2
import numpy as np
import guided as g


def board():
    image = np.full((600, 800, 3), 255, np.uint8)
    for y in range(7):
        for x in range(10): cv2.circle(image, (100+x*60, 120+y*60), 12, (0, 0, 0), -1)
    return image


class GuidedTests(unittest.TestCase):
    def test_slow_detection_does_not_block_preview_or_mismatch_saved_image(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'samples').mkdir();(root/'detected').mkdir()
            wizard=g.Wizard(SimpleNamespace(width=800,height=600,preview_fps=30,detect_fps=10))
            wizard.session=root;wizard.measurement={'spacing_mm':12};wizard.size=(800,600)
            first=board();centers=g.detect_circles(first)
            wizard.latest_raw=(first,time.monotonic(),time.time_ns(),1)
            entered=threading.Event();release=threading.Event()
            def slow(frame):
                entered.set();release.wait(2)
                return centers if np.array_equal(frame,first) else None
            preview=threading.Thread(target=wizard.make_preview)
            detection=threading.Thread(target=wizard.detect_latest)
            with patch.object(g,'detect_circles',side_effect=slow):
                try:
                    preview.start();detection.start();self.assertTrue(entered.wait(2))
                    initial=wizard.frame_sequence
                    for seq in range(2,8):
                        with wizard.lock:
                            wizard.latest_raw=(np.full_like(first,seq),time.monotonic(),time.time_ns(),seq)
                        time.sleep(.04)
                    self.assertGreater(wizard.frame_sequence,initial+2)
                    self.assertFalse(wizard.state()['ready'])
                    release.set()
                    deadline=time.monotonic()+2
                    while wizard.frame_time==0 and time.monotonic()<deadline:time.sleep(.005)
                    # Explicitly pin a valid result from the first image while raw frames differ.
                    with wizard.lock:
                        wizard.frame, wizard.centers=first,centers
                        wizard.frame_time=time.monotonic();wizard.error=''
                        wizard.capture(0)
                    np.testing.assert_array_equal(cv2.imread(str(root/'samples/001.png')),first)
                    wizard.frame_time=time.monotonic()-1.1
                    self.assertFalse(wizard.state()['ready'])
                    with self.assertRaises(ValueError):wizard.capture(1)
                finally:
                    release.set();wizard.stop_event.set();preview.join(3);detection.join(3)
            self.assertFalse(preview.is_alive());self.assertFalse(detection.is_alive())

    def test_preview_reuses_camera_without_creating_calibration(self):
        with tempfile.TemporaryDirectory() as tmp:
            wizard=g.Wizard(SimpleNamespace(width=800,height=600,stop_preview=False,camera='0',output=tmp))
            with patch.object(cv2,'VideoCapture') as factory, patch.object(g.threading,'Thread') as thread:
                cap=factory.return_value;cap.isOpened.return_value=True;cap.read.return_value=(True,board());cap.get.return_value=60
                thread.return_value.is_alive.return_value=True
                wizard.begin_preview();wizard.begin_preview()
                factory.assert_called_once()
                self.assertIsNone(wizard.session)
                self.assertFalse(wizard.state()['ready'])
                with self.assertRaises(ValueError):wizard.capture(0)
                wizard.start(dict(horizontal_mm=108,vertical_mm=72,confirmed=True))
                factory.assert_called_once()
                self.assertTrue(wizard.session.is_dir())
                self.assertEqual(wizard.measurement['spacing_mm'],12)

    def test_measurement_and_scale_validation(self):
        self.assertEqual(g.spacing_from_spans(108, 72), 12)
        self.assertEqual(g.spacing_from_spans(162, 108), 18)
        for x, y in [(120, 72), (0, 0), (float('nan'), 72), (108, float('inf'))]:
            with self.assertRaises(ValueError): g.spacing_from_spans(x, y)

    def test_real_circle_detection_and_blur(self):
        frame = board(); centers = g.detect_circles(frame)
        self.assertIsNotNone(centers)
        self.assertEqual(len(centers), 70)
        self.assertEqual(g.quality(frame, centers), '')
        self.assertTrue(g.duplicate(centers[::-1], [centers]))
        self.assertFalse(g.duplicate(centers+20, [centers]))
        self.assertIsNone(g.detect_circles(np.full_like(frame, 255)))
        self.assertTrue(g.quality(cv2.GaussianBlur(frame, (51, 51), 12), centers))
        h = cv2.getPerspectiveTransform(np.float32([[0,0],[799,0],[799,599],[0,599]]),
                                       np.float32([[70,50],[760,0],[700,580],[10,540]]))
        self.assertIsNotNone(g.detect_circles(cv2.warpPerspective(frame,h,(800,600),borderValue=(255,255,255))))

    def test_intrinsics_holdout_and_files(self):
        k = np.array([[900., 0, 640], [0, 910., 360], [0, 0, 1]])
        obj = g.calibrate.object_grid(g.PATTERN, .012)
        rng = np.random.default_rng(13)
        observations = []
        for _ in range(25):
            pixels, _ = cv2.projectPoints(obj, rng.uniform(-.6,.6,3),
                np.array([rng.uniform(-.14,.07),rng.uniform(-.10,.04),rng.uniform(.25,.4)]), k, np.zeros(5))
            observations.append(pixels)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            report = g.calculate(root, observations, (1280,720), {'spacing_mm':12})
            np.testing.assert_allclose(report['camera_matrix'], k, atol=.1)
            self.assertLess(max(report['holdout_rms_px']), .001)
            self.assertTrue((root/'intrinsics.yaml').exists())
            self.assertFalse((root/'camera.yaml').exists())
            observations[-1] = observations[-1] + rng.normal(0, 6, observations[-1].shape).astype(np.float32)
            bad = g.calculate(root, observations, (1280,720), {'spacing_mm':12})
            self.assertEqual(bad['status'], 'needs_recapture')
            np.testing.assert_allclose(bad['camera_matrix'], report['camera_matrix'])

    def test_capture_freshness_original_undo_and_duplicate(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp);(root/'samples').mkdir();(root/'detected').mkdir()
            wizard = g.Wizard(SimpleNamespace(width=800,height=600))
            wizard.session = root;wizard.measurement = {'spacing_mm':12};wizard.size=(800,600)
            wizard.frame = board();wizard.centers = g.detect_circles(wizard.frame)
            wizard.frame_time = time.monotonic()-3
            with self.assertRaises(ValueError): wizard.capture(0)
            wizard.frame_time = time.monotonic();wizard.capture(0)
            np.testing.assert_array_equal(cv2.imread(str(root/'samples/001.png')), wizard.frame)
            self.assertFalse(np.array_equal(cv2.imread(str(root/'detected/001.png')), wizard.frame))
            with self.assertRaises(ValueError): wizard.capture(0)
            with self.assertRaises(ValueError): wizard.capture(1)
            wizard.undo(1)
            self.assertFalse((root/'samples/001.png').exists())
            self.assertEqual(len(list((root/'retakes').glob('*/samples.png'))), 1)
            wizard.capture(0)
            self.assertEqual(json.loads((root/'session.json').read_text())['captured'], 1)

    def test_http_start_is_lazy_and_origin_protected(self):
        wizard = g.Wizard(SimpleNamespace(width=1280,height=720))
        server = ThreadingHTTPServer(('127.0.0.1',0),g.handler_for(wizard))
        thread = threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        base = f'http://127.0.0.1:{server.server_port}'
        try:
            with patch.object(cv2, 'VideoCapture') as camera:
                with urlopen(base+'/') as r: self.assertIn('25'.encode(),r.read())
                with urlopen(base+'/api/state') as r: self.assertFalse(json.load(r)['started'])
                with urlopen(base+'/board.pdf') as r: self.assertTrue(r.read().startswith(b'%PDF'))
                camera.assert_not_called()
                request = Request(base+'/api/start',data=b'{}',headers={'Content-Type':'application/json','Origin':'http://other-host'})
                with self.assertRaises(HTTPError) as error: urlopen(request)
                self.assertEqual(error.exception.code,403)
                with self.assertRaises(HTTPError): urlopen(base+'/intrinsics.yaml')
        finally:
            server.shutdown();server.server_close();thread.join()

    def test_start_rejects_wrong_camera_size_and_final_capture_solves(self):
        with tempfile.TemporaryDirectory() as tmp:
            args = SimpleNamespace(width=1280,height=720,stop_preview=False,camera='0',output=tmp)
            wizard = g.Wizard(args)
            with patch.object(cv2,'VideoCapture') as factory:
                cap = factory.return_value;cap.isOpened.return_value=True
                cap.read.return_value=(True,board());cap.get.return_value=60
                with self.assertRaises(ValueError):
                    wizard.start(dict(horizontal_mm=108,vertical_mm=72,confirmed=True))
                cap.release.assert_called_once()
                self.assertIsNone(wizard.session)
            root=Path(tmp);(root/'samples').mkdir();(root/'detected').mkdir()
            wizard.session=root;wizard.measurement={'spacing_mm':12};wizard.size=(800,600)
            wizard.frame=board();wizard.centers=g.detect_circles(wizard.frame)
            wizard.observations=[wizard.centers+20+i*10 for i in range(24)]
            wizard.frame_time=time.monotonic()
            done=threading.Event()
            def calculated(*args):
                done.set()
                return {'status':'candidate_requires_validation'}
            with patch.object(g,'calculate',side_effect=calculated):
                wizard.capture(24)
                self.assertTrue(done.wait(5))
                deadline=time.monotonic()+5
                while wizard.busy and time.monotonic()<deadline: time.sleep(.01)
                self.assertEqual(wizard.result['status'],'candidate_requires_validation')
                self.assertTrue(wizard.stop_event.is_set())
                self.assertEqual(json.loads((root/'session.json').read_text())['captured'],25)
                with self.assertRaises(ValueError):wizard.capture(25)


if __name__ == '__main__': unittest.main()
