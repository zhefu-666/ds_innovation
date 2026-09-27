#!/usr/bin/env python3
"""Browser-guided 10 x 7 symmetric-circle camera intrinsics calibration."""
import argparse
import errno
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import subprocess
import threading
import time
import uuid
from urllib.request import ProxyHandler, build_opener

import cv2
import numpy as np
import calibrate

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PATTERN = (10, 7)
STEPS = [
    '中央，正对相机，中等距离',
    '左上区域，完整保留所有圆点', '上方中央，完整保留所有圆点',
    '右上区域，完整保留所有圆点', '右侧中央，完整保留所有圆点',
    '右下区域，完整保留所有圆点', '下方中央，完整保留所有圆点',
    '左下区域，完整保留所有圆点', '左侧中央，完整保留所有圆点',
    '中央靠近，让圆点阵列占画面宽度约 60%～80%',
    '中央稍远，让圆点阵列占画面宽度约 30%～40%',
    '中央，平板左边靠近相机，倾斜约 20°',
    '中央，平板右边靠近相机，倾斜约 20°',
    '中央，平板上边靠近相机，倾斜约 20°',
    '中央，平板下边靠近相机，倾斜约 20°',
    '偏左，平板左边靠近相机，倾斜约 35°',
    '偏右，平板右边靠近相机，倾斜约 35°',
    '偏上，平板上边靠近相机，倾斜约 35°',
    '偏下，平板下边靠近相机，倾斜约 35°',
    '左上区域，同时向左右和上下方向倾斜',
    '右下区域，换一个左右和上下倾斜姿态',
    '中央，平板在自身平面内旋转约 20°，再稍微倾斜',
    '验证照片：偏左、稍近，换一个倾斜姿态',
    '验证照片：偏右、稍远，换一个倾斜姿态',
    '验证照片：中央，同时向两个方向倾斜',
]


def spacing_from_spans(horizontal, vertical):
    """Measured first-to-last CENTER spans: 9 horizontal / 6 vertical intervals."""
    x, y = float(horizontal) / 9, float(vertical) / 6
    if not all(math.isfinite(v) and 0.5 <= v <= 100 for v in (x, y)):
        raise ValueError('尺寸无效：请填写毫米数，横向量第 1 到第 10 个圆心，纵向量第 1 到第 7 个圆心。')
    if abs(x-y) / ((x+y)/2) > .01:
        raise ValueError('横纵圆心间距相差超过 1%，请检查量尺位置或平板是否拉伸了 PDF。')
    return (x+y)/2


def detect_circles(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    params = cv2.SimpleBlobDetector_Params()
    params.filterByColor = True; params.blobColor = 0
    params.filterByArea = True; params.minArea = 12; params.maxArea = gray.size * .015
    params.filterByCircularity = False
    params.filterByConvexity = True; params.minConvexity = .75
    params.filterByInertia = True; params.minInertiaRatio = .15
    detector = cv2.SimpleBlobDetector_create(params)
    # Detect subpixel blob centers at ORIGINAL resolution. Limit graph matching to
    # plausible candidates so a cluttered scene cannot monopolize detection.
    blobs = detector.detect(gray)
    if not 70 <= len(blobs) <= 180:
        return None
    found, centers = cv2.findCirclesGrid(gray, PATTERN,
        flags=cv2.CALIB_CB_SYMMETRIC_GRID, blobDetector=detector)
    return centers if found and len(centers) == 70 else None


def quality(frame, centers):
    if centers is None:
        return '还未识别全部 70 个圆点：请调整距离、反光和倾角。'
    h, w = frame.shape[:2]
    p = centers.reshape(-1, 2)
    if p[:, 0].min() < 12 or p[:, 1].min() < 12 or p[:, 0].max() > w-12 or p[:, 1].max() > h-12:
        return '圆点太靠近画面边缘，请留出白边。'
    if cv2.contourArea(cv2.convexHull(centers))/(w*h) < .025:
        return '圆点阵列太小，请靠近一些。'
    x, y, bw, bh = cv2.boundingRect(centers)
    gray = cv2.cvtColor(frame[y:y+bh, x:x+bw], cv2.COLOR_BGR2GRAY)
    if cv2.Laplacian(gray, cv2.CV_64F).var() < 30:
        return '图像模糊，请保持平板静止，检查焦点。'
    return ''


def duplicate(centers, previous):
    # A symmetric grid may be returned in reverse order. Handle both orientations.
    return any(min(float(np.sqrt(np.mean(np.sum((centers-old)**2, axis=2)))),
                   float(np.sqrt(np.mean(np.sum((centers-old[::-1])**2, axis=2))))) < 7
               for old in previous)


def save_json(path, data):
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False))
    tmp.replace(path)


def calculate(session, observations, size, measurement):
    if len(observations) != len(STEPS):
        raise ValueError('需要按提示完成 25 张照片；原图已保留。')
    spacing = measurement['spacing_mm'] / 1000
    # Last three poses are held out: they are never used to fit intrinsics.
    rms, k, d, errors = calibrate.solve_intrinsics(observations[:22], size, PATTERN, spacing)
    obj = calibrate.object_grid(PATTERN, spacing)
    checks = []
    for centers in observations[22:]:
        ok, r, t = cv2.solvePnP(obj, centers, k, d)
        if not ok:
            raise ValueError('独立照片位姿验证失败，请检查原图。')
        pred, _ = cv2.projectPoints(obj, r, t, k, d)
        checks.append(float(np.sqrt(np.mean(np.sum((pred-centers)**2, axis=2)))))
    coverage = (np.ptp(np.concatenate(observations[:22]).reshape(-1, 2), axis=0) / np.array(size)).tolist()
    if not np.isfinite([rms, *errors, *checks, *coverage]).all():
        raise ValueError('计算结果包含非有限值，请重拍。')
    warnings = []
    if rms > .5: warnings.append('拟合 RMS 超过 0.5 像素。')
    if max(errors) > 1: warnings.append('至少一张拟合照片误差超过 1 像素。')
    if max(checks) > 1: warnings.append('独立验证照片误差超过 1 像素。')
    if min(coverage) < .6: warnings.append('圆点覆盖不足，请增加靠近画面边缘的姿态。')
    if not (0 < k[0, 2] < size[0] and 0 < k[1, 2] < size[1]):
        warnings.append('主点落在图像范围外，请检查拍摄姿态是否足够多样。')
    report = dict(status='needs_recapture' if warnings else 'candidate_requires_validation',
        image_size=list(size), camera_matrix=k.tolist(), dist_coeffs=d.ravel().tolist(),
        measurement=measurement, fit_rms_px=float(rms), fit_per_image_rms_px=errors,
        holdout_rms_px=checks, coverage_span=coverage, warnings=warnings,
        fit_images=list(range(1, 23)), holdout_images=[23, 24, 25],
        session=str(session), intrinsics_file=str(session/'intrinsics.yaml'),
        note='仅相机内参与畸变；未标定相机安装外参和地面映射，未接入主程序。')
    calibrate.write_yaml(session/'intrinsics.yaml', k, d, size)
    save_json(session/'report.json', report)
    (session/'使用说明.txt').write_text(
        '本次标定结果\n' + json.dumps(report, ensure_ascii=False, indent=2) + '\n\n'
        '无需手抄 fx、fy、cx、cy 或畸变系数，intrinsics.yaml 已按 OpenCV 格式保存。\n'
        'tools/camera_calibration/calibrate.py 的 ground/extrinsics 模式：\n'
        f'  --intrinsics {session}/intrinsics.yaml\n'
        '注意：原 extrinsics 模式仅检测方格棋盘；圆点板请用 ground 模式的实测地面点流程。\n'
        '完成并验收地面标定后，才将生成的完整 camera.yaml 放在 config/camera.yaml。\n'
        'config/rescue.yaml 对应字段为 calibration_file: ./config/camera.yaml。\n'
        '当前主程序尚未读取该 YAML，填写路径不会自动启用距离计算。\n'
        '只有相同分辨率、焦点、镜头模式下才可复用本次内参。\n'
        '本次是候选内参；平板反光、屏幕条纹、姿态不充分等会影响实际精度。\n')
    return report


class Wizard:
    def __init__(self, args):
        self.args = args
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.worker = None
        self.processing_workers = []
        self.latest_raw = None
        self.raw_sequence = 0
        self.preview_time = 0.
        self.capture_fps = self.preview_fps = self.detection_fps = 0.
        self.camera_reported_fps = 0.
        self.detection_ms = 0.
        self.detection_jpeg = None
        self.camera_error = ""
        self.cap = None
        self.frame = self.centers = self.jpeg = None
        self.frame_time = 0.
        self.frame_timestamp_ns = 0
        self.frame_sequence = 0
        self.preview_size = (0, 0)
        self.foxglove = None
        self.error = ''
        self.observations = []
        self.session = None
        self.measurement = None
        self.size = None
        self.result = None
        self.busy = False

    def start(self, values):
        with self.lock:
            if self.session:
                raise ValueError('本次会话已开始，不能中途修改尺寸或重复开始。需要重拍时请重新启动程序。')
            spacing = spacing_from_spans(values['horizontal_mm'], values['vertical_mm'])
            if values.get('confirmed') is not True:
                raise ValueError('请确认已锁定屏幕缩放、方向和相机焦点。')
            self.begin_preview()
            session = Path(self.args.output).resolve()/('circles_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
            (session/'samples').mkdir(parents=True, exist_ok=False)
            (session/'detected').mkdir()
            measurement = dict(horizontal_mm=float(values['horizontal_mm']), vertical_mm=float(values['vertical_mm']),
                spacing_mm=spacing, cols=10, rows=7, image_size=list(self.size), camera=self.args.camera)
            save_json(session/'session.json', dict(measurement=measurement, steps=STEPS, captured=0))
            self.session, self.measurement = session, measurement

    def begin_preview(self):
        with self.lock:
            if self.cap is not None:
                if self.stop_event.is_set() or not self.worker.is_alive():
                    raise ValueError('相机已停止，请重新启动程序。')
                return
            if self.args.stop_preview:
                subprocess.run(['python3', str(ROOT/'tools/remote_camera_telemetry/manage_project.py'), 'stop'],
                               check=True, timeout=15, cwd=ROOT)
            source = int(self.args.camera) if self.args.camera.isdigit() else self.args.camera
            cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.args.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.args.height)
            cap.set(cv2.CAP_PROP_FPS, getattr(self.args, "camera_fps", 60))
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not cap.isOpened():
                cap.release()
                raise ValueError('无法打开相机，请检查设备号或其他占用相机的程序。')
            ok, frame = cap.read()
            actual = (frame.shape[1], frame.shape[0]) if ok else None
            if actual != (self.args.width, self.args.height):
                cap.release()
                raise ValueError(f'相机实际尺寸 {actual} 与要求 {self.args.width}×{self.args.height} 不符，未开始标定。')
            self.cap, self.size = cap, actual
            self.camera_reported_fps = float(cap.get(cv2.CAP_PROP_FPS))
            self.worker = threading.Thread(target=self.acquire, daemon=True)
            self.worker.start()
            for target in (self.make_preview, self.detect_latest):
                worker = threading.Thread(target=target, daemon=True)
                self.processing_workers.append(worker)
                worker.start()

    def acquire(self):
        start, count = time.monotonic(), 0
        try:
            while not self.stop_event.is_set():
                ok, frame = self.cap.read()
                timestamp, timestamp_ns = time.monotonic(), time.time_ns()
                if not ok:
                    raise ValueError('相机断开或读取失败，请重启程序；已拍原图仍保留。')
                if (frame.shape[1], frame.shape[0]) != self.size:
                    raise ValueError('采集中图像尺寸发生变化，已停止。')
                count += 1
                with self.lock:
                    self.raw_sequence += 1
                    # Immutable references; consumers skip older frames instead of building a queue.
                    self.latest_raw = (frame, timestamp, timestamp_ns, self.raw_sequence)
                    if timestamp-start >= 1:
                        self.capture_fps = count/(timestamp-start)
                        start, count = timestamp, 0
        except Exception as exc:
            with self.lock:
                self.camera_error = str(exc)
                self.latest_raw = None
                self.preview_time = self.frame_time = 0
                self.stop_event.set()
        finally:
            self.cap.release()

    def make_preview(self):
        previous, count, start = 0, 0, time.monotonic()
        period = 1/getattr(self.args, 'preview_fps', 30)
        try:
            while not self.stop_event.is_set():
                begin = time.monotonic()
                with self.lock: raw = self.latest_raw
                if raw is None or raw[3] == previous:
                    self.stop_event.wait(.003)
                    continue
                frame, timestamp, stamp, sequence = raw
                previous = sequence
                scale = min(1., 960/frame.shape[1])
                preview = cv2.resize(frame, None, fx=scale, fy=scale)
                ok, jpeg = cv2.imencode('.jpg', preview, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if not ok: raise ValueError('预览图像编码失败')
                with self.lock:
                    self.jpeg = jpeg.tobytes()
                    self.frame_timestamp_ns, self.preview_time = stamp, timestamp
                    self.frame_sequence += 1
                    self.preview_size = (preview.shape[1], preview.shape[0])
                    count += 1
                    now = time.monotonic()
                    if now-start >= 1:
                        self.preview_fps = count/(now-start)
                        start, count = now, 0
                self.stop_event.wait(max(0, period-(time.monotonic()-begin)))
        except Exception as exc:
            with self.lock:
                self.camera_error = str(exc)
                self.preview_time = 0
                self.stop_event.set()

    def detect_latest(self):
        previous, count, start = 0, 0, time.monotonic()
        period = 1/getattr(self.args, 'detect_fps', 10)
        try:
            while not self.stop_event.is_set():
                begin = time.monotonic()
                with self.lock: raw = self.latest_raw
                if raw is None or raw[3] == previous:
                    self.stop_event.wait(.005)
                    continue
                frame, timestamp, stamp, sequence = raw
                previous = sequence
                centers = detect_circles(frame)
                reason = quality(frame, centers)
                annotated = frame.copy()
                if centers is not None:
                    cv2.drawChessboardCorners(annotated, PATTERN, centers, True)
                scale = min(1., 640/frame.shape[1])
                annotated = cv2.resize(annotated, None, fx=scale, fy=scale)
                ok, jpeg = cv2.imencode('.jpg', annotated)
                with self.lock:
                    # Store frame AND centers together. Never save latest_raw with older centers.
                    self.frame, self.centers, self.frame_time = frame, centers, timestamp
                    self.detection_jpeg = jpeg.tobytes() if ok else None
                    self.detection_ms = (time.monotonic()-begin)*1000
                    self.error = reason
                    count += 1
                    now = time.monotonic()
                    if now-start >= 1:
                        self.detection_fps = count/(now-start)
                        start, count = now, 0
                self.stop_event.wait(max(0, period-(time.monotonic()-begin)))
        except Exception as exc:
            with self.lock:
                self.error = '圆点检测失败：' + str(exc)
                self.frame_time = 0

    def capture(self, expected_count):
        with self.lock:
            if self.session is None:
                raise ValueError('当前仅预览，请先填写实测尺寸并开始标定。')
            if self.busy or self.result or len(self.observations) >= len(STEPS):
                raise ValueError('本次拍摄已完成或正在计算。')
            if expected_count != len(self.observations):
                raise ValueError('页面进度已变化，请稍后再拍，避免重复请求。')
            if self.frame is None or time.monotonic()-self.frame_time > 1 or self.stop_event.is_set():
                raise ValueError('没有新鲜相机画面，请等待或检查相机。')
            if self.error: raise ValueError(self.error)
            if self.centers is None: raise ValueError('尚未识别完整圆点阵列。')
            if duplicate(self.centers, self.observations):
                raise ValueError('与已保存的姿态太相近，请按当前提示移动或倾斜平板。')
            index = len(self.observations)+1
            path = self.session/'samples'/f'{index:03d}.png'
            if not cv2.imwrite(str(path), self.frame):
                raise ValueError('原图保存失败，请检查磁盘空间。')
            annotated = self.frame.copy()
            cv2.drawChessboardCorners(annotated, PATTERN, self.centers, True)
            if not cv2.imwrite(str(self.session/'detected'/path.name), annotated):
                raise ValueError('检测图保存失败，请检查磁盘空间。')
            save_json(self.session/f'{index:03d}.json', dict(step=STEPS[index-1],
                centers=self.centers.reshape(-1, 2).tolist(), image_size=list(self.size),
                detection_frame_monotonic_s=self.frame_time))
            self.observations.append(self.centers.copy())
            save_json(self.session/'session.json', dict(measurement=self.measurement, steps=STEPS,
                captured=len(self.observations)))
            if len(self.observations) == len(STEPS):
                self.busy = True
                self.stop_event.set()
                threading.Thread(target=self.solve, daemon=True).start()

    def solve(self):
        try:
            result = calculate(self.session, self.observations, self.size, self.measurement)
            with self.lock: self.result = result; self.error = ''
        except Exception as exc:
            with self.lock: self.error = '计算失败，原图已保留：' + str(exc)
        finally:
            with self.lock: self.busy = False

    def undo(self, expected_count):
        with self.lock:
            if self.busy or self.result or self.stop_event.is_set():
                raise ValueError('计算开始后不能撤销，请新建一轮拍摄。')
            if not self.observations or expected_count != len(self.observations):
                raise ValueError('没有可撤销照片或页面进度已变化。')
            index = len(self.observations)
            # Preserve mistaken images for traceability instead of deleting them.
            archive = self.session/'retakes'/str(time.time_ns()); archive.mkdir(parents=True)
            for folder in ('samples', 'detected'):
                (self.session/folder/f'{index:03d}.png').rename(archive/f'{folder}.png')
            (self.session/f'{index:03d}.json').rename(archive/'capture.json')
            self.observations.pop()
            save_json(self.session/'session.json', dict(measurement=self.measurement, steps=STEPS,
                captured=len(self.observations)))

    def state(self):
        with self.lock:
            n = len(self.observations)
            fresh = bool(self.frame_time) and time.monotonic()-self.frame_time <= 1 and not self.stop_event.is_set()
            camera_fresh = bool(self.preview_time) and time.monotonic()-self.preview_time <= 2 and not self.stop_event.is_set()
            return dict(started=self.session is not None, count=n, total=len(STEPS),
                step=STEPS[n] if n < len(STEPS) else '照片已齐，正在计算参数',
                ready=self.session is not None and fresh and not self.error and not self.busy and not self.result and n < len(STEPS),
                detected=self.centers is not None and fresh, error=self.camera_error or self.error,
                busy=self.busy, result=self.result, image_size=self.size,
                frame_sequence=self.frame_sequence, camera_fresh=camera_fresh,
                previewing=self.cap is not None and not self.stop_event.is_set(),
                frame_age_ms=round((time.monotonic()-self.preview_time)*1000, 1) if self.preview_time else None,
                detection_age_ms=round((time.monotonic()-self.frame_time)*1000, 1) if self.frame_time else None,
                capture_fps=round(self.capture_fps, 1), preview_fps=round(self.preview_fps, 1),
                detection_fps=round(self.detection_fps, 1), detection_ms=round(self.detection_ms, 1),
                camera_reported_fps=self.camera_reported_fps,
                preview_size=self.preview_size,
                foxglove_port=self.foxglove.port if self.foxglove else None,
                foxglove_clients=len(self.foxglove.clients) if self.foxglove else 0,
                requested_size=[self.args.width, self.args.height], session=str(self.session) if self.session else None)

    def foxglove_snapshot(self):
        with self.lock:
            return self.state(), self.jpeg, self.frame_timestamp_ns

    def close(self):
        self.stop_event.set()
        if self.worker: self.worker.join(timeout=3)
        for worker in self.processing_workers: worker.join(timeout=1)


def handler_for(wizard):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_): pass

        def send(self, body, content_type, status=200):
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split('?')[0]
            if path == '/': self.send((HERE/'guided.html').read_bytes(), 'text/html; charset=utf-8')
            elif path == '/board.pdf': self.send((HERE/'assets/circles_10x7.pdf').read_bytes(), 'application/pdf')
            elif path == '/api/state': self.send(json.dumps(wizard.state(), ensure_ascii=False).encode(), 'application/json')
            elif path == '/stream.mjpg':
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                previous = 0
                try:
                    while not wizard.stop_event.is_set():
                        with wizard.lock:
                            jpeg, seq, stamp = wizard.jpeg, wizard.frame_sequence, wizard.preview_time
                        if jpeg and seq != previous and time.monotonic()-stamp < 2:
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
                            self.wfile.flush()
                            previous = seq
                        wizard.stop_event.wait(.01)
                except (BrokenPipeError, ConnectionResetError, OSError): pass
            elif path == '/detection.jpg':
                with wizard.lock: jpeg = wizard.detection_jpeg
                self.send(jpeg or b'', 'image/jpeg', 200 if jpeg else 204)
            elif path == '/preview.jpg':
                with wizard.lock: jpeg = wizard.jpeg
                self.send(jpeg or b'', 'image/jpeg', 200 if jpeg else 204)
            elif path in ('/intrinsics.yaml', '/report.json', '/instructions.txt'):
                with wizard.lock:
                    ready = wizard.result is not None
                    session = wizard.session
                if not ready: self.send(b'Not ready', 'text/plain', 404); return
                name = {'/instructions.txt': '使用说明.txt'}.get(path, path[1:])
                self.send((session/name).read_bytes(), 'text/plain; charset=utf-8')
            else: self.send(b'Not found', 'text/plain', 404)

        def do_POST(self):
            # No CORS; require JSON and same-host Origin for browser mutations.
            origin = self.headers.get('Origin')
            if origin and origin != 'http://' + self.headers.get('Host', ''):
                self.send(b'Forbidden', 'text/plain', 403); return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 4096 or self.headers.get('Content-Type') != 'application/json':
                    raise ValueError('请求格式无效。')
                values = json.loads(self.rfile.read(length))
                if self.path == '/api/start': wizard.start(values)
                elif self.path == '/api/preview': wizard.begin_preview()
                elif self.path == '/api/capture': wizard.capture(values['expected_count'])
                elif self.path == '/api/undo': wizard.undo(values['expected_count'])
                else: raise ValueError('未知操作。')
                self.send(json.dumps(wizard.state(), ensure_ascii=False).encode(), 'application/json')
            except (ValueError, KeyError, TypeError, OSError, subprocess.SubprocessError, cv2.error) as exc:
                self.send(json.dumps(dict(error=str(exc)), ensure_ascii=False).encode(), 'application/json', 400)
    return Handler


def describe_existing_service(args):
    """Report a running calibration without interrupting it or taking its camera."""
    host = '127.0.0.1' if args.host in ('0.0.0.0', '') else args.host
    if host == '::': host = '::1'
    if ':' in host: host = f'[{host}]'
    try:
        opener = build_opener(ProxyHandler({}))
        with opener.open(f'http://{host}:{args.port}/api/state', timeout=2) as response:
            state = json.loads(response.read(65536))
        if not isinstance(state, dict) or not all(key in state for key in
                ('started', 'count', 'total', 'step', 'requested_size', 'session')):
            return False
    except (OSError, ValueError):
        return False
    print(f'标定程序已经在运行，请直接打开 http://192.168.1.123:{args.port}/')
    print(f'当前已保存 {state["count"]}/{state["total"]} 张照片；现有会话和相机继续运行。')
    if state.get('foxglove_port'):
        print(f'Foxglove：ws://192.168.1.123:{state["foxglove_port"]}')
    print('本次未启动第二份程序，新的启动参数未应用。需要重启时请先结束原进程。')
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', default='/dev/video0')
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--camera-fps', type=int, default=60)
    parser.add_argument('--preview-fps', type=int, default=30)
    parser.add_argument('--detect-fps', type=int, default=10)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8081)
    parser.add_argument('--foxglove-port', type=int, default=8766, help='Read-only Foxglove WebSocket port')
    parser.add_argument('--output', default=str(ROOT/'calibration_runs'))
    parser.add_argument('--stop-preview', action='store_true', help='Stop managed preview when user starts camera')
    parser.add_argument('--recalculate', type=Path, help='Recalculate an existing complete session from its original PNGs')
    args = parser.parse_args()
    if not (1 <= args.camera_fps <= 120 and 1 <= args.preview_fps <= 60 and 1 <= args.detect_fps <= 30):
        parser.error('Invalid frame rate')
    cv2.setNumThreads(2)
    if args.width <= 0 or args.height <= 0: parser.error('Invalid image size')
    if args.recalculate:
        session = args.recalculate.resolve()
        meta = json.loads((session/'session.json').read_text())
        measurement = meta['measurement']; size = tuple(measurement['image_size'])
        if meta['captured'] != 25: parser.error('Session must contain all 25 captures')
        observations = []
        for i in range(1, 26):
            frame = cv2.imread(str(session/'samples'/f'{i:03d}.png'))
            if frame is None or (frame.shape[1], frame.shape[0]) != size: parser.error(f'Invalid image {i}')
            centers = detect_circles(frame)
            if centers is None: parser.error(f'Circle detection failed on image {i}')
            observations.append(centers)
        print(json.dumps(calculate(session, observations, size, measurement), ensure_ascii=False, indent=2))
        return
    wizard = Wizard(args)
    try:
        server = ThreadingHTTPServer((args.host, args.port), handler_for(wizard))
    except OSError as exc:
        if exc.errno == errno.EADDRINUSE:
            if describe_existing_service(args): return
            parser.exit(1, f'端口 {args.port} 已被其他服务占用，或标定服务暂时无响应。请检查占用进程；未停止任何现有程序。\n')
        parser.exit(1, f'无法启动标定网页：{exc}\n')
    from foxglove_bridge import FoxgloveBridge
    wizard.foxglove = FoxgloveBridge(wizard, args.host, args.foxglove_port, fps=args.preview_fps)
    try: wizard.foxglove.start()
    except RuntimeError as exc:
        server.server_close()
        wizard.foxglove.close()
        parser.exit(1, f'{exc}。请检查 {args.foxglove_port} 端口，或使用 --foxglove-port 指定空闲端口。\n')
    print(f'圆点标定已启动。电脑打开 http://192.168.1.123:{args.port}/', flush=True)
    print('点击「仅预览」或填写尺寸后点击「开始」才打开相机。Ctrl+C 退出；原图与结果会保留。', flush=True)
    print(f'Foxglove 连接 ws://192.168.1.123:{wizard.foxglove.port}，图像话题 /calibration/image', flush=True)
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally: server.server_close(); wizard.close(); wizard.foxglove.close()


if __name__ == '__main__': main()
