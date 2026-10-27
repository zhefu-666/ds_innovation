#!/usr/bin/env python3
"""Rotation-only AX=XB: raw HI91 gyro integration + fixed circle-board PnP.

R_AB maps B vectors to A. P_k=R_CkT; B_ij=P_i P_j^T;
A_ij=R_IiIj from body-rate integration; X=R_IC. Translation is not estimated.
"""
import json
import math
from pathlib import Path
import subprocess
import threading
import time

import cv2
import numpy as np
import calibrate

STEPS = [
    '保持整体静止至少 5 秒，记录初始姿态和陀螺仪零偏',
    '整体左转约 15°，停稳 2 秒', '整体右转约 30°，停稳 2 秒',
    '整体回到中央，停稳 2 秒',
    '整体抬头约 15°，停稳 2 秒', '整体低头约 30°，停稳 2 秒',
    '整体回到中央，停稳 2 秒',
    '整体左倾约 15°，停稳 2 秒', '整体右倾约 30°，停稳 2 秒',
    '整体回到中央，停稳 2 秒',
    '整体左转并稍抬头，停稳 2 秒', '整体右转并稍低头，停稳 2 秒',
    '整体回到中央，停稳 2 秒',
    '整体稍左倾并抬头，停稳 2 秒', '整体稍右倾并低头，停稳 2 秒',
    '整体回到中央，停稳 2 秒', '整体左转约 20°，停稳 2 秒',
    '独立验证起点：整体回到中央，停稳 2 秒',
    '独立验证：整体右转约 20°，停稳 2 秒',
    '独立验证：整体回到中央，停稳 2 秒',
    '独立验证：整体抬头约 20°，停稳 2 秒',
    '独立验证：整体低头约 20°，停稳 2 秒',
    '独立验证：整体左倾并稍左转，停稳 2 秒',
    '独立验证：整体右倾并稍右转，停稳 2 秒',
    '独立验证：整体回到中央，停稳 2 秒',
]
FIT_POSES = 17
MAX_ANGLE = math.radians(80)
MAX_GAP = .1


def exp(v):
    return cv2.Rodrigues(np.asarray(v, dtype=float).reshape(3))[0]


def log(r):
    return cv2.Rodrigues(np.asarray(r, dtype=float))[0].reshape(3)


def angle(r):
    return float(np.linalg.norm(log(r)) * 180 / math.pi)


def rotation(r):
    r = np.asarray(r, dtype=float)
    if r.shape != (3, 3) or not np.isfinite(r).all() or not np.allclose(r.T @ r, np.eye(3), atol=1e-5) or abs(np.linalg.det(r)-1) > 1e-5:
        raise ValueError('Invalid rotation matrix')
    return r


def prepare_imu(rows):
    if len(rows) < 20:
        raise ValueError('IMU 样本不足')
    host = np.array([s['received_us']/1e6 for s in rows])
    seq = np.array([s['sequence'] for s in rows], dtype=np.int64)
    ticks = np.array([s['device_time_ms'] for s in rows], dtype=np.int64)
    delta = np.diff(ticks) % (2**32)
    gyro = np.array([s['gyro_rad_s'] for s in rows], dtype=float)
    accel = np.array([s['accel_m_s2'] for s in rows], dtype=float)
    if (not all(s['measurements_valid'] for s in rows) or
            not np.isfinite(host).all() or not np.isfinite(gyro).all() or
            not np.isfinite(accel).all() or gyro.shape != (len(rows), 3) or
            accel.shape != gyro.shape):
        raise ValueError('IMU 包含无效数值或测量')
    if np.any(np.diff(host) < 0) or np.any(np.diff(seq) != 1):
        raise ValueError('IMU 序号不连续或主机时间倒退，请重新采集')
    if np.any(delta == 0) or np.any(delta >= 2**31) or np.any(delta/1000 > MAX_GAP) or np.any(np.diff(host) > MAX_GAP):
        raise ValueError('IMU 时间重复、倒退或存在超过 100 ms 的断流')
    device = np.r_[0., np.cumsum(delta)/1000.]
    # Integrate device-clock intervals; map once to CLOCK_MONOTONIC for endpoints.
    # Host timestamps are receive timestamps, not hardware exposure synchronization.
    offset = float(np.median(host-device))
    return device+offset, gyro, accel, float(np.ptp(host-device)*1000)


def stationary(t, gyro, accel, stamp, seconds, bias=None):
    if stamp-seconds < t[0] or stamp > t[-1]:
        raise ValueError(f'请保持整体静止至少 {seconds:g} 秒后再拍摄')
    mask = (t >= stamp-seconds) & (t <= stamp)
    g, a = gyro[mask], accel[mask]
    if len(g) < 15 or np.max(np.std(g, axis=0)) > math.radians(.15) or np.max(np.std(a, axis=0)) > .2:
        raise ValueError('静止窗口有明显运动，请停稳后重拍')
    average = np.mean(g, axis=0)
    residual = g-(average if bias is None else bias)
    if np.max(np.linalg.norm(residual, axis=1)) > math.radians(.8):
        raise ValueError('拍摄时仍在转动，请停稳后重拍')
    if bias is None and np.linalg.norm(average) > math.radians(2):
        raise ValueError('初始角速度过大，不能作为静止零偏')
    return average


def integrate(t, gyro, start, end, bias):
    if not t[0] <= start < end <= t[-1]:
        raise ValueError('IMU 数据未覆盖两张图像的完整时间区间')
    if end-start > 45:
        raise ValueError('两次拍摄超过 45 秒，请连续采集，减少积分漂移')
    times = np.r_[start, t[(t > start) & (t < end)], end]
    rates = np.column_stack([np.interp(times, t, gyro[:, axis]) for axis in range(3)])-bias
    r = np.eye(3)
    for dt, w in zip(np.diff(times), (rates[:-1]+rates[1:])/2):
        r = r @ exp(w*dt)
    return r


def board_pose(centers, k, d, pattern, spacing, previous=None):
    obj = calibrate.object_grid(pattern, spacing)
    centers = np.asarray(centers, dtype=np.float32).reshape(-1, 1, 2)
    candidates = []
    # Symmetric circle grids may reverse their point order between frames.
    for points in (centers, centers[::-1].copy()):
        ok, rv, tv = cv2.solvePnP(obj, points, k, d, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            continue
        r = exp(rv)
        if np.min((obj @ r.T + tv.reshape(3))[:, 2]) <= 0:
            continue
        pred = cv2.projectPoints(obj, rv, tv, k, d)[0]
        rms = float(np.sqrt(np.mean(np.sum((pred-points)**2, axis=2))))
        if not np.isfinite(rms) or rms > 1.0:
            continue
        score = angle(previous @ r.T) if previous is not None else rms
        candidates.append((score, r, rms))
    if not candidates:
        raise ValueError('标定板 PnP 失败或重投影误差超过 1 px')
    _, r, rms = min(candidates, key=lambda c: c[0])
    if previous is not None and angle(previous @ r.T) > math.degrees(MAX_ANGLE):
        raise ValueError('相邻姿态超过 80°，圆点顺序无法可靠关联，请减小转角')
    return r, rms


def solve_rotation(pairs):
    if len(pairs) < 6:
        raise ValueError('有效多轴转动不足，需要至少 6 段 5°～80° 的转动')
    av = np.array([log(a) for a, b in pairs])
    bv = np.array([log(b) for a, b in pairs])
    sa = np.linalg.svd(av, compute_uv=False)
    sb = np.linalg.svd(bv, compute_uv=False)
    ratios = [float(sa[1]/sa[0]), float(sb[1]/sb[0])]
    if min(ratios) < .15:
        raise ValueError('单轴或近单轴运动退化：必须整体增加俯仰/横滚，不能只原地偏航')
    u, _, vt = np.linalg.svd(av.T @ bv)
    x = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    # Local least squares on SO(3); NumPy/OpenCV only, no extra dependency.
    def residual(r):
        return np.concatenate([log(a @ r @ b.T @ r.T) for a, b in pairs])
    for _ in range(15):
        e = residual(x)
        eps = 1e-5
        jac = np.column_stack([(residual(exp(np.eye(3)[i]*eps) @ x)-e)/eps for i in range(3)])
        step = np.linalg.lstsq(jac, -e, rcond=None)[0]
        if np.linalg.norm(step) < 1e-9:
            break
        trial = exp(step) @ x
        if np.linalg.norm(residual(trial)) >= np.linalg.norm(e):
            break
        x = trial
    return rotation(x), ratios


def residuals(pairs, x):
    return [angle(a @ x @ b.T @ x.T) for a, b in pairs]


def calculate(session, observations, size, measurement):
    from guided import save_json
    session = Path(session)
    if len(observations) != len(STEPS):
        raise ValueError('需要完成 25 张停稳照片，最后 8 张为独立验证序列')
    k, d, expected = calibrate.load_intrinsics(session/'intrinsics.yaml')
    if tuple(size) != expected:
        raise ValueError('图像与内参分辨率不一致')
    rows = [json.loads(line) for line in (session/'imu.jsonl').read_text().splitlines() if line.strip()]
    t, gyro, accel, jitter = prepare_imu(rows)
    stamps = [json.loads((session/f'{i:03d}.json').read_text())['detection_frame_monotonic_s'] for i in range(1, 26)]
    stamps = np.array(stamps)+measurement.get('camera_time_offset_ms', 0)/1000
    if np.any(np.diff(stamps) <= 0):
        raise ValueError('图像时间戳必须严格递增')
    bias = stationary(t, gyro, accel, stamps[0], 5)
    poses, rms = [], []
    for centers, stamp in zip(observations, stamps):
        stationary(t, gyro, accel, stamp, 1.5, bias)
        p, error = board_pose(centers, k, d, (measurement['cols'], measurement['rows']),
                              measurement['spacing_mm']/1000, poses[-1] if poses else None)
        poses.append(p); rms.append(error)
    fit, holdout, rejected, accepted = [], [], [], []
    for j in range(1, len(poses)):
        # No training/holdout crossing; validation frames never enter the fit.
        if j == FIT_POSES:
            continue
        a = integrate(t, gyro, stamps[j-1], stamps[j], bias)
        b = poses[j-1] @ poses[j].T
        aa, ba = angle(a), angle(b)
        if not 5 <= min(aa, ba) <= max(aa, ba) <= 80 or abs(aa-ba) > 2:
            rejected.append(dict(images=[j, j+1], imu_deg=aa, camera_deg=ba,
                                 reason='角度不在 5～80° 或两路相差超过 2°'))
            continue
        (fit if j < FIT_POSES else holdout).append((a, b))
        accepted.append(dict(images=[j, j+1], split='fit' if j < FIT_POSES else 'holdout'))
    x, ratios = solve_rotation(fit)
    # Validate excitation independently without using held-out data to update X.
    if len(holdout) < 6:
        raise ValueError('独立验证有效转动不足 6 段，请重拍完整多轴验证序列')
    _, holdout_ratios = solve_rotation(holdout)
    fit_errors, checks = residuals(fit, x), residuals(holdout, x)
    warnings = []
    if max(fit_errors) > 2 or np.median(checks) > 1 or max(checks) > 2:
        warnings.append('旋转残差超过默认验收门限：拟合/验证最大 2°，验证中位数 1°；检查同步、板是否移动及刚性连接。')
    if jitter > 50:
        warnings.append('IMU 主机收帧相对设备时钟抖动超过 50 ms，需检查串口负载或同步。')
    report = dict(mode='handeye', status='needs_recapture' if warnings else 'candidate_requires_validation',
        equation='A_ij R_imu_from_camera = R_imu_from_camera B_ij',
        rotation_convention='v_imu_device = R_imu_from_camera @ v_camera_optical',
        R_imu_from_camera=x.tolist(), R_camera_from_imu=x.T.tolist(), translation_estimated=False,
        fit_pair_count=len(fit), holdout_pair_count=len(holdout), fit_errors_deg=fit_errors,
        holdout_errors_deg=checks, fit_axis_ratios=ratios, holdout_axis_ratios=holdout_ratios,
        gyro_bias_rad_s=bias.tolist(), pnp_rms_px=rms, rejected_pairs=rejected, accepted_pairs=accepted,
        image_size=list(size), measurement=measurement, timestamp_jitter_ms=jitter,
        timestamp_method='IMU device intervals mapped by median receive-clock offset; camera read completion CLOCK_MONOTONIC',
        warnings=warnings, session=str(session), rotation_file=str(session/'camera_imu_rotation.yaml'),
        note='仅相机光学系到原始 imu_device 系的旋转；不包含平移、车体高度或地面映射，未替换主程序 camera.yaml。')
    fs = cv2.FileStorage(str(session/'camera_imu_rotation.yaml'), cv2.FILE_STORAGE_WRITE)
    if not fs.isOpened(): raise ValueError('无法保存旋转结果')
    fs.write('calibration_type', 'camera_imu_rotation_only')
    fs.write('source_frame', 'camera_optical'); fs.write('target_frame', 'imu_device')
    fs.write('R_imu_from_camera', x); fs.write('R_camera_from_imu', x.T)
    fs.write('translation_estimated', 0); fs.write('extrinsics_validated', 0)
    fs.write('status', report['status']); fs.release()
    save_json(session/'report.json', report)
    (session/'使用说明.txt').write_text(report['note']+'\n'+report['rotation_convention']+'\n'+
        '固定同一个相机俯仰机构角度采集与使用；转动相机舵机后此旋转不再适用。\n'+
        '结果不能直接作为主程序的 T_camera_from_robot 或 ground_homography。\n'+
        '板端已确认 IMU 到 base_link FLU 安装旋转为 diag(-1,1,-1)，需要时左乘该矩阵；\n'+
        '地面映射使用的 robot 坐标约定应另外核对，且仍需实测平移和独立地面验证。\n'+
        '离线重算：python3 tools/camera_calibration/guided.py --recalculate '+str(session)+'\n')
    return report


class Recorder:
    def __init__(self, args, session):
        self.rows, self.error = [], ''
        self.lock = threading.RLock()
        self.closed = False
        self.log = (session/'imu.jsonl').open('w', buffering=1)
        self.errlog = (session/'imu_stderr.log').open('w')
        try:
            self.process = subprocess.Popen([str(args.imu_recorder), args.imu_port, str(args.imu_baud), '3600'],
                stdout=subprocess.PIPE, stderr=self.errlog, text=True, bufsize=1)
        except Exception:
            self.log.close(); self.errlog.close(); raise
        self.worker = threading.Thread(target=self.read, daemon=True)
        self.worker.start()

    def read(self):
        try:
            for line in self.process.stdout:
                self.log.write(line)
                row = json.loads(line)
                with self.lock: self.rows.append(row)
            if not self.closed:
                self.error = 'IMU 记录进程退出，请检查串口占用和会话 imu_stderr.log'
        except Exception as exc:
            self.error = 'IMU 记录失败：'+str(exc)

    def snapshot(self):
        with self.lock: return list(self.rows)

    def check(self, stamp, initial=False):
        if self.error: raise ValueError(self.error)
        rows = self.snapshot()
        if not rows or time.monotonic()-rows[-1]['received_us']/1e6 > .2:
            raise ValueError('等待新鲜 IMU 数据，请检查串口或停止占用它的预览进程')
        # Bound online work; final solve checks the complete stream, including motion.
        recent = rows[-2000:]
        t, gyro, accel, _ = prepare_imu(recent)
        return stationary(t, gyro, accel, stamp, 5 if initial else 1.5,
                          None if initial else self.bias)

    def close(self):
        if self.closed: return
        self.closed = True
        if self.process.poll() is None: self.process.terminate()
        try: self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill(); self.process.wait()
        self.worker.join(timeout=3)
        self.process.stdout.close(); self.log.close(); self.errlog.close()


def make_wizard_class():
    # Lazy import avoids a cycle with guided.py's shared camera/web machinery.
    import guided as g
    class HandeyeWizard(g.Wizard):
        def __init__(self, args):
            super().__init__(args)
            self.recorder = None
            self.poses = []
            self.k, self.d, expected = calibrate.load_intrinsics(args.intrinsics)
            if expected != (args.width, args.height):
                raise ValueError('内参分辨率与当前相机模式不一致')

        def start(self, values):
            # Parent creates the session and camera; metadata adds immutable intrinsics.
            super().start(values)
            self.measurement.update(mode='handeye', intrinsics_source=str(self.args.intrinsics),
                camera_time_offset_ms=self.args.camera_time_offset_ms,
                imu_port=self.args.imu_port, imu_baud=self.args.imu_baud,
                camera_pitch_note='采集和使用期间保持同一相机舵机角度')
            calibrate.write_yaml(self.session/'intrinsics.yaml', self.k, self.d, self.size)
            g.save_json(self.session/'session.json', dict(measurement=self.measurement, steps=STEPS, captured=0))
            try:
                self.recorder = Recorder(self.args, self.session)
            except Exception as exc:
                self.camera_error = 'IMU 启动失败，请修复后重启本次会话：'+str(exc)
                self.stop_event.set()
                raise ValueError(self.camera_error) from exc

        def capture(self, expected_count):
            with self.lock:
                if self.session is None or self.recorder is None:
                    raise ValueError('请先开始 AX=XB 标定会话')
                if expected_count != len(self.observations):
                    raise ValueError('拍摄进度已变化，请刷新')
                if self.centers is None or self.frame_time == 0:
                    raise ValueError('等待完整圆点检测')
                stamp = self.frame_time+self.args.camera_time_offset_ms/1000
                bias = self.recorder.check(stamp, not self.observations)
                pose, _ = board_pose(self.centers, self.k, self.d, g.PATTERN,
                                      self.measurement['spacing_mm']/1000, self.poses[-1] if self.poses else None)
                if self.poses and angle(self.poses[-1] @ pose.T) < 5:
                    raise ValueError('相邻姿态旋转不足 5°；请整体转动，相机与 IMU 保持刚性连接')
                if not self.observations: self.recorder.bias = bias
                super().capture(expected_count)
                self.poses.append(pose)

        def undo(self, expected_count):
            with self.lock:
                super().undo(expected_count)
                self.poses.pop()

        def solve(self):
            try:
                self.recorder.close()
                result = calculate(self.session, self.observations, self.size, self.measurement)
                with self.lock: self.result = result; self.error = ''
            except Exception as exc:
                with self.lock: self.error = 'AX=XB 计算失败，原始数据已保留：'+str(exc)
            finally:
                with self.lock: self.busy = False

        def state(self):
            state = super().state()
            state['mode'] = 'handeye'
            rows = self.recorder.snapshot() if self.recorder else []
            state['imu_samples'] = len(rows)
            state['imu_fresh'] = bool(rows) and time.monotonic()-rows[-1]['received_us']/1e6 < .2
            if self.session and not self.result and not self.busy:
                try:
                    if self.recorder is None: raise ValueError('IMU 尚未启动')
                    self.recorder.check(self.frame_time+self.args.camera_time_offset_ms/1000, not self.observations)
                except ValueError as exc:
                    state['ready'] = False
                    state['error'] = state['error'] or str(exc)
            return state

        def close(self):
            super().close()
            if self.recorder: self.recorder.close()
    return HandeyeWizard
