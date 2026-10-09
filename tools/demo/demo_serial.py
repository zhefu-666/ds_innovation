#!/usr/bin/env python3
"""Bounded supervised motion and zero-speed commissioning; never imported to send."""
import importlib.util
import json
import math
from pathlib import Path
import struct
import time

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('demo_frame_ctl', ROOT / 'tools/frame/frame_ctl.py')
frame = importlib.util.module_from_spec(spec)
spec.loader.exec_module(frame)
from frame_semantics import check_known_id, hold_angle


def packet(vx, wz, angle, action_id, pitch_rx):
    if not all(math.isfinite(v) for v in (vx, wz)) or abs(vx) > .10 or abs(wz) > .25:
        raise ValueError('Demo velocity exceeds 0.10 m/s or 0.25 rad/s')
    if angle not in (0, 20) or not 0 <= action_id <= 255 or not 0 <= pitch_rx <= 80:
        raise ValueError('Invalid actuator command')
    data = struct.pack('<BffbBh', 0x56, vx, wz, angle, action_id, pitch_rx - 40)
    return data + struct.pack('<H', frame.crc16(data))


def valid_feedback(fb):
    return fb[0] in (0, 1) and 0 <= fb[2] <= 80


def initial_feedback(fd, parser, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        frames = frame.receive(fd, parser, .02)
        if frames:
            if not valid_feedback(frames[-1]):
                raise ValueError('Invalid A6 state or camera angle; no TX')
            return frames[-1]
    raise TimeoutError('No fresh CRC-valid A6 feedback; no TX')


def manual_motion(port, baud, mapping, vx, wz, duration, emit=print, known=None):
    if not math.isfinite(duration) or not 0 < duration <= 1:
        raise ValueError('Manual wheel motion duration must be in (0, 1] seconds')
    # Validate before opening any port.
    packet(vx, wz, 0, 0, 40)
    with frame.serial_port(port, baud, False) as fd:
        parser = frame.Feedback()
        fb = initial_feedback(fd, parser)
        angle = hold_angle(fb[0], mapping, known)
        check_known_id(fb[1], known)
        target = packet(vx, wz, angle, fb[1], fb[2])
        stop = packet(0, 0, angle, fb[1], fb[2])
        started = last_rx = time.monotonic()
        try:
            while time.monotonic() - started < duration:
                now = time.monotonic()
                if now-last_rx > .2:
                    raise TimeoutError('A6 stale; wheel command stopped')
                frame.send_packet(fd, target, now + .1)
                for received in frame.receive(fd, parser, .04):
                    if not valid_feedback(received) or received[:2] != fb[:2] or received[2] != fb[2]:
                        raise ValueError('Actuator state/id/pitch changed during hold; wheel command stopped')
                    last_rx = time.monotonic()
                emit(json.dumps({'event': 'manual_motion', 'elapsed_s': time.monotonic()-started,
                                 'vx': vx, 'wz': wz, 'angle': angle, 'action_id': fb[1]}))
        finally:
            frame.send_packet(fd, stop, time.monotonic()+.15)
            emit('ZERO_SENT: last frame angle and action ID preserved; physical stop is not measured')


def commission(port, baud, angle, emit=print):
    if angle not in (0, 20):
        raise ValueError('Commissioning supports only raw angles 0 and 20')
    with frame.serial_port(port, baud, False) as fd:
        parser = frame.Feedback()
        fb = initial_feedback(fd, parser)
        action_id = fb[1] % 255 + 1
        command = packet(0, 0, angle, action_id, fb[2])
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            frame.send_packet(fd, command, min(deadline, time.monotonic()+.1))
            for received in frame.receive(fd, parser, .04):
                emit(json.dumps({'event': 'mapping_sample', 'command_angle': angle,
                                 'command_id': action_id, 'a6_state': received[0],
                                 'a6_id': received[1], 'pitch_rx': received[2]}))
                if valid_feedback(received) and received[1] == action_id:
                    emit('ID_ECHO_ONLY: record the physical action and raw A6 bit; mapping remains unverified')
                    return
        raise TimeoutError('No matching action ID; no reverse action sent')
