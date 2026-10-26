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
    if angle not in (0, 20) or not 0 <= action_id <= 255 or not 90 <= pitch_rx <= 150:
        raise ValueError('Invalid actuator command')
    # Logical targets map to signed TX +5/-5; A6 semantics require separate confirmation.
    data = struct.pack('<BffbBh', 0x56, vx, wz, 5 if angle == 0 else -5, action_id, max(-30, min(30, pitch_rx - 120)))
    return data + struct.pack('<H', frame.crc16(data))


def valid_feedback(fb):
    return fb[0] in (0, 1) and 90 <= fb[2] <= 150


def initial_feedback(fd, parser, timeout=2):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        frames = frame.receive(fd, parser, .02)
        if frames:
            if not valid_feedback(frames[-1]):
                raise ValueError('Invalid A6 state or camera angle; no TX')
            return frames[-1]
    raise TimeoutError('No fresh CRC-valid A6 feedback; no TX')


def manual_motion(port, baud, mapping, vx, wz, duration=None, emit=print, known=None):
    # duration=None keeps the same vx/wz until Ctrl+C or demo.py stop; A6 staleness still stops the wheels.
    if duration is None:
        deadline = None
    else:
        if not math.isfinite(duration) or not 0 < duration <= 1:
            raise ValueError('Manual wheel motion duration must be in (0, 1] seconds')
        deadline = time.monotonic() + duration
    # Validate before opening any port.
    packet(vx, wz, 0, 0, 120)
    with frame.serial_port(port, baud, False) as fd:
        parser = frame.Feedback()
        fb = initial_feedback(fd, parser)
        angle = hold_angle(fb[0], mapping, known, fb[1])
        check_known_id(fb[1], known)
        target = packet(vx, wz, angle, fb[1], fb[2])
        stop = packet(0, 0, angle, fb[1], fb[2])
        emit(json.dumps({'event': 'INITIAL_RX', 'state': fb[0], 'action_id': fb[1],
                         'pitch_rx': fb[2], 'raw': fb[3].hex(' ')}))
        emit(json.dumps({'event': 'HOLD_TX_PLAN', 'vx': vx, 'wz': wz,
                         'gripper_offset': 5 if angle == 0 else -5, 'action_id': fb[1],
                         'pitch_tx': fb[2]-120, 'raw': target.hex(' ')}))
        started = last_rx = time.monotonic()
        command_recorded = False
        try:
            while deadline is None or time.monotonic() < deadline:
                now = time.monotonic()
                if now-last_rx > .2:
                    raise TimeoutError('A6 stale; wheel command stopped')
                frame.send_packet(fd, target, now + .1)
                if not command_recorded:
                    emit('GRIPPER_COMMAND_SENT ' + json.dumps({'gripper_offset': 5 if angle == 0 else -5, 'action_id': fb[1], 'vx': vx, 'wz': wz, 'pitch_tx': fb[2]-120, 'raw': target.hex(' '), 'meaning': 'last sent command; physical position unverified'}))
                    command_recorded = True
                for received in frame.receive(fd, parser, .04):
                    if not valid_feedback(received) or received[:2] != fb[:2] or received[2] != fb[2]:
                        raise ValueError(
                            'Actuator state/id/pitch changed during hold; wheel command stopped; '
                            'expected state=%d id=%d pitch=%d; received state=%d id=%d pitch=%d; RX=%s' %
                            (*fb[:3], *received[:3], received[3].hex(' ').upper()))
                    last_rx = time.monotonic()
                emit(json.dumps({'event': 'manual_motion', 'elapsed_s': time.monotonic()-started,
                                 'vx': vx, 'wz': wz, 'angle': angle, 'action_id': fb[1]}))
        finally:
            frame.send_packet(fd, stop, time.monotonic()+.15)
            emit('ZERO_SENT: gripper command offset and action ID preserved; physical position and stop are not measured')


def commission(port, baud, angle, emit=print):
    if angle not in (0, 20):
        raise ValueError('Commissioning supports only raw angles 0 and 20')
    with frame.serial_port(port, baud, False) as fd:
        parser = frame.Feedback()
        fb = initial_feedback(fd, parser)
        action_id = fb[1] % 255 + 1
        command = packet(0, 0, angle, action_id, fb[2])
        deadline = time.monotonic()+2
        command_recorded = False
        while time.monotonic() < deadline:
            frame.send_packet(fd, command, min(deadline, time.monotonic()+.1))
            if not command_recorded:
                emit('GRIPPER_COMMAND_SENT ' + json.dumps({'gripper_offset': 5 if angle == 0 else -5, 'action_id': action_id, 'vx': 0, 'wz': 0, 'pitch_tx': fb[2]-120, 'raw': command.hex(' '), 'meaning': 'last sent command; physical position unverified'}))
                command_recorded = True
            for received in frame.receive(fd, parser, .04):
                emit(json.dumps({'event': 'mapping_sample', 'command_angle': angle,
                                 'command_id': action_id, 'a6_state': received[0],
                                 'a6_id': received[1], 'pitch_rx': received[2]}))
                if valid_feedback(received) and received[1] == action_id:
                    emit('ID_ECHO_ONLY: record the physical action and raw A6 bit; mapping remains unverified')
                    return
        raise TimeoutError('No matching action ID; no reverse action sent')
