#!/usr/bin/env python3
"""Explicit temporary done-bit interpretation; bounded supervised commissioning only.

Never used to approve production calibration or infer startup position from A6.
The operator supplies the last known target and action ID from this session.
"""
import argparse
import json
import time
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
import demo_serial as serial
from demo import demo_lock


def next_target(angle, action_id, target):
    if angle not in (0, 20) or target not in (0, 20):
        raise ValueError('Only frame angles 0 and 20 are allowed')
    return action_id if angle == target else action_id % 255 + 1


def ack_matches(fb, action_id, pitch):
    return serial.valid_feedback(fb) and fb[0] == 1 and fb[1] == action_id and fb[2] == pitch


def transaction(fd, parser, current_angle, current_id, target_angle, pitch, emit=print):
    action_id = next_target(current_angle, current_id, target_angle)
    packet = serial.packet(0, 0, target_angle, action_id, pitch)
    emit(json.dumps({'event':'TX_PLAN', 'vx':0, 'wz':0, 'frame_angle':target_angle,
                     'action_id':action_id, 'pitch_tx':pitch-40, 'pitch_rx_target':pitch,
                     'temporary_semantics':'done_bit', 'raw':packet.hex(' ')}))
    start = time.monotonic()
    deadline = start + 2
    last_rx = start
    matched_since = None
    next_send = start
    last = None
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now-last_rx > .2:
            raise TimeoutError('A6 stale; stop this test, do not infer position')
        if now >= next_send:
            serial.frame.send_packet(fd, packet, min(deadline, now+.1))
            next_send = now+.05
        for fb in serial.frame.receive(fd, parser, .01):
            now = time.monotonic()
            last_rx = now
            if not serial.valid_feedback(fb):
                raise ValueError('Invalid A6 values')
            if fb[1] not in (current_id, action_id):
                raise ValueError('Unexpected action ID / possible MCU restart')
            if fb[:3] != last:
                emit(json.dumps({'event':'RX', 'elapsed_s':round(now-start,4),
                                 'done_assumed':fb[0], 'action_id':fb[1],
                                 'pitch_rx':fb[2], 'raw':fb[3].hex(' ')}))
                last=fb[:3]
            if ack_matches(fb, action_id, pitch):
                if matched_since is None:
                    matched_since=now
                if now-matched_since >= .15:
                    emit(json.dumps({'event':'PROTOCOL_ACK_ONLY','frame_command':target_angle,
                                     'action_id':action_id,'pitch_rx':pitch,
                                     'physical_position_verified':False}))
                    return target_angle,action_id
            else:
                matched_since=None
    raise TimeoutError('No stable matching done/id/pitch feedback within 2 seconds')


def wheel_transaction(fd, parser, angle, action_id, pitch, vx, wz, duration, emit=print):
    if not 0 < duration <= .5 or abs(vx) > .05 or abs(wz) > .20 or (vx and wz):
        raise ValueError('Wheel probe limited to one axis, .05 m/s or .20 rad/s, .5 seconds')
    packet=serial.packet(vx,wz,angle,action_id,pitch)
    zero=serial.packet(0,0,angle,action_id,pitch)
    start=last_rx=time.monotonic()
    try:
        while time.monotonic()-start < duration:
            now=time.monotonic()
            if now-last_rx>.15:
                raise TimeoutError('A6 stale during wheel probe')
            serial.frame.send_packet(fd,packet,now+.05)
            for fb in serial.frame.receive(fd,parser,.04):
                if not ack_matches(fb,action_id,pitch):
                    raise ValueError('Actuator feedback changed during wheel hold')
                last_rx=time.monotonic()
            emit(json.dumps({'event':'WHEEL_TX','vx':vx,'wz':wz,'angle':angle,
                             'action_id':action_id,'elapsed_s':round(time.monotonic()-start,4)}))
    finally:
        serial.frame.send_packet(fd,zero,time.monotonic()+.15)
        emit('ZERO_SENT: physical wheel stopping not measured by A6')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--known-angle',type=int,choices=[0,20],required=True)
    p.add_argument('--expected-id',type=int,required=True)
    p.add_argument('--expected-pitch-rx',type=int,required=True)
    p.add_argument('--target-angle',type=int,choices=[0,20])
    p.add_argument('--pitch-rx',type=int,choices=range(0,81))
    p.add_argument('--site-clear',action='store_true')
    p.add_argument('--wheels-disabled',action='store_true')
    p.add_argument('--assume-done-bit',action='store_true')
    p.add_argument('--run',action='store_true')
    p.add_argument('--stationary-supervised',action='store_true',help='Known-position zero-speed actuator test after physical direction confirmation')
    p.add_argument('--wheel',choices=['forward','backward','left','right'])
    p.add_argument('--watchdog-tested',action='store_true',help='Operator has confirmed a measured watchdog upper bound <=200ms')
    a=p.parse_args()
    if not 1 <= a.expected_id <= 255 or not 0 <= a.expected_pitch_rx <= 80:
        p.error('Require a known nonzero action ID and valid RX pitch')
    angle=a.known_angle if a.target_angle is None else a.target_angle
    pitch=a.expected_pitch_rx if a.pitch_rx is None else a.pitch_rx
    if sum([a.target_angle is not None,a.pitch_rx is not None,a.wheel is not None]) > 1:
        p.error('Change only one actuator per test')
    if not a.run:
        print('PREVIEW NO DEVICE ACCESS',angle,next_target(a.known_angle,a.expected_id,angle),pitch)
        return
    if not(a.site_clear and a.assume_done_bit):
        p.error('Require explicit temporary semantics and site clear')
    if a.wheel:
        if not a.watchdog_tested or a.wheels_disabled:
            p.error('Wheel test requires measured watchdog and available wheels')
    elif not(a.wheels_disabled or a.stationary_supervised):
        p.error('Require wheels disabled or explicit supervised known-position test')
    with demo_lock('done-assumption-probe'):
        with serial.frame.serial_port('/dev/ttyACM0',115200,False) as fd:
            parser=serial.frame.Feedback()
            fb=serial.initial_feedback(fd,parser)
            print(json.dumps({'event':'INITIAL_RX','raw':fb[3].hex(' '),'known_angle_source':'explicit_session_history'}),flush=True)
            if not ack_matches(fb,a.expected_id,a.expected_pitch_rx):
                raise ValueError('Startup differs from session history; NO TX')
            emit=lambda line:print(line,flush=True)
            if a.wheel:
                vx,wz={'forward':(.05,0),'backward':(-.05,0),'left':(0,.20),'right':(0,-.20)}[a.wheel]
                wheel_transaction(fd,parser,a.known_angle,a.expected_id,a.expected_pitch_rx,vx,wz,.5,emit)
            else:
                transaction(fd,parser,a.known_angle,a.expected_id,angle,pitch,emit)


if __name__=='__main__':
    try:
        main()
    except (OSError,ValueError,TimeoutError) as exc:
        print('ABORT: '+str(exc),file=sys.stderr)
        sys.exit(2)
