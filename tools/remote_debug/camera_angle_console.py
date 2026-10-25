#!/usr/bin/env python3
"""Zero-speed camera console for the 2026-10-23 MCU protocol."""
import argparse
import math
import struct
import sys
import time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'frame'))
import frame_ctl as wire


def packet(angle, hold, ident):
    if not math.isfinite(angle) or not 90 <= angle <= 150:
        raise ValueError('角度须为90..150度；120中位，增大向下')
    if not -5 <= hold <= 5 or not 0 <= ident <= 255:
        raise ValueError('夹爪偏移或编号无效')
    target = int(math.floor(angle + .5))
    data = struct.pack('<BffbBh', 0x56, 0., 0., hold, ident, target - 120)
    return data + struct.pack('<H', wire.crc16(data)), target


def feedback(fd, parser, duration=.3):
    frames = []
    end = time.monotonic() + duration
    while time.monotonic() < end:
        frames.extend(wire.receive(fd, parser, .02))
    if not frames or not 90 <= frames[-1][2] <= 150:
        raise TimeoutError('未收到有效A6相机反馈；未发送新命令')
    return frames[-1]


def show(f):
    print('A6: 相机=%d° 偏移=%+d° 夹爪状态=%d 编号=%d  CRC OK [%s]' %
          (f[2], f[2]-120, f[0], f[1], f[3].hex(' ')), flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--hold-gripper-offset', type=int, choices=range(-5, 6))
    ap.add_argument('--status', action='store_true', help='只读一次状态后退出')
    args = ap.parse_args()
    with wire.serial_port(args.port, args.baud, args.status) as fd:
        parser = wire.Feedback()
        initial = feedback(fd, parser)
        show(initial)
        if args.status:
            return
        hold = args.hold_gripper_offset
        if hold is None:
            if initial[1] != 0:
                raise ValueError('编号非0，无法推断夹爪偏移；请显式提供已知--hold-gripper-offset')
            hold = 0  # MCU reports no previous action; confirmed neutral session.
        print('零车速；保持夹爪偏移%+d°。输入90..150设角度，s读状态，q退出。' % hold, flush=True)
        while True:
            try:
                line = input('相机角度> ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                print('\n串口已关闭；不自动改变退出角度。')
                return
            if line in ('q', 'quit', 'exit'):
                return
            if not line:
                continue
            current = feedback(fd, parser)
            if current[:2] != initial[:2]:
                raise RuntimeError('夹爪状态/编号发生变化，停止调试')
            if line in ('s', 'status'):
                show(current)
                continue
            try:
                data, target = packet(float(line), hold, current[1])
            except ValueError as exc:
                print(exc)
                continue
            end = time.monotonic()+3
            next_tx = 0
            last_rx = time.monotonic()
            stable = 0
            reached = False
            while time.monotonic() < end:
                now = time.monotonic()
                if now-last_rx > .2:
                    raise TimeoutError('A6反馈超过200ms未更新，停止发送')
                if now >= next_tx:
                    wire.send_packet(fd, data, now+.03)
                    next_tx = now+.05
                for f in wire.receive(fd, parser, .01):
                    last_rx = time.monotonic()
                    if f[:2] != initial[:2] or not 90 <= f[2] <= 150:
                        raise RuntimeError('反馈状态变化或角度无效，停止发送')
                    stable = stable+1 if f[2] == target else 0
                    if stable >= 5:
                        show(f)
                        print('目标%d°，连续5帧回读一致；TX偏移%+d°。' % (target,target-120), flush=True)
                        reached = True
                        break
                if reached:
                    break
            if not reached:
                print('3秒内未取得目标角稳定反馈，停止发送。')
                show(feedback(fd, parser))


if __name__ == '__main__':
    try:
        main()
    except (OSError, RuntimeError, ValueError) as exc:
        sys.exit(str(exc))
