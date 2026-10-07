#!/usr/bin/env python3
"""方框升降：up/raise=+20°，down/lower=0°；status只读。

升降时车速固定为0，相机保持A6返回角度：TX=RX-40（下发偏移角，回传0..80）。
--dry-run只读反馈并预览，不发送。A6暂按0=放下、1=抬起，需匹配动作编号。
"""
import argparse
from contextlib import contextmanager
import fcntl
import math
import os
import select
import struct
import sys
import termios
import time

ALIASES = {'raise': 'up', 'lower': 'down', 'open': 'up', 'close': 'down'}
TARGETS = {'up': (20, 1, '上升'), 'down': (0, 0, '下降')}


def crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xa001 if crc & 1 else crc >> 1
    return crc


def motion_packet(offset, action_id, pitch_deg):
    if offset not in (0, 20) or not 0 <= action_id <= 255 or not 0 <= pitch_deg <= 80:
        raise ValueError('方框目标、动作编号或相机实际角无效')
    payload = struct.pack('<BffbBh', 0x56, 0., 0., offset, action_id, pitch_deg - 40)
    return payload + struct.pack('<H', crc16(payload))


class Feedback:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        result = []
        while len(self.buffer) >= 8:
            raw = bytes(self.buffer[:8])
            if (raw[0] == 0xa6 and raw[7] == 10 and
                    crc16(raw[:5]) == int.from_bytes(raw[5:7], 'little')):
                result.append((raw[1], raw[2], int.from_bytes(raw[3:5], 'little', signed=True), raw))
                del self.buffer[:8]
            else:
                del self.buffer[0]
        return result


@contextmanager
def serial_port(port, baud, readonly):
    fd = os.open(port, (os.O_RDONLY if readonly else os.O_RDWR) | os.O_NOCTTY | os.O_NONBLOCK)
    old = None
    exclusive = False
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        exclusive = True
        old = termios.tcgetattr(fd)
        a = termios.tcgetattr(fd)
        a[0] = a[1] = a[3] = 0
        a[2] = (a[2] & ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | termios.CRTSCTS)) | termios.CS8 | termios.CREAD | termios.CLOCAL
        a[4] = a[5] = getattr(termios, 'B%d' % baud)
        a[6][termios.VMIN] = a[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, a)
        termios.tcflush(fd, termios.TCIFLUSH)
        yield fd
    finally:
        try:
            if old is not None:
                termios.tcsetattr(fd, termios.TCSANOW, old)
        finally:
            try:
                if exclusive:
                    fcntl.ioctl(fd, termios.TIOCNXCL)
            finally:
                os.close(fd)


def receive(fd, parser, wait=0.05):
    if not select.select([fd], [], [], wait)[0]:
        return []
    try:
        data = os.read(fd, 4096)
    except BlockingIOError:
        return []
    if not data:
        raise OSError('串口已断开')
    return parser.feed(data)


def send_packet(fd, packet, deadline):
    pending = memoryview(packet)
    while pending:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('串口写入超时')
        if select.select([], [fd], [], min(0.05, remaining))[1]:
            try:
                n = os.write(fd, pending)
            except BlockingIOError:
                continue
            if n <= 0:
                raise OSError('串口写入失败')
            pending = pending[n:]


def show_feedback(feedback):
    state, action_id, pitch, raw = feedback
    label = {0: '放下', 1: '抬起'}.get(state, '未知')
    print('RX: %s | 状态=%d(%s，按当前A6约定) 编号=%d 相机实际角=%d°' %
          (raw.hex(' ').upper(), state, label, action_id, pitch), flush=True)


def run(args):
    action = ALIASES.get(args.action, args.action)
    with serial_port(args.port, args.baud, action == 'status' or args.dry_run) as fd:
        parser = Feedback()
        deadline = time.monotonic() + args.timeout
        current = None
        while time.monotonic() < deadline and current is None:
            frames = receive(fd, parser)
            if frames:
                current = frames[-1]
        if current is None:
            raise TimeoutError('未收到CRC有效A6反馈，未发送任何命令')
        show_feedback(current)
        if action == 'status':
            return 0
        state, old_id, pitch, _ = current
        if state not in (0, 1) or not 0 <= pitch <= 80:
            raise ValueError('反馈状态或相机实际角无效，拒绝发送')
        offset, target_state, label = TARGETS[action]
        action_id = old_id % 255 + 1  # Explicit manual request: always issue a fresh action.
        packet = motion_packet(offset, action_id, pitch)
        print('%s: 方框=%+d°，车速=0，相机保持RX%d°（TX偏移%d°），动作编号=%d\n包: %s' %
              (label, offset, pitch, pitch-40, action_id, packet.hex(' ').upper()), flush=True)
        if args.dry_run:
            print('预览完成：TX=0，未发送任何命令。')
            return 0
        deadline = time.monotonic() + args.timeout
        next_send = 0.0
        last = current
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                send_packet(fd, packet, deadline)
                next_send = now + 0.05
            for feedback in receive(fd, parser, min(0.01, max(0., deadline - time.monotonic()))):
                if feedback[0:3] != last[0:3]:
                    show_feedback(feedback)
                last = feedback
                if feedback[0] == target_state and feedback[1] == action_id:
                    print('%s反馈已匹配（不等于独立机械到位检测）：编号%d、状态%d。' % (label, action_id, target_state))
                    return 0
        raise TimeoutError('%s反馈超时：目标编号%d/状态%d，最后编号%d/状态%d；停止重发，不反向动作' %
                           (label, action_id, target_state, last[1], last[0]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['up', 'down', 'raise', 'lower', 'status', 'open', 'close'])
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--baud', type=int, choices=[9600, 115200, 230400, 460800, 921600], default=115200)
    parser.add_argument('--timeout', type=float, default=3.)
    parser.add_argument('--dry-run', action='store_true', help='只读反馈并预览，TX=0')
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout必须为正数')
    try:
        return run(args)
    except (OSError, ValueError, TimeoutError) as exc:
        print('失败：%s（请确认主程序及其他串口工具未占用设备）' % exc, file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('\n已中止，不再发送。', file=sys.stderr)
        return 130


if __name__ == '__main__':
    sys.exit(main())
