#!/usr/bin/env python3
"""夹爪开合：open/up/raise=0（业务值，线上偏移+5），close/down/lower=20（线上偏移-5）；status只读。

A6状态字节的实际含义尚待固件确认；现有配置匹配仅作协议检查，不代表机械到位。

升降时车速固定为0，相机保持A6返回角度：TX=RX-120（下发偏移角，回传90..150）。
--dry-run只读反馈并预览，不发送。A6开闭语义由已验收配置提供，需匹配动作编号。
"""
import argparse
import json
from contextlib import contextmanager
import fcntl
import math
import os
import select
import struct
import sys
import termios
import time

from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from frame_semantics import ANGLES, DEFAULT_CONFIG, DONE_FLAG_NOTE, done_flag, load_mapping
ALIASES = {'raise': 'open', 'lower': 'close', 'up': 'open', 'down': 'close'}
TARGETS = {'open': (ANGLES['open'], '开'), 'close': (ANGLES['close'], '关')}


def crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xa001 if crc & 1 else crc >> 1
    return crc


# 线上航向字段 0.01°、-18000..18000；-32768 表示IMU不可用。
IMU_YAW_INVALID = -32768


def imu_yaw_wire(yaw_deg):
    """弧度以外的航向输入统一走这里：度数归一到(-180,180]后转0.01°。None表示无效。"""
    if yaw_deg is None:
        return IMU_YAW_INVALID
    yaw = (float(yaw_deg) + 180.0) % 360.0 - 180.0
    return max(-18000, min(18000, int(round(yaw * 100.0))))


def motion_packet(offset, action_id, pitch_deg, imu_yaw_deg=None):
    if offset not in (0, 20) or not 0 <= action_id <= 255 or not 90 <= pitch_deg <= 150:
        raise ValueError('夹爪目标、动作编号或相机实际角无效')
    # TX command offset is +5=open/-5=closed; this does not establish A6 semantics.
    payload = struct.pack('<BffbBh', 0x56, 0., 0., 5 if offset == 0 else -5, action_id, max(-30, min(30, pitch_deg - 120)))
    payload += struct.pack('<h', imu_yaw_wire(imu_yaw_deg))
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
    label = '原始A6状态，须按验收配置解释'
    print('RX: %s | 状态=%d(%s，按当前A6约定) 编号=%d 相机实际角=%d°' %
          (raw.hex(' ').upper(), state, label, action_id, pitch), flush=True)


def run(args):
    action = ALIASES.get(args.action, args.action)
    mapping = load_mapping(args.frame_config) if action != 'status' else None
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
        if state not in (0, 1) or not 90 <= pitch <= 150:
            raise ValueError('反馈状态或相机实际角无效，拒绝发送')
        offset, label = TARGETS[action]
        flag_mode = done_flag(mapping)
        target_state = 1 if flag_mode else mapping[action]
        if flag_mode:
            print(DONE_FLAG_NOTE + '; settle_ms=%d' % mapping['settle_ms'], flush=True)
        action_id = old_id if state == target_state else old_id % 255 + 1  # MCU仅在偏移变化时计数。
        if flag_mode:
            raise ValueError("当前实车为夹爪状态映射；done_flag配置已废弃")
        packet = motion_packet(offset, action_id, pitch)
        print('%s: 夹爪目标=%s（TX偏移%+d°），车速=0，相机保持RX%d°（TX偏移%d°），动作编号=%d\n包: %s' %
              (label, '开' if offset == 0 else '关', 5 if offset == 0 else -5, pitch, pitch-120, action_id, packet.hex(' ').upper()), flush=True)
        if args.dry_run:
            print('预览完成：TX=0，未发送任何命令。')
            return 0
        deadline = time.monotonic() + args.timeout
        next_send = 0.0
        last = current
        started = None
        sent_count = 0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                send_packet(fd, packet, deadline)
                sent_count += 1
                if sent_count == 1:
                    print('GRIPPER_COMMAND_SENT ' + json.dumps({'gripper_offset': 5 if offset == 0 else -5, 'action_id': action_id, 'vx': 0, 'wz': 0, 'pitch_tx': pitch-120, 'raw': packet.hex(' '), 'meaning': 'last sent command; physical position unverified'}), flush=True)
                started = now if started is None else started
                next_send = now + 0.05
            for feedback in receive(fd, parser, min(0.01, max(0., deadline - time.monotonic()))):
                if feedback[0:3] != last[0:3]:
                    show_feedback(feedback)
                last = feedback
                if feedback[0] == target_state and feedback[1] == action_id:
                    if flag_mode and time.monotonic() - started < mapping['settle_ms'] / 1000:
                        continue  # 新编号约10ms即回1，早于舵机转动；须等满settle
                    print('%s反馈已匹配（不等于独立机械到位检测）：编号%d、状态%d。' % (label, action_id, target_state))
                    if flag_mode:
                        print('KNOWN_FRAME angle=%d id=%d（TEMP_ASSUMPTION：后续传 --known-frame-angle %d --known-frame-id %d）' %
                              (offset, action_id, offset, action_id), flush=True)
                    return 0
        raise TimeoutError('%s反馈超时：目标编号%d/状态%d，最后编号%d/状态%d；已发送%d包；'
                           'TX=%s；最后RX=%s；停止重发，不反向动作。编号匹配不代表目标状态或机构已到位' %
                           (label, action_id, target_state, last[1], last[0], sent_count,
                            packet.hex(' ').upper(), last[3].hex(' ').upper()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['up', 'down', 'raise', 'lower', 'status', 'open', 'close'])
    parser.add_argument('--frame-config', default=str(DEFAULT_CONFIG))
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--baud', type=int, choices=[9600, 115200, 230400, 460800, 921600], default=115200)
    parser.add_argument('--timeout', type=float, default=3.)
    parser.add_argument('--dry-run', action='store_true', help='只读反馈并预览，TX=0')
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout必须为正数')
    try:
        return run(args)
    except TimeoutError as exc:
        print('失败：%s' % exc, file=sys.stderr)
        return 1
    except ValueError as exc:
        print('失败：%s' % exc, file=sys.stderr)
        return 1
    except OSError as exc:
        print('失败：%s（请确认主程序及其他串口工具未占用设备）' % exc, file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('\n已中止，不再发送。', file=sys.stderr)
        return 130


if __name__ == '__main__':
    sys.exit(main())
