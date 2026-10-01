#!/usr/bin/env python3
"""终端手动控制夹爪（不启动主程序）。

用法：gripper_ctl.py {status|open|close} [--port /dev/ttyACM0] [--baud 115200] [--timeout 3] [--dry-run]
先读A6反馈拿到下位机当前动作编号，新动作用“当前编号+1”（1..255循环，跳过0），
以20Hz重复发送同一包直到反馈“编号一致且done=1”或超时。速度固定为0；
相机pitch沿用读回角，避免顺带转动相机。
"""
import argparse, os, struct, sys, termios, time, tty

INVALID_PITCH = -32768
PITCH_LIMIT = 2500  # 暂时±25°：固件只有-25/0/+25三档；固件支持任意角后改回3500


def crc16(data):
    c = 0xFFFF
    for x in data:
        c ^= x
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c


def motion_packet(gripper_open, action_id, pitch):
    b = struct.pack('<BffBBh', 0x56, 0.0, 0.0, gripper_open, action_id, pitch)
    return b + struct.pack('<H', crc16(b))


class Feedback:
    """按帧头+8字节+CRC+帧尾0x0A解析A6，不按换行切分。"""
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        frames = []
        while len(self.buf) >= 8:
            if self.buf[0] != 0xA6:
                del self.buf[0]
                continue
            f = bytes(self.buf[:8])
            if crc16(f[:5]) == struct.unpack('<H', f[5:7])[0] and f[7] == 0x0A:
                frames.append((f[1], f[2], struct.unpack('<h', f[3:5])[0]))
                del self.buf[:8]
            else:
                del self.buf[0]
        return frames


def open_port(port, baud):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    a = termios.tcgetattr(fd)
    a[4] = a[5] = getattr(termios, 'B%d' % baud)
    termios.tcsetattr(fd, termios.TCSANOW, a)
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


def read_frames(fd, parser):
    try:
        return parser.feed(os.read(fd, 512))
    except BlockingIOError:
        return []


def wait_feedback(fd, parser, timeout):
    end = time.time() + timeout
    while time.time() < end:
        frames = read_frames(fd, parser)
        if frames:
            return frames[-1]
        time.sleep(0.01)
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('action', choices=['status', 'open', 'close'])
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--timeout', type=float, default=3.0, help='等待完成的秒数')
    ap.add_argument('--dry-run', action='store_true', help='只读反馈并打印将发送的包，不写串口')
    args = ap.parse_args()

    try:
        fd = open_port(args.port, args.baud)
    except OSError as e:
        sys.exit('打开 %s 失败：%s（网页串口助手或主程序是否占用？）' % (args.port, e))
    parser = Feedback()
    try:
        fb = wait_feedback(fd, parser, 0.5)
        if fb is None:
            sys.exit('0.5s内未收到A6反馈，检查下位机与串口')
        done, cur_id, pitch = fb
        pitch_txt = '无效' if pitch == INVALID_PITCH else '%.2f°' % (pitch / 100)
        print('当前: 动作编号=%d done=%d pitch=%s' % (cur_id, done, pitch_txt))
        if args.action == 'status':
            return

        new_id = 1 if cur_id >= 255 else cur_id + 1
        keep_pitch = 0 if pitch == INVALID_PITCH else max(-PITCH_LIMIT, min(PITCH_LIMIT, pitch))
        pkt = motion_packet(1 if args.action == 'open' else 0, new_id, keep_pitch)
        print('TX (%s, 编号%d): %s' % ('张开' if args.action == 'open' else '合上', new_id, pkt.hex(' ').upper()))
        if args.dry_run:
            return

        end = time.time() + args.timeout
        while time.time() < end:
            os.write(fd, pkt)
            for done, fb_id, _ in read_frames(fd, parser):
                if fb_id == new_id and done == 1:
                    print('完成: 编号%d' % new_id)
                    return
            time.sleep(0.05)
        sys.exit('超时%.1fs未确认完成（最后反馈 编号=%d done=%d）' % (args.timeout, fb_id if 'fb_id' in dir() else cur_id, done))
    finally:
        os.close(fd)


if __name__ == '__main__':
    main()
