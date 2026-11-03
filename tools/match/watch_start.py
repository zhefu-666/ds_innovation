#!/usr/bin/env python3
"""只读查看 A7 启动包；过滤 A6，不发送命令，也不启动任务流程。"""
import argparse
from datetime import datetime
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'frame'))
import frame_ctl


class StartPackets:
    """按长度和 CRC 拆分混合字节流，仅返回有效的 5 字节 A7 包。"""

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        packets = []
        while self.buffer:
            header = self.buffer[0]
            size = {0xA6: 8, 0xA7: 5}.get(header)
            if size is None:
                del self.buffer[0]
                continue
            if len(self.buffer) < size:
                break
            raw = bytes(self.buffer[:size])
            if (raw[-1] != 0x0A or
                    frame_ctl.crc16(raw[:-3]) != int.from_bytes(raw[-3:-1], 'little')):
                del self.buffer[0]
                continue
            # 整包跳过 A6，避免把其载荷中的 A7 当成启动帧。
            del self.buffer[:size]
            if header == 0xA7:
                packets.append(raw)
        return packets


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', default='/dev/ttyACM0', help='串口设备，默认 /dev/ttyACM0')
    ap.add_argument('--baud', type=int, default=115200, help='波特率，默认 115200')
    ap.add_argument('--duration', type=float, default=0,
                    help='监听秒数；默认 0 表示持续监听，Ctrl+C 退出')
    args = ap.parse_args()
    if not math.isfinite(args.duration) or args.duration < 0:
        ap.error('--duration 必须为有限的非负数')
    if not hasattr(frame_ctl.termios, 'B%d' % args.baud):
        ap.error('不支持的波特率：%d' % args.baud)

    count = 0
    try:
        with frame_ctl.serial_port(args.port, args.baud, readonly=True) as fd:
            parser = StartPackets()
            deadline = time.monotonic() + args.duration if args.duration else None
            print('只读监听 %s @ %d，仅显示 CRC 正确的 A7 包；Ctrl+C 退出。' %
                  (args.port, args.baud), file=sys.stderr, flush=True)
            while True:
                wait = 0.2
                if deadline is not None:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    wait = min(wait, remaining)
                for raw in frame_ctl.receive(fd, parser, wait):
                    count += 1
                    label = {0: '未按下', 1: '按键按下'}.get(raw[1], '未知值')
                    timestamp = datetime.now().astimezone().isoformat(timespec='milliseconds')
                    print('[%s] A7 #%d start=%d (%s) CRC=OK RX=%s' %
                          (timestamp, count, raw[1], label, raw.hex(' ').upper()), flush=True)
    except KeyboardInterrupt:
        pass
    except (OSError, frame_ctl.termios.error) as exc:
        print('监听失败：%s；请检查设备连接及串口是否被其他程序占用。' % exc,
              file=sys.stderr)
        return 1
    finally:
        print('监听结束：收到 %d 个有效 A7 包。' % count, file=sys.stderr, flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
