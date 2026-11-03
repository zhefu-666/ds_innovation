#!/usr/bin/env python3
"""开机等待 CRC 有效的 A7 start=1，释放串口后启动一次整场程序。"""
import argparse
import os
from pathlib import Path
import shlex
import signal
import sys
import time

from watch_start import StartPackets, frame_ctl

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_COMMAND = ['/usr/bin/bash', str(ROOT / 'tools/match/match_main.sh'),
                   '--run', '--site-clear', '--frame', 'close']


def wait_for_start(port, baud):
    last_error = None
    while True:
        try:
            with frame_ctl.serial_port(port, baud, readonly=True) as fd:
                print('WAIT_A7 port=%s baud=%d；只读等待 start=1，过滤 A6。' %
                      (port, baud), flush=True)
                last_error = None
                parser = StartPackets()
                while True:
                    for raw in frame_ctl.receive(fd, parser, wait=0.2):
                        print('RX_A7 start=%d CRC=OK raw=%s' %
                              (raw[1], raw.hex(' ').upper()), flush=True)
                        if raw[1] == 1:
                            return raw
        except (OSError, frame_ctl.termios.error) as exc:
            message = str(exc)
            if message != last_error:
                print('WAIT_SERIAL %s；1 秒后重试。' % message,
                      file=sys.stderr, flush=True)
                last_error = message
            time.sleep(1)


def stop(_signum, _frame):
    raise KeyboardInterrupt


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('command', nargs=argparse.REMAINDER,
                    help='可在 -- 后指定启动命令；默认运行 match_main.sh --run --site-clear --frame close')
    args = ap.parse_args()
    if not hasattr(frame_ctl.termios, 'B%d' % args.baud):
        ap.error('不支持的波特率：%d' % args.baud)
    command = args.command
    if command and command[0] == '--':
        command = command[1:]
    command = command or DEFAULT_COMMAND
    signal.signal(signal.SIGTERM, stop)
    try:
        wait_for_start(args.port, args.baud)
        # with 已退出：串口恢复、解除独占并关闭后，才允许项目入口打开设备。
        print('START_PROJECT serial_released=1 command=%s' % shlex.join(command), flush=True)
        os.chdir(ROOT)
        os.execvp(command[0], command)
    except KeyboardInterrupt:
        print('A7 启动监听已停止。', flush=True)
        return 0
    except OSError as exc:
        print('项目启动失败：%s' % exc, file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    sys.exit(main())
