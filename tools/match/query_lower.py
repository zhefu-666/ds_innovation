#!/usr/bin/env python3
"""启动前只读查询下位机A6反馈：O_RDONLY，TX=0，不发送任何字节。"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'frame'))
import frame_ctl


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--timeout', type=float, default=3.)
    ap.add_argument('--samples', type=int, default=5)
    args = ap.parse_args()
    frames = []
    try:
        with frame_ctl.serial_port(args.port, args.baud, True) as fd:
            parser = frame_ctl.Feedback()
            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline and len(frames) < args.samples:
                frames += frame_ctl.receive(fd, parser)
    except OSError as exc:
        print('失败：%s（请确认主程序及其他串口工具未占用设备）' % exc, file=sys.stderr)
        return 1
    if not frames:
        print('失败：%.0f秒内未收到CRC有效的A6反馈（检查MCU供电/串口/是否被占用）' % args.timeout, file=sys.stderr)
        return 1
    state, action_id, pitch, raw = frames[-1]
    stable = int(all(f[:3] == frames[-1][:3] for f in frames))
    print('[下位机] 最近A6帧 %s' % raw.hex(' ').upper())
    print('LOWER_STATE state=%d id=%d pitch=%d samples=%d stable=%d' % (state, action_id, pitch, len(frames), stable))
    print('[下位机] 完成位=%d  方框动作编号=%d  相机实际角=%d°(RX 0..80)' % (state, action_id, pitch))
    if not stable:
        print('[下位机] 警告：采样期间反馈在变化，可能有其他程序正在控制下位机')
    if action_id == 0:
        print('[下位机] 编号0=下位机本次上电后还没收到过方框动作；A6不含方框位置，实际是0°还是20°只能由现场确认')
    else:
        print('[下位机] A6不含方框位置，只能确认最后一次动作编号；实际角度仍以现场为准')
    return 0


if __name__ == '__main__':
    sys.exit(main())
