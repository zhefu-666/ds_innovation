#!/usr/bin/env python3
"""提前运行整场入口；主程序完成准备后直接等待A7或本地START。"""
import argparse
import os
from pathlib import Path
import shlex
import sys

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_COMMAND = ['/usr/bin/bash', str(ROOT / 'tools/match/match_main.sh'),
                   '--run', '--site-clear', '--frame', 'close']


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('command', nargs=argparse.REMAINDER,
                    help='可在 -- 后指定准备入口；默认运行 match_main.sh --run --site-clear --frame close')
    args = ap.parse_args()
    command = args.command
    if command and command[0] == '--':
        command = command[1:]
    command = command or DEFAULT_COMMAND
    print('PREPARE_PROJECT command=%s；准备完成后等待A7或match_ctl.py start。' %
          shlex.join(command), flush=True)
    try:
        os.chdir(ROOT)
        os.execvp(command[0], command)
    except OSError as exc:
        print('项目准备失败：%s' % exc, file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    sys.exit(main())
