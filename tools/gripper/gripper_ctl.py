#!/usr/bin/env python3
"""夹爪入口：open/close发送有符号偏移+5/-5；A6反馈仍为1/0。"""
from pathlib import Path
import runpy
if __name__ == '__main__':
    runpy.run_path(str(Path(__file__).resolve().parents[1] / 'frame/frame_ctl.py'), run_name='__main__')
