#!/usr/bin/env python3
"""旧夹爪入口兼容：open=0°、close=20°；A6由验收配置定义。"""
from pathlib import Path
import runpy
if __name__ == '__main__':
    runpy.run_path(str(Path(__file__).resolve().parents[1] / 'frame/frame_ctl.py'), run_name='__main__')
