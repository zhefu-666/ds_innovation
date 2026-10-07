#!/usr/bin/env python3
"""旧夹爪入口兼容：open/close现在分别调用方框上升/下降。"""
from pathlib import Path
import runpy
if __name__ == '__main__':
    runpy.run_path(str(Path(__file__).resolve().parents[1] / 'frame/frame_ctl.py'), run_name='__main__')
