#!/bin/sh
# 旧入口兼容：改为方框升降工具。
exec python3 "$(dirname "$0")/../frame/frame_ctl.py" down "$@"
