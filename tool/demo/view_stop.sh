#!/usr/bin/env bash
set -euo pipefail
base="$(cd -- "$(dirname -- "$0")" && pwd)"
python3 - "$base" <<'PYCODE'
import os,signal,sys
from pathlib import Path
base=Path(sys.argv[1]);file=base/'view.pid'
if not file.exists():
 print('没有本入口记录的查看会话');raise SystemExit(0)
pid=int(file.read_text());cmd=Path('/proc/%d/cmdline'%pid)
if cmd.exists() and str(base/'view.py').encode() in cmd.read_bytes().split(b'\0'):
 os.kill(pid,signal.SIGTERM);print('已请求结束查看会话')
else:print('PID记录已失效，未发送信号')
PYCODE
