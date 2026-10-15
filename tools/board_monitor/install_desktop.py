#!/usr/bin/env python3
"""Install a read-only monitor launcher for the current desktop user (no root)."""
import datetime
import os
from pathlib import Path
import shutil
import subprocess

base=Path(__file__).resolve().parent
try:
    desktop=Path(subprocess.check_output(['xdg-user-dir','DESKTOP'],text=True).strip())
except (OSError,subprocess.SubprocessError):
    desktop=Path.home()/'Desktop'
if not desktop.is_absolute() or desktop==Path.home():
    desktop=Path.home()/'Desktop'
root=base.parent.parent
entry='''[Desktop Entry]
Version=1.0
Type=Application
Name=机器人实时监看
Comment=只读显示比赛画面、状态与日志
Exec=python3 "{script}" --root "{root}" --open
Icon=utilities-system-monitor
Terminal=false
Categories=Utility;
StartupNotify=false
'''.format(script=str(base/'monitor.py'),root=str(root))
# Desktop Entry command quoting is distinct from shell quoting.
if any(c in str(base) for c in ['"','`','$','\\','\n']):
    raise SystemExit('unsupported installation path')
backup=Path.home()/('backup-board-monitor-desktop-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
for folder in (Path.home()/'.local/share/applications',desktop):
    folder.mkdir(parents=True,exist_ok=True)
    dest=folder/'rescue-board-monitor.desktop'
    if dest.exists():
        backup.mkdir(exist_ok=True)
        shutil.copy2(dest,backup/(folder.name+'-rescue-board-monitor.desktop'))
    dest.write_text(entry)
    dest.chmod(0o755)
    # Trust metadata may not be supported (e.g. headless install); executable launcher still exists.
    if shutil.which('gio'):
        subprocess.run(['gio','set',str(dest),'metadata::trusted','true'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    print(dest)
