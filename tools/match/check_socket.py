#!/usr/bin/env python3
"""Remove only an owned, unbound stale match socket. Sends no commands."""
import errno
import os
from pathlib import Path
import socket
import stat

p = Path('/tmp/rescue-match.sock')
try:
    before = p.lstat()
except FileNotFoundError:
    raise SystemExit(0)
if not stat.S_ISSOCK(before.st_mode) or before.st_uid != os.getuid():
    raise SystemExit('Refusing to remove a non-socket or foreign-owned match path')
with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as probe:
    try:
        probe.connect(str(p))
    except OSError as exc:
        if exc.errno == errno.ENOENT:
            raise SystemExit(0)
        if exc.errno != errno.ECONNREFUSED:
            raise SystemExit('Cannot verify match socket: ' + str(exc))
    else:
        raise SystemExit('Match socket is active; stop the existing host before starting another')
after = p.lstat()
if (before.st_dev, before.st_ino) != (after.st_dev, after.st_ino):
    raise SystemExit('Match socket changed during check; refusing cleanup')
p.unlink()
print('Removed stale match socket:', p)
