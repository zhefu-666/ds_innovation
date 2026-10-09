#!/usr/bin/env python3
"""Save one JPEG per interval from the local MJPEG bridge, named by epoch microseconds.
Waits for the stream to come up and reconnects if it drops, until the time budget ends."""
import sys, time, urllib.request
from pathlib import Path
out = Path(sys.argv[1]); seconds = float(sys.argv[2]); every = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
out.mkdir(parents=True, exist_ok=True)
end = time.time() + seconds; last = 0.0
while time.time() < end:
    buf = b''
    try:
        with urllib.request.urlopen('http://127.0.0.1:8080/stream.mjpg', timeout=5) as r:
            while time.time() < end:
                chunk = r.read(4096)
                if not chunk: break
                buf += chunk
                a = buf.find(b'\xff\xd8'); b = buf.find(b'\xff\xd9', a + 2) if a >= 0 else -1
                if a >= 0 and b >= 0:
                    jpg = buf[a:b + 2]; buf = buf[b + 2:]
                    now = time.time()
                    if now - last >= every:
                        (out / ('%d.jpg' % int(now * 1e6))).write_bytes(jpg); last = now
    except OSError:
        pass
    time.sleep(0.5)
