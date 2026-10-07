#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."

bridge_pid=
cleanup() {
  if [[ -n "$bridge_pid" ]]; then
    kill "$bridge_pid" 2>/dev/null || true
    wait "$bridge_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

bridge_ready() {
  python3 - <<'PY'
import json
import asyncio
import sys
from urllib.request import urlopen
import websockets

try:
    with urlopen('http://127.0.0.1:8080/status', timeout=1) as response:
        status = json.load(response)
    if status.get('health', {}).get('source') != 'rescue_upper_host':
        raise ValueError('unexpected telemetry source')
    async def probe():
        async with websockets.connect('ws://127.0.0.1:8765',
                                      subprotocols=['foxglove.websocket.v1'],
                                      open_timeout=1) as ws:
            if ws.subprotocol != 'foxglove.websocket.v1':
                raise ValueError('unexpected WebSocket protocol')
    asyncio.run(probe())
except Exception:
    sys.exit(1)
PY
}

if bridge_ready; then
  echo 'Foxglove bridge already running on 8765; reusing it.'
else
  python3 -u tools/remote_camera_telemetry/project_bridge.py &
  bridge_pid=$!
  ready=0
  for ((i=0; i<30; ++i)); do
    if bridge_ready; then ready=1; break; fi
    if ! kill -0 "$bridge_pid" 2>/dev/null; then break; fi
    sleep 0.2
  done
  if (( ! ready )); then
    echo 'Foxglove bridge did not become ready on 8080/8765.' >&2
    exit 1
  fi
fi

echo 'Foxglove: ws://192.168.34.7:8765'
echo 'Starting hardware host; match auto-starts only after live preflight passes.'
bash tools/match/start_hardware_blue.sh --auto-run
