#!/usr/bin/env bash
# 旧入口：起遥测桥后跑硬件蓝方预览。桥逻辑统一放在 telemetry_bridge.sh，避免两份实现分叉。
set -euo pipefail
cd "$(dirname "$0")/../.."

source tools/match/telemetry_bridge.sh

echo 'Starting hardware host; match auto-starts only after live preflight passes.'
bash tools/match/start_hardware_blue.sh --auto-run
