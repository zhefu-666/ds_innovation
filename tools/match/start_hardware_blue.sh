#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
if (( $# > 1 )) || { (( $# == 1 )) && [[ "$1" != "--auto-run" ]]; }; then
  echo "Usage: bash tools/match/start_hardware_blue.sh [--auto-run]" >&2
  exit 2
fi

mkdir -p telemetry_logs
run_log="telemetry_logs/hardware-$(date +%Y%m%d-%H%M%S)-$$.log"
echo "Run log: $PWD/$run_log (view from another terminal with tail -F)"
exec >> "$run_log" 2>&1

exec 9>/tmp/rescue-match-launch.lock
flock -n 9 || { echo 'Another match launcher is running.' >&2; exit 1; }
python3 tools/match/check_socket.py

# The pitch tool owns the MCU port until 45-degree A6 feedback is stable.
python3 tools/camera_pitch/pitch_ctl.py prepare45 --timeout 5

exec build-camera-offset-20261007/rescue_upper_host \
  --hardware --no-show --telemetry \
  --team blue --imu --imu-port /dev/ttyUSB0 \
  --calibration config/camera.yaml \
  --model models/detect_fp.rknn \
  --pose-model-blue models/zone_pose_fp.rknn \
  --task-calibration config/task_calibration.40_runtime.json \
  --startup-advance-ms 5000 --startup-speed 1 "$@"
