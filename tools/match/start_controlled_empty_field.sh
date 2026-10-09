#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
mode="${1:---check}"
case "$mode" in
  --check|--preview|--run|--auto-run) ;;
  *) echo 'Usage: start_controlled_empty_field.sh [--check|--preview|--run|--auto-run]' >&2; exit 2 ;;
esac
if (( $# > 1 )); then echo 'Only one mode is accepted.' >&2; exit 2; fi
binary=build-camera-offset-20261007/rescue_upper_host
common=(--controlled-empty-field --search-cues --team blue --no-show
  --calibration config/camera.yaml --model models/detect_fp.rknn
  --pose-model-blue models/zone_pose_fp.rknn
  --task-calibration config/task_calibration.40_runtime.json --startup-advance-ms 5000
  --startup-speed 1 --scan-wz 3 --turn-wz 3)
if [[ "$mode" == --check ]]; then exec "$binary" --check-config "${common[@]}"; fi
# All static checks run before anything opens a device or moves the pitch servo.
"$binary" --check-config "${common[@]}"
if [[ ! -c /dev/ttyACM0 ]]; then
  echo 'MCU serial device /dev/ttyACM0 is missing. Check MCU power/USB enumeration; /dev/ttyUSB0 is the IMU.' >&2
  exit 1
fi
mkdir -p telemetry_logs
run_log="telemetry_logs/controlled-$(date +%Y%m%d-%H%M%S)-$$.log"
echo "Run log: $PWD/$run_log (view from another terminal with tail -F)"
exec >> "$run_log" 2>&1
exec 9>/tmp/rescue-match-launch.lock
flock -n 9 || { echo 'Another match launcher is running.' >&2; exit 1; }
python3 tools/match/check_socket.py

source tools/match/telemetry_bridge.sh
if [[ "$mode" == --preview ]]; then
  stdbuf -oL -eL "$binary" --dry-run --imu --imu-port /dev/ttyUSB0 --pitch-feedback --telemetry "${common[@]}"
  exit $?
fi
cat <<'TXT'
CONTROLLED TEST ONLY: starting this mode confirms that only our own zone is present,
initial targets are outside it, forward/rear travel is cleared, and no people or
opponents may enter. Clearance and zone identity come from this operator setup.
Startup: command 1 m/s forward for 5 seconds, then search at 3 rad/s.
The nominal forward command distance is 5 m, not measured odometry.
Search uses all known colours as cues; select cargo near the cue.
First complete delivery must be green; afterwards black has priority. Closed-jaw clearing may push blue at 0.10 m/s for at most 0.3 s, twice; requires verified clearance.
No global localization is used. Measured-translation watchdog is disabled in this
mode; IMU/serial/frame health, phase budgets, 180-second timer and host STOP remain.
TXT
python3 tools/camera_pitch/pitch_ctl.py prepare45 --timeout 5
if [[ "$mode" == --auto-run ]]; then
  stdbuf -oL -eL "$binary" --hardware --imu --imu-port /dev/ttyUSB0 --telemetry --auto-run "${common[@]}"
  exit $?
fi
stdbuf -oL -eL "$binary" --hardware --imu --imu-port /dev/ttyUSB0 --telemetry "${common[@]}"
