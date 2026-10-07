#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
mode="${1:---check}"
case "$mode" in
  --check|--preview|--run|--auto-run) ;;
  *) echo 'Usage: start_capture_scan.sh [--check|--preview|--run|--auto-run] [--controlled-ignore-clearance]' >&2; exit 2 ;;
esac
extra=()
if (( $# > 2 )); then echo 'Too many arguments.' >&2; exit 2; fi
if (( $# == 2 )); then
  [[ "$2" == --controlled-ignore-clearance ]] || { echo 'Unknown contact-test option.' >&2; exit 2; }
  extra+=(--controlled-ignore-clearance)
fi
binary=build-camera-offset-20261007/rescue_upper_host
common=(--no-match-time-limit --controlled-empty-field --search-cues --team blue --no-show
  --allow-mechanical-pitch-model --pitch-presets 500,4000,4000
  --calibration config/camera.mechanical_5_40.yaml --model models/detect_fp.rknn
  --pose-model-blue models/zone_pose_fp.rknn
  --task-calibration config/task_calibration.40_runtime.json --startup-advance-ms 0
  --startup-speed 1 --scan-wz 1 --turn-wz 1)
common+=("${extra[@]}")
if (( ${#extra[@]} )); then echo "CONTACT TEST: forward/reverse/turn/jaw collision checks DISABLED; health checks and STOP retained."; fi
if [[ "$mode" == --check ]]; then exec "$binary" --check-config "${common[@]}"; fi
# All static checks run before anything opens a device or moves the pitch servo.
"$binary" --check-config "${common[@]}"
if [[ ! -c /dev/ttyACM0 ]]; then
  echo 'MCU serial device /dev/ttyACM0 is missing. Check MCU power/USB enumeration; /dev/ttyUSB0 is the IMU.' >&2
  exit 1
fi
mkdir -p telemetry_logs
run_log="telemetry_logs/capture-scan-$(date +%Y%m%d-%H%M%S)-$$.log"
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
Startup: no timed forward advance; immediately search at 1.0 rad/s.
Search uses all known colours as cues; stop near the cue and select cargo.
First complete delivery must be green; afterwards black has priority. Closed-jaw clearing may push blue at 0.40 m/s until the observed target reaches 0.20 m, with bounded travel and two attempts; requires verified clearance.
No global localization is used. Measured-translation watchdog is disabled in this
mode; IMU/serial/frame health, phase budgets and host STOP remain. Total match time limit is disabled for debugging.
TXT
python3 tools/camera_pitch/pitch_ctl.py prepare45 --timeout 5
if [[ "$mode" == --auto-run ]]; then
  stdbuf -oL -eL "$binary" --hardware --imu --imu-port /dev/ttyUSB0 --telemetry --auto-run "${common[@]}"
  exit $?
fi
stdbuf -oL -eL "$binary" --hardware --imu --imu-port /dev/ttyUSB0 --telemetry "${common[@]}"
