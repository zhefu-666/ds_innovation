#!/usr/bin/env bash
# 3 分钟主程序整场入口：不带 demo/assume 标志。默认 --check，不打开任何设备、不发任何控制字节。
set -euo pipefail
cd "$(dirname "$0")/../.."

usage() {
  cat >&2 <<'TXT'
用法：
  tools/match/match_main.sh --status
  tools/match/match_main.sh [--check] [--frame open|close|keep] [--known-frame-angle 0|20 --known-frame-id N]
  tools/match/match_main.sh --run --site-clear [--frame open|close|keep] [--known-frame-angle 0|20 --known-frame-id N]

--status            只读查询下位机A6反馈(完成位/动作编号/相机角)和本机记录的方框状态，TX=0，不动任何机构；
--clear-neighbours  给主程序加 --clear-neighbours（扫开目标走廊内的邻近物资，不搬运）；
--run 开始前自动做同样的下位机只读查询；--frame keep 时记录/给定的编号与下位机编号不一致则拒绝。
--frame open|close  开局固定先发反向动作、再发目标动作（open=先close再open；不看记录和下位机编号，不跳过），最终停在目标状态(开框0°/关框20°)，并记录已知方框状态；
--frame keep        不动方框，沿用上次 open/close 的记录，或用 --known-frame-* 显式给出（默认）。
--run 的顺序：静态检查 -> 方框 -> 相机回 UI 90°（内部 -30°） -> 再次静态检查 -> 启动主程序（180秒，--auto-run）。
TXT
  exit 2
}

mode=check
site_clear=0
frame=keep
cli_angle=
cli_id=
startup_ms=0
startup_speed=.10
clear_neighbours=0
while (( $# )); do
  case "$1" in
    --check) mode=check ;;
    --status) mode=status ;;
    --clear-neighbours) clear_neighbours=1 ;;
    --run) mode=run ;;
    --site-clear) site_clear=1 ;;
    --frame) [[ $# -ge 2 ]] || usage; frame="$2"; shift ;;
    --known-frame-angle) [[ $# -ge 2 ]] || usage; cli_angle="$2"; shift ;;
    --known-frame-id) [[ $# -ge 2 ]] || usage; cli_id="$2"; shift ;;
    --startup-advance-ms) [[ $# -ge 2 ]] || usage; startup_ms="$2"; shift ;;
    --startup-speed) [[ $# -ge 2 ]] || usage; startup_speed="$2"; shift ;;
    *) usage ;;
  esac
  shift
done
[[ "$frame" == open || "$frame" == close || "$frame" == keep ]] || usage
if [[ -n "$cli_angle$cli_id" ]]; then
  [[ "$cli_angle" == 0 || "$cli_angle" == 20 ]] || { echo '--known-frame-angle 必须是 0 或 20' >&2; exit 2; }
  [[ "$cli_id" =~ ^[0-9]+$ ]] && (( cli_id >= 1 && cli_id <= 255 )) || { echo '--known-frame-id 必须是 1..255' >&2; exit 2; }
  [[ "$frame" == keep ]] || { echo '--frame open|close 会自己记录方框状态，不能同时给 --known-frame-*' >&2; exit 2; }
fi
[[ "$startup_ms" =~ ^[0-9]+$ ]] && (( startup_ms <= 60000 )) || { echo '--startup-advance-ms 必须是 0..60000 的整数' >&2; exit 2; }
[[ "$startup_speed" =~ ^[0-9]*\.?[0-9]+$ ]] && awk -v v="$startup_speed" 'BEGIN{exit !(v>0 && v<=1)}' || { echo '--startup-speed 必须在 (0,1] m/s' >&2; exit 2; }
if [[ "$mode" == run && $site_clear -ne 1 ]]; then
  echo '--run 须显式带 --site-clear：确认机构/前后/旋转扫掠空间已清空，场内只有己方区域、无人员和对手。' >&2
  exit 2
fi

binary=${RESCUE_BINARY:-$PWD/build-release/rescue_upper_host}
binary_abs=$binary
[[ "$binary_abs" == /* ]] || binary_abs="$PWD/$binary"

# 任何设备动作之前打印运行身份，使 --check/--status 也能确认实际会被执行的二进制。
print_identity() {
  if [[ -x "$binary_abs" ]]; then
    echo "[identity] binary=$binary_abs md5=$(md5sum "$binary_abs" | cut -d' ' -f1) mtime=$(date -Is -r "$binary_abs")"
  else
    local cands
    cands=$(cd "$PWD" && ls -d build-*/ 2>/dev/null | tr '\n' ' ')
    echo "[identity] binary=$binary_abs 不存在；仓库内构建目录：${cands:-无}" >&2
  fi
}
print_identity
# R1: CPUs 0..3 are A55; CPUs 4..7 are A76. Scope affinity to this child only.
cpu_set=${RESCUE_CPUSET:-4-7}
cpu_launcher=()
if [[ "$cpu_set" != all ]]; then
  command -v taskset >/dev/null || { echo '缺少 taskset，无法应用 CPU 核配置' >&2; exit 1; }
  taskset -c "$cpu_set" true || { echo "CPU 核配置不可用：$cpu_set" >&2; exit 1; }
  cpu_launcher=(taskset -c "$cpu_set")
fi
demo_profile="$PWD/config/project.json#/profiles/demo_assume"
known_file="/tmp/rescue-demo-$(id -u)/known_frame.json"
placeholder=0
frame_done=0

resolve_known() {
  known_angle=; known_id=
  if [[ -n "$cli_angle" ]]; then known_angle=$cli_angle; known_id=$cli_id; known_src=cli; return; fi
  local rec
  rec=$(python3 - "$known_file" <<'PY' 2>/dev/null || true
import json, sys
d = json.load(open(sys.argv[1]))
if d.get('angle') in (0, 20) and type(d.get('id')) is int and 1 <= d['id'] <= 255:
    print(d['angle'], d['id'])
PY
)
  if [[ -n "$rec" ]]; then read -r known_angle known_id <<<"$rec"; known_src="recorded $known_file"; return; fi
  known_src=none
}

build_args() {
  args=(--config config/project.json
    --team blue --no-show --parallel-infer --async-capture
    --controlled-empty-field --match-seconds 180
    --startup-advance-ms "$startup_ms" --startup-speed "$startup_speed" --scan-wz 1.6 --turn-wz .25
    --telemetry --telemetry-fps 30
    --hardware --imu --imu-port /dev/ttyUSB0 --imu-baud 115200 --port /dev/ttyACM0 --baud 115200
    --known-frame-angle "$known_angle" --known-frame-id "$known_id")
  (( clear_neighbours )) && args+=(--clear-neighbours)
  return 0
}

static_check() {
  [[ -x "$binary_abs" ]] || {
    echo "缺少主程序：$binary_abs" >&2
    echo "用 RESCUE_BINARY=/path/to/rescue_upper_host 显式指定，或先在仓库内构建。" >&2
    return 1
  }
  resolve_known
  if [[ "$known_src" == none ]]; then
    if [[ "$mode" != check && ( "$frame" == keep || $frame_done -eq 1 ) ]]; then
      echo '未知方框当前状态：用 --frame open|close，或给 --known-frame-angle/--known-frame-id。' >&2
      return 1
    fi
    known_angle=0; known_id=1; placeholder=1
    echo '[check] 方框状态将由 --frame 动作写入，现在 --known-frame-* 用占位值 0/1，仅用于配置检查，不会发给设备。'
  fi
  build_args
  echo "[check] 方框已知状态：angle=$known_angle id=$known_id ($known_src)"
  echo "[check] $binary_abs --check-config ${args[*]}"
  "$binary_abs" --check-config "${args[@]}"
}

live_id=
lower_state() { # 只读查询下位机A6，设置 live_id
  local out
  out=$(python3 tools/match/query_lower.py 2>&1) || { printf '%s\n' "$out" >&2; return 1; }
  printf '%s\n' "$out"
  live_id=$(sed -n 's/^LOWER_STATE .* id=\([0-9]*\) .*/\1/p' <<<"$out")
  [[ -n "$live_id" ]]
}

demo_tool() { # 方框/相机动作沿用演示入口，复用 A6 编号记录和验收门槛
  local tool="$1"; shift
  RESCUE_DEMO_PROFILE="$demo_profile" "tools/demo/$tool" "$@"
}

if [[ "$mode" == status ]]; then
  lower_state
  resolve_known
  if [[ "$known_src" == none ]]; then
    echo "[本机记录] 没有方框状态记录（$known_file 不存在或无效）"
  else
    echo "[本机记录] 方框angle=$known_angle id=$known_id ($known_src)"
    if [[ "$known_id" == "$live_id" ]]; then echo '[对比] 记录编号与下位机一致'
    else echo "[对比] 记录编号 $known_id 与下位机编号 $live_id 不一致：记录已过期，--frame keep 会被拒绝"; fi
  fi
  if [[ "$live_id" == 0 ]]; then
    echo '[建议] 下位机编号为0，--known-frame-id 不接受0；用 --frame open|close（会动框，需先清空框周围）让下位机记下一个编号'
  else
    echo "[建议] 现场确认方框实际角度后：--frame open|close（会动框），或 --known-frame-angle 0|20 --known-frame-id $live_id（不动框）"
  fi
  exit 0
fi

if [[ "$mode" == check ]]; then
  static_check
  case "$frame" in
    open|close) demo_tool "frame_$frame.sh" ;;
  esac
  cam_known=()
  [[ -n "$known_angle" ]] && cam_known=(--known-frame-angle "$known_angle" --known-frame-id "$known_id")
  demo_tool camera.sh --offset-deg -30 "${cam_known[@]}"
  echo '[check] 完成：未打开设备。'
  exit 0
fi

# 第一次检查先于任何设备动作；下位机查询是只读的（TX=0）。
lower_state || { echo '无法读取下位机状态，停止启动。' >&2; exit 1; }
static_check
if [[ "$frame" == keep && "$known_id" != "$live_id" ]]; then
  echo "方框已知编号 $known_id ($known_src) 与下位机当前编号 $live_id 不一致，停止启动：先用 --frame open|close，或按现场实际给出 --known-frame-*。" >&2
  exit 1
fi
[[ -c /dev/ttyACM0 ]] || { echo 'MCU 串口 /dev/ttyACM0 不存在；检查 MCU 供电/USB。' >&2; exit 1; }
[[ -c /dev/ttyUSB0 ]] || { echo 'IMU 串口 /dev/ttyUSB0 不存在。' >&2; exit 1; }

mkdir -p telemetry_logs
run_log="telemetry_logs/match-main-$(date +%Y%m%d-%H%M%S)-$$.log"
echo "运行日志：$PWD/$run_log"
exec > >(tee -a "$run_log") 2>&1
echo "RUN_HEADER time=$(date -Is) host=$(hostname) git=$(git rev-parse --short HEAD 2>/dev/null || echo none) frame=$frame startup_ms=$startup_ms startup_speed=$startup_speed"
echo "RUN_HEADER binary=$binary_abs md5=$(md5sum "$binary_abs" | cut -d' ' -f1) mtime=$(date -Is -r "$binary_abs") script_md5=$(md5sum "$0" | cut -d' ' -f1)"

case "$frame" in
  open|close)
    # A6 不含方框位置；下位机对“与当前相同”的目标不回传，其内部状态与物理位置可能不一致。
    # 因此无论编号和记录如何，都先发反向动作让下位机状态翻转，再发目标动作，保证开局结束在目标状态。
    opp=close; [[ "$frame" == close ]] && opp=open
    resolve_known
    [[ "$known_src" == none ]] || echo "[方框] 记录 angle=$known_angle id=$known_id，下位机 id=$live_id（仅供参考，不据此跳过）。"
    echo "[方框] 先 ${opp}，再 ${frame}（开局固定反向一次，最终为 ${frame}）。"
    demo_tool "frame_$opp.sh" --run --site-clear || echo "[方框] ${opp} 无回传(下位机可能认为已在该状态)，继续执行 ${frame}。"
    demo_tool "frame_$frame.sh" --run --site-clear ;;
esac
frame_done=1
cam_known=()
[[ -n "$cli_angle" ]] && cam_known=(--known-frame-angle "$cli_angle" --known-frame-id "$cli_id")
demo_tool camera.sh --offset-deg -30 --run --site-clear "${cam_known[@]}"

# 演示入口自己持有同一把启动锁，所以方框/相机动作结束后才能取锁。
exec 9>/tmp/rescue-match-launch.lock
flock -n 9 || { echo '另一个比赛/演示入口正在运行。' >&2; exit 1; }
python3 tools/match/check_socket.py

# 方框动作后状态记录已更新，用真实值再检查一次再启动。
static_check
args+=(--auto-run)

source tools/match/telemetry_bridge.sh
cat <<'TXT'
整场主程序：180 秒（超过 10 分钟自动停止），--auto-run 在实时预检通过后自动发 START；无 demo/assume 标志。
统一配置中的 frame.assume 与机械俯仰模型均为临时假设，结果不构成验收。
Ctrl+C 或 match_ctl.py 可停止；零速发送以现场急停为准。
TXT
# 主程序整场只需 180 秒；超过 MATCH_MAX_RUN_S（默认 600 秒）仍未退出则发 SIGTERM（主程序会收尾停车），10 秒后仍在则 SIGKILL。
max_run_s=${MATCH_MAX_RUN_S:-600}
set +e
start_s=$SECONDS
echo "[CPU] affinity=$cpu_set; OpenCV threads=${RESCUE_OPENCV_THREADS:-1}"
timeout --signal=TERM --kill-after=10 "$max_run_s" "${cpu_launcher[@]}" stdbuf -oL -eL "$binary_abs" "${args[@]}"
rc=$?
elapsed_s=$((SECONDS - start_s))
if [[ $rc -eq 124 || $rc -eq 137 ]]; then
  echo "MATCH_MAIN_TIMEOUT: 主程序运行 ${elapsed_s}s 超过上限 ${max_run_s}s，已被强制停止(rc=$rc)。"
fi
echo "MATCH_MAIN_ELAPSED_S: $elapsed_s"
echo "MATCH_MAIN_EXIT_CODE: $rc"
exit "$rc"
