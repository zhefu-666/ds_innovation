#!/usr/bin/env bash
# 一键 demo：关框 -> 开框 -> 相机回 5° -> 网页桥 -> 抓拍 -> 搬运演示（--assume-all-safe；--target injured 时加 --assume-injured-trip）。
# 默认只检查，不打开任何设备、不发任何控制字节；--run 需同时带 --site-clear。
set -euo pipefail
base="$(cd -- "$(dirname -- "$0")" && pwd)"
cd "$base"

usage() {
  cat >&2 <<'TXT'
用法：
  demo_oneclick.sh [--check]                       只检查文件/端口/编号，不动设备
  demo_oneclick.sh --run --site-clear [选项]       完整流程（框子会动）
选项：
  --target T         目标类型：injured（伤员，右半区，默认）或 supply（普通物资，左半区）；一次只搬一趟
  --seconds N        演示时长，1..60（默认 60）
  --run-no N         指定日志编号（默认 logs/ 下最大编号 +1）
  --no-frame-cycle   跳过 关框->开框（框子状态已知时用；仍会做相机回 5°）
TXT
  exit 2
}

mode=check; site_clear=0; seconds=60; run_no=; frame_cycle=1; target=injured
while (( $# )); do
  case "$1" in
    --check) mode=check ;;
    --run) mode=run ;;
    --site-clear) site_clear=1 ;;
    --seconds) [[ $# -ge 2 ]] || usage; seconds="$2"; shift ;;
    --run-no) [[ $# -ge 2 ]] || usage; run_no="$2"; shift ;;
    --target) [[ $# -ge 2 ]] || usage; target="$2"; shift ;;
    --no-frame-cycle) frame_cycle=0 ;;
    *) usage ;;
  esac
  shift
done
case "$target" in
  injured) trip_flag=(--assume-injured-trip) ;;
  supply)  trip_flag=() ;;
  *) echo '--target 只能是 injured 或 supply' >&2; exit 2 ;;
esac
[[ "$seconds" =~ ^[0-9]+$ ]] && (( seconds >= 1 && seconds <= 60 )) || { echo '--seconds 必须是 1..60 的整数' >&2; exit 2; }
if [[ "$mode" == run && $site_clear -ne 1 ]]; then
  echo '--run 须显式带 --site-clear：确认机构/前后/旋转扫掠空间已清空，场内只有己方区域、无人员和对手。' >&2
  exit 2
fi

export RESCUE_DEMO_PROFILE="$base/runtime/config/demo_20261008.assume.json"
[[ -f "$RESCUE_DEMO_PROFILE" ]] || { echo "缺少 profile: $RESCUE_DEMO_PROFILE" >&2; exit 1; }
[[ -x "$base/runtime/bin/rescue_upper_host" ]] || { echo '缺少可执行的 runtime/bin/rescue_upper_host' >&2; exit 1; }
for f in view_stop.sh frame_close.sh frame_open.sh camera.sh web.sh carry_once.sh stop.sh logs/grab_stream.py; do
  [[ -e "$base/$f" ]] || { echo "缺少 $f" >&2; exit 1; }
done

if [[ -z "$run_no" ]]; then
  last=$(ls -d logs/run[0-9]* 2>/dev/null | sed 's#logs/run##' | sort -n | tail -1 || true)
  run_no=$(( ${last:-0} + 1 ))
fi
[[ "$run_no" =~ ^[0-9]+$ ]] || { echo '--run-no 必须是整数' >&2; exit 2; }
run_dir="logs/run${run_no}"
[[ ! -e "$run_dir/carry.txt" ]] || { echo "$run_dir/carry.txt 已存在，换个 --run-no" >&2; exit 2; }

ports_up() { ss -ltn 2>/dev/null | grep -q ':8080 ' && ss -ltn 2>/dev/null | grep -q ':18765 '; }

echo "profile : $RESCUE_DEMO_PROFILE"
echo "binary  : $(md5sum runtime/bin/rescue_upper_host | cut -c1-12)  runtime/bin/rescue_upper_host"
echo "目标    : $target"
echo "日志目录: $run_dir   时长: ${seconds}s   关开框: $([[ $frame_cycle -eq 1 ]] && echo 是 || echo 否)"
echo "网页桥  : $(ports_up && echo 已在运行 || echo 未运行，--run 时自动启动)"

if [[ "$mode" == check ]]; then
  echo '[CHECK] 仅检查，未动设备。各动作的静态检查：'
  for s in "frame_close.sh" "frame_open.sh" "camera.sh --offset-deg 5"; do
    echo "--- $s"; timeout 40 ./$s --site-clear 2>&1 | tail -4 || true
  done
  echo '--- carry_once.sh（未做 frame_open/close 时报 BLOCKED: done_flag 属正常）'
  timeout 40 ./carry_once.sh --site-clear --assume-all-safe ${trip_flag[@]+"${trip_flag[@]}"} --seconds "$seconds" 2>&1 | tail -4 || true
  exit 0
fi

mkdir -p "$run_dir/grab"
grab_pid=
cleanup() {
  [[ -n "$grab_pid" ]] && kill "$grab_pid" 2>/dev/null || true
}
on_signal() {
  echo; echo '[ONECLICK] 收到中断，停车'
  timeout 20 ./stop.sh 2>&1 | tail -3 || true
  cleanup
  exit 130
}
trap cleanup EXIT
trap on_signal INT TERM

step() { echo; echo "[ONECLICK] $*"; }
fail() { echo "[ONECLICK] 失败: $*" >&2; exit 1; }

step '1/6 关闭旧查看会话'
timeout 20 ./view_stop.sh || true

if (( frame_cycle )); then
  step '2/6 关框 -> 开框（记录已知框状态）'
  if ! timeout 40 ./frame_close.sh --run --site-clear; then
    # 框已在关闭位时再发 close 没有新反馈会超时；先 open 再 close 即可（与手动处理一致）
    echo '[ONECLICK] 关框无反馈，按"已在关闭位"处理：先开框再关框'
    timeout 40 ./frame_open.sh  --run --site-clear || fail 'frame_open(重试)'
    timeout 40 ./frame_close.sh --run --site-clear || fail 'frame_close'
  fi
  timeout 40 ./frame_open.sh  --run --site-clear || fail 'frame_open'
else
  step '2/6 跳过关开框（沿用上次记录的框状态）'
fi

step '3/6 相机回 5°（必须）'
timeout 40 ./camera.sh --run --site-clear --offset-deg 5 || fail 'camera'

step '4/6 网页桥 8080 / 18765'
if ! ports_up; then
  nohup bash web.sh --run --site-clear > "$run_dir/web.txt" 2>&1 &
  for _ in $(seq 1 30); do ports_up && break; sleep 0.5; done
fi
ports_up || fail "网页桥未起来，见 $run_dir/web.txt"

step "5/6 启动抓拍（$((seconds + 15))s，每 0.5s 一张）"
nohup python3 logs/grab_stream.py "$run_dir/grab" "$((seconds + 15))" 0.5 > "$run_dir/grab.log" 2>&1 &
grab_pid=$!

step "6/6 搬运演示 ${seconds}s（Ctrl-C 会先停车）"
set +e
bash carry_once.sh --run --site-clear --assume-all-safe ${trip_flag[@]+"${trip_flag[@]}"} --seconds "$seconds" 2>&1 | tee "$run_dir/carry.txt"
rc=${PIPESTATUS[0]}
set -e

echo; echo "[ONECLICK] 结束 rc=$rc  日志: $base/$run_dir/carry.txt  抓拍: $base/$run_dir/grab ($(ls "$run_dir/grab" | wc -l) 张)"
exit "$rc"
