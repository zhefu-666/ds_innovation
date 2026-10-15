#!/usr/bin/env bash
# 开机自启包装：等串口/相机就绪后，用与终端手动运行相同的参数启动整场主程序，输出写入 /tmp/match_run_stdout.log。
set -u
cd /home/cat/ds_innovation
exec > /tmp/match_run_stdout.log 2>&1 < /dev/null

for _ in $(seq 1 60); do
  [[ -c /dev/ttyACM0 && -c /dev/ttyUSB0 && -e /dev/video0 ]] && break
  sleep 1
done
sleep 5

exec tools/match/match_main.sh --run --site-clear --frame open --clear-neighbours --startup-advance-ms 12000 --startup-speed .4
