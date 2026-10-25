#!/usr/bin/env bash
# 开机自启包装：systemd 启动本脚本。当前调试阶段禁止开机自动操作硬件，脚本直接退出，不做任何设备动作。
#
# 恢复开机自启时需要：
#   1. 删除下面的 exit 0 保护；
#   2. 取消 RECOVERY_COMMAND 那行注释并改为实际执行。
# 恢复命令记录（2026-10-15 禁用前的原始整场参数，未在本轮验证）：
#   tools/match/match_main.sh --run --site-clear --frame open --clear-neighbours \
#     --startup-advance-ms 12000 --startup-speed .4
#
# 原始脚本还会先等待 /dev/ttyACM0、/dev/ttyUSB0、/dev/video0 就绪再睡眠 5 秒；
# 恢复时需一并还原，不能只取消命令行注释。
set -u

# 调试阶段禁止开机自动操作硬件（2026-10-15）。
echo Match autostart disabled for debugging.
exit 0
