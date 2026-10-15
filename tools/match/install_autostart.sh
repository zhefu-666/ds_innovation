#!/usr/bin/env bash
# 在板子上安装/卸载开机自启。只注册，不立即启动（立即启动会动方框和底盘）。
set -euo pipefail
cd "$(dirname "$0")"
if [[ "${1:-}" == --uninstall ]]; then
  sudo systemctl disable --now match-main.service 2>/dev/null || true
  sudo rm -f /etc/systemd/system/match-main.service
  sudo systemctl daemon-reload
  echo '已卸载 match-main.service'
  exit 0
fi
chmod +x match_autostart.sh match_main.sh
sudo install -m 644 match-main.service /etc/systemd/system/match-main.service
sudo systemctl daemon-reload
sudo systemctl enable match-main.service
echo '已启用：下次开机自动运行。查看：systemctl status match-main.service ；日志：/tmp/match_run_stdout.log'
echo '取消：tools/match/install_autostart.sh --uninstall'
