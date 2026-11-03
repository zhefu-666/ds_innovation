#!/usr/bin/env bash
# 安装 cat 用户的开机按键监听服务；不依赖图形登录，需 Linger=yes。
set -euo pipefail
cd "$(dirname "$0")"
unit=rescue-start-listener.service
target="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user/$unit"
case "${1:-}" in
  --uninstall)
    systemctl --user disable --now "$unit"
    rm -f "$target"
    systemctl --user daemon-reload
    echo "已卸载 $unit"
    exit 0
    ;;
  "") ;;
  *) echo "用法：$0 [--uninstall]" >&2; exit 2 ;;
esac
if [[ "$(loginctl show-user "$(id -u)" -p Linger --value)" != yes ]]; then
  echo "需先启用用户开机服务：loginctl enable-linger $(id -un)" >&2
  exit 1
fi
# 避免重装时重置已经运行的比赛进程。
if systemctl --user is-active --quiet "$unit"; then
  echo "$unit 已在运行；停止现有服务后再安装。" >&2
  exit 1
fi
install -Dm644 "$unit" "$target"
systemctl --user daemon-reload
systemctl --user enable --now "$unit"
echo "已启用 $unit：开机等待 CRC 正确的 A7 start=1。"
echo "查看日志：journalctl --user -u $unit -f"
