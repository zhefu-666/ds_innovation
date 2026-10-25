#!/usr/bin/env bash
set -euo pipefail
cat <<'HELP'
在 /home/cat/ds_innovation 下运行：

方框：
  tools/demo/frame_open.sh                         # 0°抬起/开框，默认检查
  tools/demo/frame_close.sh                        # 20°放下/关框，默认检查
  tools/demo/frame_status.sh --run                 # 只读A6
移动：
  tools/demo/move_forward.sh --speed 0.05           # 无--seconds即持续，Ctrl+C停止
  tools/demo/move_backward.sh --speed 0.05
  tools/demo/turn_left.sh --wz 0.20
  tools/demo/turn_right.sh --wz 0.20
相机：
  tools/demo/camera.sh --offset-deg 5               # RX45°
  tools/demo/camera.sh --offset-deg 20              # RX60°
  tools/demo/camera.sh --offset-deg 40              # RX80°
查看与停止：
  tools/demo/view.sh                               # 只读图像和数据，最多1小时
  tools/demo/view_stop.sh                          # 结束整套查看
  tools/demo/connect.sh                            # 电脑端SSH隧道
  tools/demo/status.sh
  tools/demo/stop.sh                               # 停止动作入口

方框、移动、相机执行时加 --run --site-clear；先结束占用串口的view。
默认配置仍有验收缺项，缺项会拒绝动作，不要直接改为通过。
使用自己的配置：--profile /绝对路径/demo.local.json。
完成位模式保持机构需本轮已知角度与编号：
  --known-frame-angle 0 --known-frame-id <实际编号>
不要照抄历史编号。通用点动上限0.10m/s；历史0.4测试不是通用入口。
HELP
