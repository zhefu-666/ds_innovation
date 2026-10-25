# 夹爪开合工具

在鲁班猫 `/home/cat/ds_innovation` 下运行：

```bash
bash tools/frame/frame_up.sh        # 兼容入口：打开夹爪
bash tools/frame/frame_down.sh      # 兼容入口：闭合夹爪
bash tools/frame/frame_status.sh    # 只读当前A6反馈
bash tools/frame/frame_up.sh --dry-run   # 只读反馈并预览上升帧，不发送
bash tools/frame/frame_down.sh --dry-run # 只读反馈并预览下降帧，不发送
```

默认串口 `/dev/ttyACM0`，115200。可加 `--port`、`--baud`、`--timeout 3`。

也可调用 `python3 tools/frame/frame_ctl.py open|close|status`；up/down/raise/lower 为兼容别名。
原 `tools/gripper/gripper_open.sh`、`gripper_close.sh`、`gripper_status.sh` 及 `gripper_ctl.py` 保留入口。

夹爪动作时 vx/wz 均为0，相机保持首次新鲜A6返回的绝对角（线上范围90..150，120为零点）。线上偏移9（从0计数，第10字节）为有符号 `gripper_offset`：+5（05）开、-5（FB）关、0为85°中位。A6反馈仍是0关/1开，发送与反馈不能混用。业务兼容值0开/20关由编码器转换，不直接上串口。目标状态不变时沿用当前动作编号，状态变化时按1..255循环分配新编号。同一次调用内以20Hz重发同一编号。

反馈沿用目前二值约定：0闭合、1张开。成功表示CRC有效且动作编号、状态匹配，不等于独立机械到位检测。超时停止重发，不反向动作。未收到A6、相机角度无效、状态非0/1时拒绝发送。

status和dry-run以O_RDONLY打开串口，TX=0；退出恢复串口参数。使用flock和TIOCEXCL排他占用，与主程序共用串口时会报错。运行升降工具前先正常停止主程序，不同时启动其他串口控制工具。

验证：tests/test_frame_ctl.py 使用虚拟串口，不操作实车。

相机新协议（2026-10-23 起）：A6 回传绝对角 `90..150°`，`120°` 为零点；TX 偏移限幅 `-30..+30°`。方框升降允许在 `90..150°` 内保持当前位置，不按旧 `0..80/40°` 截断。
