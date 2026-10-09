# 方框升降工具

在鲁班猫 `/home/cat/ds_innovation` 下运行：

```bash
bash tools/frame/frame_up.sh        # 上升：+20度，实际发送
bash tools/frame/frame_down.sh      # 下降：0度，实际发送
bash tools/frame/frame_status.sh    # 只读当前A6反馈
bash tools/frame/frame_up.sh --dry-run   # 只读反馈并预览上升帧，不发送
bash tools/frame/frame_down.sh --dry-run # 只读反馈并预览下降帧，不发送
```

默认串口 `/dev/ttyACM0`，115200。可加 `--port`、`--baud`、`--timeout 3`。

也可调用 `python3 tools/frame/frame_ctl.py up|down|status`；raise/lower为别名。
原 `tools/gripper/gripper_open.sh`、`gripper_close.sh`、`gripper_status.sh` 及 `gripper_ctl.py open|close|status` 保留兼容入口，实际转到方框工具。

升降时vx/wz均为0，相机保持首次新鲜A6返回的实际角度（线上范围0..80，80对应旧40度；方框工具发送RX-40的偏移角以保持当前位置）。每次显式up/down都使用当前编号+1（1..255循环）并实际发送；即使A6状态已等于目标也不跳过。同一次调用内以20Hz重发同一编号，不能把旧编号的相同状态当作本次确认。

反馈沿用目前二值约定：0放下、1抬起。成功表示CRC有效且动作编号、状态匹配，不等于独立机械到位检测。超时停止重发，不反向动作。未收到A6、相机角度无效、状态非0/1时拒绝发送。

status和dry-run以O_RDONLY打开串口，TX=0；退出恢复串口参数。使用flock和TIOCEXCL排他占用，与主程序共用串口时会报错。运行升降工具前先正常停止主程序，不同时启动其他串口控制工具。

验证：tests/test_frame_ctl.py 使用虚拟串口，不操作实车。

相机新协议：方框升降允许保持80度，不会把80度当无效或截断到40度。主程序TX保持旧标定角，RX减40后供旧标定使用。
