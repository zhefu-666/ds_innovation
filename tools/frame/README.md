# 方框工具

入口保留在 `frame_up.sh`、`frame_down.sh`、`frame_status.sh` 和 `frame_ctl.py`。up/down 会发送机构动作，即使轮速为0也不是只读操作。status和dry-run不发送控制字节，但会打开串口读取反馈。

默认串口 `/dev/ttyACM0`，115200；与主程序互斥占用。确认动作需要新鲜反馈、匹配动作编号及期望状态，不独立证明机械到位。

当前A6持续表示开闭状态，开为1、关为0。相机下发代码按用户要求保持现状。统一说明见 [通信协议](../../docs/通信协议.md)。虚拟串口测试为 `tests/test_frame_ctl.py`。
