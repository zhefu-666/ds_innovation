# 工具目录

`tools/` 是仓库内脚本工具的唯一入口，按使用场景分目录：

| 目录 | 职责 |
| --- | --- |
| `tools/demo/` | 独立演示、只读预览、受控点动和演示查看入口；实现集中在 `demo.py` |
| `tools/match/` | 正式比赛启动、控制和遥测入口 |
| `tools/frame/`、`tools/gripper/`、`tools/camera_pitch/` | 框、夹爪和相机俯仰的协议工具 |
| `tools/camera_calibration/` | 相机内参、外参和地面映射标定 |
| `tools/remote_camera_telemetry/`、`tools/board_monitor/` | 遥测桥接和板上监看 |
| `tools/runtime/` | 固定运行基线记录与比较器 |

`src/tools/` 仍保留 C++ 监视器源码；它由 CMake 构建，不与本目录的脚本工具重复。

所有演示命令使用 `tools/demo/...`。旧的 `tool/demo/...` 目录已移除。
