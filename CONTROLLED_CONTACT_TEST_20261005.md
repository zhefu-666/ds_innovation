# 受控碰撞测试开关

用户明确要求取消前后路径碰撞检查。默认关闭，仅允许与 --controlled-empty-field 一起使用 --controlled-ignore-clearance。

开关放行前进、后退、原地转向、弧线转向及张爪的路径检查，包括缺少映射或已检测到蓝色障碍的情况；这些许可不代表真实安全证据。界面与运行日志明确标记检查已关闭。

仍保留 IMU/通信/帧健康、STOP、夹爪动作反馈、目标几何、货物类别/走廊规则、安全区身份与交付验证、单次重搜时间/转角限制。不是取消所有任务约束。

已移除累计丢目标重搜次数上限，以及 MID_APPROACH 闭爪路径受阻两次触发 no_clear_cargo_path 的分支。受阻时重新识别目标，不盲目移动。

仅静态检查（不打开设备）：
```bash
bash tools/match/start_capture_scan.sh --check --controlled-ignore-clearance
```

人工启动并等待比赛开始指令：
```bash
bash tools/match/start_capture_scan.sh --run --controlled-ignore-clearance
```

当前运行进程不会自动加载新程序。此次修改不自动重启、不复位、不发送运动指令。
