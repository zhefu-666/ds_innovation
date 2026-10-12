# 板上显示器实时监看

在板子桌面双击“机器人实时监看”，或执行：

```bash
python3 /home/cat/ds_innovation/tools/board_monitor/monitor.py --open
```

浏览器地址 `http://127.0.0.1:18080/`。只在板子本机监听，不需要网络访问。

- 左侧为比赛程序已有的带标注相机画面，右侧为最新 `match-main-*.log`。
- 可选状态事件或全部日志、暂停滚动、全屏。
- 新一趟自动切换日志。遥测断开/过期时清除实时状态并隐藏旧图；日志可作为历史查看。
- 必须正常启动比赛程序/既有遥测桥后才有实时画面。窗口本身不打开摄像头，不操作串口，不发送比赛命令。
- 不要为此额外运行 `view.py`：该脚本可能另起预览占用摄像头。
- 日志读取量和显示行数均有限制，避免长比赛日志拖慢窗口。
- 复用现有 `http://127.0.0.1:8080/status` 与 `/stream.mjpg`；已有比赛启动脚本负责遥测桥。

首次安装桌面入口：`python3 /home/cat/ds_innovation/tools/board_monitor/install_desktop.py`。
无需 sudo，不改比赛程序、不设置自动启动比赛。桌面环境需有 Chromium 或 Firefox；若无浏览器，安装步骤保留脚本与入口但无法显示窗口。

测试：`python3 /home/cat/ds_innovation/tools/board_monitor/test_monitor.py`。
