# USB 相机远程查看

这是独立的 Python 相机工具，使用现有 OpenCV 4.6.0 和 websockets 10.4，提供 Foxglove WebSocket v1 兼容协议及浏览器 MJPEG 预览。它不使用 C++ Foxglove SDK，不连接机器人控制程序。原有 `foxglove_telemetry` 示例未修改。

## 现在查看

- 浏览器：<http://192.168.1.123:8080/>，显示实时图像和相机状态。
- Lichtblick 桌面版：添加 Foxglove WebSocket 连接 `ws://192.168.1.123:8765`。
- Image 面板选择 `/camera/image`；Raw Messages 面板选择 `/system/health`。
- Plot 可选择 `/system/health.capture_fps` 或 `/system/health.frame_age_s`。

当前只发布相机图像和采集状态；检测结果、IMU、状态机尚未接入。服务面向可互通的局域网，HTTP 和 WebSocket 均未配置身份认证/TLS。

## 板子上启动

程序部署在 `/home/cat/camera-telemetry/server.py`，默认按稳定设备路径打开相机，避免插拔后 `/dev/videoN` 编号变化。

首次部署后已启动后台进程。不要重复启动；8080 或 8765 被占用会启动失败。

停止当前后台进程（在板子上运行）：

```bash
kill "$(cat /home/cat/camera-telemetry/server.pid)"
```

重启或板子重启后启动（在板子上运行）：

```bash
nohup python3 /home/cat/camera-telemetry/server.py \
  > /home/cat/camera-telemetry/server.log 2>&1 < /dev/null &
echo $! > /home/cat/camera-telemetry/server.pid
```

前台调试（先停止后台服务，Ctrl+C 退出）：

```bash
python3 /home/cat/camera-telemetry/server.py --device /dev/video0
```

查看日志：

```bash
tail -n 50 /home/cat/camera-telemetry/server.log
```

未配置开机自启。不要同时运行其他占用同一相机的程序。以后接入机器人主程序时，应复用其采集帧，避免再次打开相机。

## 行为

- 请求 MJPG 640×480、30 FPS 采集；JPEG 质量 70、发布上限 10 FPS。
- 独立采集线程，仅保留最新 JPEG；慢客户端不会让采集线程等待网络。
- 摄像头读取失败会重试；页面状态显示失败或帧过期。
- WebSocket capabilities 为空，只接受订阅/退订，不支持发布、参数修改或控制。
- 浏览器显示原始相机方向，不自动旋转。

## 验证

本地 Node 22+：

```bash
node /home/liu/ds_innovation/tools/remote_camera_telemetry/verify.mjs
```

验证 HTTP 状态与 JPEG、Foxglove 握手/通道/时间戳/20 帧图像、健康消息、重连和拒绝参数修改。
测试图片写入本地 `/tmp/remote-camera.jpg` 与 `/tmp/remote-camera-foxglove.jpg`。

2026-09-17 实测：本地浏览器已显示真实画面；HTTP 与 Foxglove 图像均解码为 640×480 RGB；Foxglove 在 2.421 秒内收到 20 帧图像及 23 条健康消息；相机采集约 30 FPS。未在 Lichtblick GUI 内验收，仅进行了 Foxglove 协议客户端验证。

## 新增文件

本地工作空间：

- `tools/remote_camera_telemetry/server.py`：独立服务源代码，预览网页内嵌。
- `tools/remote_camera_telemetry/verify.mjs`：本地端到端验证程序。
- `tools/remote_camera_telemetry/README.md`：本说明。

板子（原 `gongchuang` 项目目录之外）：

- `/home/cat/camera-telemetry/server.py`
- `/home/cat/camera-telemetry/README.md`
- `/home/cat/camera-telemetry/server.log`：运行日志。
- `/home/cat/camera-telemetry/server.pid`：后台进程编号。

本次没有安装新依赖，没有修改原机器人业务代码或创建系统服务。
