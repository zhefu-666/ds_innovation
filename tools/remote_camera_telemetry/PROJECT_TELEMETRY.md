# ds_innovation 主程序实时查看

本说明对应 2026-09-26 新增的主程序遥测。板子运行 `/home/cat/ds_innovation/build-arm64-telemetry/rescue_upper_host`，电脑使用 Foxglove 或浏览器查看。当前仍为 `--dry-run` 检测预览，不启用底盘运动。

## 查看

- Foxglove：Open connection → Foxglove WebSocket → `ws://192.168.1.123:8765`。
- Image 面板：`/camera/image`，画面已经绘制检测框和类别，无需另设叠加层。
- Raw Messages：`/detections`、`/fsm/state`、`/runtime/config`。
- Plot：`/system/health.loop_fps`、`/system/health.inference_ms`、`/system/health.frame_age_s`。
- 浏览器：`http://192.168.1.123:8080/`，显示图像、检测列表、状态和运行参数。

`/cmd/motion` 是任务计算值，`hardware_output_enabled=0`，不是已发送命令或实际车速。IMU 和 ToF 未接入，健康状态明确为 false。`/fsm/state` 的 WAIT_START 是默认未开启任务运行请求的真实状态；不会为展示而伪造状态变化。

## 启动、停止和日志（在板子执行）

```bash
cd /home/cat/ds_innovation
python3 tools/remote_camera_telemetry/manage_project.py start
python3 tools/remote_camera_telemetry/manage_project.py status
```

启动命令后台运行主程序和查看服务，确认收到新图像才报告成功。退出 SSH 后仍运行，未配置开机自启。重复启动会报错，不会打开第二个摄像头。

停止：

```bash
cd /home/cat/ds_innovation
python3 tools/remote_camera_telemetry/manage_project.py stop
```

修改检测阈值后启动，例如：

```bash
python3 tools/remote_camera_telemetry/manage_project.py start --conf 0.35
```

其他主程序参数先查看 `./build-arm64-telemetry/rescue_upper_host --help`。`requested_fps` 是请求的采集帧率，`loop_fps` 是包含推理的实际处理帧率，两者不能混用。`inference_ms` 包含检测和跟踪，`capture_ms` 是读取图像的等待时间。运行配置由命令行提供，不读取 `config/rescue.yaml`。

日志：

```bash
tail -f /home/cat/ds_innovation/telemetry_logs/main.log
tail -f /home/cat/ds_innovation/telemetry_logs/bridge.log
```

每次启动保存新的时间戳日志，`main.log` 和 `bridge.log` 是最近一次启动的链接。

旧的 `/home/cat/camera-telemetry/server.py` 必须先停止：它会占用相机、8080 和 8765。查看服务 `project_bridge.py` 自身不打开摄像头。不要同时启动其他相机采集程序。

## 独立运行

```bash
cd /home/cat/ds_innovation
python3 tools/remote_camera_telemetry/project_bridge.py
```

另一个远程终端：

```bash
cd /home/cat/ds_innovation
./build-arm64-telemetry/rescue_upper_host --dry-run --no-show --telemetry
```

两个进程都在板子上运行。任何一方退出都不会给底盘发送动作。主程序停止或超过 2 秒未更新时，网页隐藏旧画面，健康通道 `robot_data_connected=false`；Foxglove Image 可能保留最后一帧，应同时关注健康通道。

## 实现与边界

主程序只在采集/检测/PushTask 更新之后复制一个受帧率限制的数据快照。独立工作线程压缩 JPEG、序列化并原子替换 `/dev/shm/rescue-telemetry.bin`。内存文件始终只保留一个完整快照，不写入 eMMC。图像和结构化数据共享采集时间戳和帧序号。主程序相机后端改为 V4L2/MJPEG，修复本板子默认 GStreamer 在配置 720p/60 FPS 后读取失败的问题。宽度最多 640，保持原始宽高比，JPEG 质量 70；不改变视觉输入分辨率。

Python 服务复用板子已有 OpenCV 4.6 和 websockets 10.4，读取该快照并发布 Foxglove WebSocket v1，以及 HTTP/MJPEG。无需安装 Foxglove C++ SDK。网络不在主程序路径中；浏览器断开、慢客户端或桥接退出不会要求主程序等待网络。遥测缓冲只保留最新状态，因此不保证保留每次状态转移，不提供 MCAP 录制。

查看服务仅支持订阅和退订；能力列表为空，参数写入、客户端发布与其他操作会被拒绝。不接受运动控制。服务面向当前局域网，沿用原服务的无认证 HTTP/WebSocket 配置。

本实现取代旧规格中的 SDK 网络发布方式；遵循旁路、压缩、限帧率、可关闭原则。数据源以当前 `PushTask` 为准，不使用旧规格中的十四状态机，也不发布占位 IMU/ToF/位姿。

## 构建与验证

```bash
cmake -S . -B build-arm64-telemetry -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DRESCUE_ENABLE_TELEMETRY=ON
cmake --build build-arm64-telemetry -j4
ctest --test-dir build-arm64-telemetry --output-on-failure
python3 tools/remote_camera_telemetry/test_project_bridge.py
```

编译选项默认 OFF；未编入遥测的二进制仍支持原有功能，传入 `--telemetry` 会明确报错。只运行 `--dry-run` 时不会创建遥测线程或快照。

修改前备份：`/home/cat/ds_telemetry_backup_20260926.tar.gz`。旧构建 `build-arm64-protocol-20260925` 未覆盖。需要恢复原始相机预览时，先停止项目查看，再启动 `/home/cat/camera-telemetry/server.py`。
