# ds_innovation 主程序实时查看

本说明对应 2026-09-26 新增的主程序遥测。板子运行 `/home/cat/ds_innovation/build-arm64-telemetry/rescue_upper_host`，电脑使用 Foxglove 或浏览器查看。当前仍为 `--dry-run` 检测预览，不启用底盘运动。

## 查看

- Foxglove：Open connection → Foxglove WebSocket → `ws://192.168.34.7:8765`。
- Image 面板：`/camera/image`，画面已经绘制检测框和类别，无需另设叠加层。
- Raw Messages：`/detections`、`/fsm/state`、`/decision/live`、`/runtime/config`。
- Plot：`/system/health.loop_fps`、`/system/health.inference_ms`、`/system/health.frame_age_s`。
- 浏览器：`http://192.168.34.7:8080/`，显示图像、检测列表、状态和运行参数。

安全区 pose 模型的 `zone_left`（类别 0）表示物资安全区半区，`zone_right`（类别 1）表示伤员安全区半区。左右以安全区入口面向区内的坐标系为准，不等于图像中的左右；模型类别不表示红蓝归属，仍需独立颜色证据。普通/核心物资投放左半区，伤员单独投放右半区。

`/decision/live` 同帧发布当前状态机阶段及原因、比赛/预检状态、候选目标、
区域身份与几何、安全许可、走廊和夹持证据、区内库存、规划落点以及计算指令。
`missing_evidence` 列出当前缺少的输入，不表示状态机会执行相应动作。
`valid=false` 表示主程序快照过期或尚未升级到提供决策数据的版本。
默认 `--dry-run` 下比赛状态保持 WAITING；不会为展示而伪造抓取流程。
蓝方单模型预览在两半区都被 pose 模型检测到、且未加载颜色标定时，
`zone_color_reason=blue_pose_model_assumption_preview_only` 表示按蓝方模型暂定蓝色。
这不是实测颜色，`zone_identity_verified` 仍为 false；`--hardware` 不采用该默认值。
在 Foxglove 的 Raw Messages 面板选择 `/decision/live`；Plot 可选择
`/decision/live.computed_vx_mps`、`/decision/live.target_distance_m`。
HTTP `http://192.168.34.7:8080/status` 也包含同一帧的 `decision` 字段。

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

蓝方模型与已有5°标定的只读预览（先停止正在占用相机的旧主程序，保留 bridge）：

```bash
./build-pitch40-telemetry-20261004/rescue_upper_host \
  --dry-run --no-show --telemetry --team blue \
  --imu --imu-port /dev/ttyUSB0 \
  --pitch-feedback --calibration config/camera.yaml \
  --model models/detect_fp.rknn --pose-model-blue models/zone_pose_fp.rknn \
  --task-calibration config/task_calibration.40_runtime.json \
  --startup-advance-ms 10000
```

`--pitch-feedback` 只读 A6，不下发 pitch 命令。当前 `camera.yaml` 仅验证 5°；
若实际读回为 0°，地面映射会以 `pitch_not_calibrated` 拒绝，不能靠默认蓝色绕过。
启动命令也不会把 `--startup-advance-ms` 变成已实测前进距离。

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
