# ds_innovation 下载后的部署与运行

本文说明从 GitHub 下载 `ds_innovation` 后，如何在 LubanCat/RK3588S 这类 ARM64 板子上完成编译，并通过网页或 Foxglove 查看主程序的实时检测画面。

当前主程序支持感知预览、离线回放和显式 `--hardware` 运动输出；网页管理脚本默认启动 `--dry-run` 预览。硬件路径使用 IMU、MCU 反馈、比赛许可和 MotionLink 门控，是否具备实车验收证据需要按实际模型、标定和运行策略分别确认。

当前架构见 [应用架构](docs/ARCHITECTURE.md)，新增 JSON 配置与最终参数导出见 [运行配置](docs/RUNTIME_CONFIGURATION.md)，本轮实施与验证边界见 [重构进度](docs/REFACTORING_PROGRESS.md)。旧命令仍使用原有参数覆盖顺序，命令行请求值不一定等于现场最终值。

新增参数、六点标注规范、无硬件回放和真实标定要求见 [阶段 0/1 实现说明](docs/history/STAGE01_IMPLEMENTATION.md)。

## 1. 获取代码

```bash
git clone https://github.com/zhefu-666/ds_innovation.git
cd ds_innovation
```

如果代码已下载：

```bash
cd ds_innovation
git pull --ff-only origin main
```

项目当前使用 `main` 分支。

## 2. 安装系统依赖

适用环境：Ubuntu 22.04/24.04、ARM64、OpenCV 4.x、CMake 3.10 或更高版本。

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config libopencv-dev nlohmann-json3-dev \
  v4l-utils python3 python3-venv
```

确认工具：

```bash
cmake --version
g++ --version
python3 --version
pkg-config --modversion opencv4
```

遥测查看服务需要 Python `websockets` 10.x。推荐使用项目自己的虚拟环境：

```bash
python3 -m venv .venv-telemetry
. .venv-telemetry/bin/activate
python -m pip install --upgrade pip
python -m pip install 'websockets>=10,<11'
```

以后启动查看服务前，都要先激活这个环境。如果系统已经提供 `websockets 10.4`，也可以直接使用系统 Python。

## 3. 补齐 Git 忽略的运行资源

为了避免把大文件和板卡二进制提交到 Git，`.gitignore` 排除了 `benchmark_results/`。完整运行仍需要模型与板卡运行库：

```text
benchmark_results/best_fp16.rknn
benchmark_results/librknnrt.so
```

请从模型制品、备份包或已有板子复制它们：

```bash
mkdir -p benchmark_results
# 示例：从另一台机器复制到当前板子
scp user@source-host:/path/best_fp16.rknn benchmark_results/
scp user@source-host:/path/librknnrt.so benchmark_results/
```

检查文件：

```bash
ls -lh benchmark_results/best_fp16.rknn benchmark_results/librknnrt.so
```

如果模型路径不同，运行时使用 `--model` 和 `--rknn-library` 指定。检测模型类别顺序必须与 `--classes` 一致；当前默认是 `blue,orange,green,black`，分别映射危险物、伤员、普通物资和核心物资。

启用 RKNN 编译还需要与运行库匹配的厂商 SDK 头文件 `rknn_api.h`。可放在 `benchmark_results/`，或通过 CMake 的 `-DRESCUE_RKNN_INCLUDE_DIR=/path/to/sdk/include` 指定。默认启用 RKNN；头文件缺失时配置会明确失败，不会使用猜测的 ABI。

## 4. 检查相机

插入 USB 相机后确认设备：

```bash
ls -l /dev/v4l/by-id/
v4l2-ctl --list-devices
```

当前默认相机是编号 `0`，程序使用 V4L2/MJPEG 读取 1280×720。若设备编号不同，启动时加 `--camera N`。

不要同时运行两个打开同一相机的程序。旧的独立相机服务和主程序只能二选一。

## 5. 编译

带主程序遥测的构建：

```bash
cmake -S . -B build-arm64-telemetry \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON \
  -DRESCUE_ENABLE_TELEMETRY=ON
cmake --build build-arm64-telemetry -j4
```

运行测试：

```bash
ctest --test-dir build-arm64-telemetry --output-on-failure
python3 tools/remote_camera_telemetry/test_project_bridge.py
```

如果只需要主程序、不需要 Foxglove 遥测，可以关闭遥测构建：

```bash
cmake -S . -B build-arm64-no-telemetry \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON \
  -DRESCUE_ENABLE_TELEMETRY=OFF
cmake --build build-arm64-no-telemetry -j4
ctest --test-dir build-arm64-no-telemetry --output-on-failure
```

`--telemetry` 只对启用了 `RESCUE_ENABLE_TELEMETRY=ON` 的二进制有效。

只运行离线测试或 CPU ONNX 推理、不使用 RKNN 时：

```bash
cmake -S . -B build-offline -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON -DRESCUE_ENABLE_RKNN=OFF -DRESCUE_ENABLE_TELEMETRY=ON
cmake --build build-offline -j4
ctest --test-dir build-offline --output-on-failure
./build-offline/rescue_upper_host --config config/runtime/push_replay.json
./build-offline/rescue_upper_host --config config/runtime/preview.json --dump-config /tmp/effective_config.json
```

`--dump-config -` 将纯 JSON 写入标准输出，诊断写入标准错误；导出后直接退出，不初始化模型、相机或串口。`--check-config` 另行校验静态标定，不能证明现场条件已满足。关闭 RKNN 的二进制不能加载 `.rknn` 模型。

## 6. 启动主程序和查看服务

推荐使用管理脚本。它会启动主程序和只读查看服务，并检查是否真的收到新图像：

```bash
. .venv-telemetry/bin/activate  # 如果使用项目虚拟环境
python3 tools/remote_camera_telemetry/manage_project.py start
```

默认启动内容：

```text
主程序：build-arm64-telemetry/rescue_upper_host --dry-run --no-show --telemetry
Foxglove：0.0.0.0:8765
网页：0.0.0.0:8080
```

查看运行状态：

```bash
python3 tools/remote_camera_telemetry/manage_project.py status
```

停止：

```bash
python3 tools/remote_camera_telemetry/manage_project.py stop
```

管理脚本不会配置开机自启。板子重启后需要重新执行 `start`。

## 7. 本机查看

确认板子 IP 后，在同一局域网的电脑上：

- 浏览器：`http://<板子IP>:8080/`
- Foxglove：选择 **Open connection → Foxglove WebSocket**，地址填 `ws://<板子IP>:8765`

Foxglove 通道：

安全区 pose 模型将 `zone_left`（类别 0）识别为物资安全区半区，`zone_right`（类别 1）识别为伤员安全区半区。左右按安全区入口面向区内的坐标系定义，不是图像中的左右排列；区域属于红方还是蓝方仍需独立颜色识别。角点顺序和几何映射见 [阶段3双模型说明](docs/history/STAGE3_TWO_MODELS.md)。

| 通道 | 内容 | 推荐面板 |
|---|---|---|
| `/camera/image` | 主程序绘制检测框后的 JPEG 图像 | Image |
| `/detections` | 类别、置信度、检测框、目标 ID | Raw Messages |
| `/fsm/state` | 状态、批次、交付计数、目标信息 | Raw Messages |
| `/decision/live` | 当前阶段、预检原因、目标和区域证据、安全许可、夹持、落点及计算输出 | Raw Messages、Plot |
| `/system/health` | 主循环帧率、推理耗时、图像延迟、遥测丢帧 | Plot |
| `/cmd/motion` | 算法计算出的速度值 | Plot |
| `/runtime/config` | 相机、模型和阈值配置 | Raw Messages |

服务端只支持订阅和退订，Foxglove capabilities 为空。参数写入、客户端发布和运动控制请求会被拒绝。

## 8. 手动启动方式

如果不使用管理脚本，需要两个终端。先激活虚拟环境，并在项目根目录执行：

终端一：

```bash
. .venv-telemetry/bin/activate
python3 tools/remote_camera_telemetry/project_bridge.py
```

终端二：

```bash
. .venv-telemetry/bin/activate
./build-arm64-telemetry/rescue_upper_host \
  --dry-run --no-show --telemetry
```

主程序和查看服务必须使用相同的快照路径；默认路径是：

```text
/dev/shm/rescue-telemetry.bin
```

## 9. 日志和验证

日志位置：

```bash
tail -f telemetry_logs/main.log
tail -f telemetry_logs/bridge.log
```

在有 Node.js 22+ 的电脑上、从项目工作副本运行协议验证脚本（板子无需安装 Node.js）：

```bash
node tools/remote_camera_telemetry/verify_project.mjs <板子IP>
```

它会检查 HTTP 状态、八个 Foxglove 通道、JPEG、时间戳递增以及只读能力。

网页状态中应看到：

```text
主程序在线 · 图像持续更新
```

如果显示主程序未更新，先检查：

```bash
python3 tools/remote_camera_telemetry/manage_project.py status
ps -ef | grep -E '[r]escue_upper_host|[p]roject_bridge.py'
ss -lntp | grep -E ':8765|:8080'
```

## 10. 常见问题

### 8765 或 8080 被占用

查看占用者：

```bash
ss -lntp | grep -E ':8765|:8080'
```

停止旧的独立相机服务或旧的查看服务后，再运行管理脚本。不要强行启动第二个实例。

### 相机被占用

```bash
fuser -v /dev/video0
```

停止占用相机的进程后重新启动主程序。旧的 `/home/cat/camera-telemetry/server.py` 会直接打开相机，不能与主程序同时运行。

### 找不到模型或 RKNN 运行库

```bash
ls -lh benchmark_results/best_fp16.rknn benchmark_results/librknnrt.so
```

这两个文件不在 Git 仓库中，需要单独从制品或备份恢复。

### 主程序提示不能使用 `--telemetry`

当前二进制必须由以下选项构建：

```bash
-DRESCUE_ENABLE_TELEMETRY=ON
```

### 页面能打开但没有图像

确认主程序在运行，并且 `/dev/shm/rescue-telemetry.bin` 的更新时间持续变化：

```bash
stat /dev/shm/rescue-telemetry.bin
```

主程序停止超过约 2 秒后，查看服务会把数据标记为过期，避免把旧图像误认为实时数据。

## 11. 当前功能边界

- `--dry-run` 不发送运动命令；显式 `--imu` 可打开 IMU 接收，显式 `--pitch-feedback` 可打开 MCU 只读反馈接收。
- `--hardware` 打开 MCU 串口下发 `PushTask` 输出（必须同时带 `--imu`，不能与 `--dry-run`/`--pitch-feedback`/回放模式同用），25Hz重发，命令停更150ms改发零速，退出补发零速，详见 [速度协议](VELOCITY_PROTOCOL.md)“主程序下发”。
- `hardware_output_enabled=0` 表示速度是算法计算值，不是底盘实测速度。
- IMU、地面几何、持有证据、受控场地和导航适配已有生产路径；实际可用性取决于当前模型、标定、传感器和许可。兼容现场策略含独立的放宽与假设，详见最终配置的 `assumptions`；本轮离线验证不能证明实车流程通过。ToF 没有接入主程序的实时测距来源。
- 遥测只保留最新快照，不记录所有中间状态，也不自动生成 MCAP。
- 查看服务断开不会阻塞主程序；主程序可以继续采集和推理。


---

# 开发参考

主入口与模块职责见 [当前架构](docs/ARCHITECTURE.md)，运行参数见 [运行配置](docs/RUNTIME_CONFIGURATION.md)。
`src/controller.cpp` 是未加入当前构建的历史控制器；`config/rescue.yaml` 是历史参考，当前运行配置使用显式 `--config` JSON。

2026-09-25 至 2026-10-04 的电控接口、终端控制脚本、IMU 接入和受控空场记录已迁入
[开发参考历史摘录](docs/history/README_DEV_REFERENCE_20261025.md)；更早的入口说明见
[历史阶段摘录](docs/history/README_BEFORE_APP_REFACTOR.md)。两者都不作为当前部署指南。
