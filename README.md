# ds_innovation 下载后的部署与运行

本文说明从 GitHub 下载 `ds_innovation` 后，如何在 LubanCat/RK3588S 这类 ARM64 板子上完成编译，并通过网页或 Foxglove 查看主程序的实时检测画面。

当前版本的主程序是**检测预览模式**：它会采集相机、运行 RKNN 检测、绘制检测框并输出任务状态；底盘运动输出仍关闭。IMU、ToF 和真实运动闭环尚未接入。

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
  build-essential cmake pkg-config libopencv-dev \
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

为了避免把大文件和板卡二进制提交到 Git，`.gitignore` 排除了 `benchmark_results/`。完整运行仍需要下面两个文件：

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

如果模型路径不同，运行时使用 `--model` 和 `--rknn-library` 指定。模型类别顺序必须与项目的七类模型契约一致。

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

| 通道 | 内容 | 推荐面板 |
|---|---|---|
| `/camera/image` | 主程序绘制检测框后的 JPEG 图像 | Image |
| `/detections` | 类别、置信度、检测框、目标 ID | Raw Messages |
| `/fsm/state` | 状态、批次、交付计数、目标信息 | Raw Messages |
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

本机可以运行协议验证脚本：

```bash
node tools/remote_camera_telemetry/verify_project.mjs <板子IP>
```

它会检查 HTTP 状态、六个 Foxglove 通道、JPEG、时间戳递增以及只读能力。

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

- `--dry-run` 模式不会打开串口，也不会向底盘发送运动命令。
- `hardware_output_enabled=0` 表示速度是算法计算值，不是底盘实测速度。
- IMU、ToF、完整安全区几何和真实运动闭环尚未接入。
- 遥测只保留最新快照，不记录所有中间状态，也不自动生成 MCAP。
- 查看服务断开不会阻塞主程序；主程序可以继续采集和推理。


---

# 项目说明与开发参考

# 智能救援地面推行上位机

当前 `rescue_upper_host` 入口已改为地面推行任务，移除了原先夹爪抓取、释放和定时前冲流程。主入口使用 `PushTask`，不调用旧的 `Controller` 夹爪控制代码。

**本阶段完成任务规则和离线回放。尚未完成实车控制接入。** 相机模式仅预览；普通启动会在打开串口前拒绝执行。需要补齐标定、合法推行路径、安全区半区、传感器与底盘适配器，才能使真实观测驱动运动。C++ RKNN 后端已接入现有 `benchmark_results/best_fp16.rknn` 和 `librknnrt.so`，默认输入为 448×448；ONNX 路径保留，但现有 best.onnx 在板子 OpenCV 4.6 下仍存在兼容问题。

## 已实现规则

- 首次成功交付只允许一件普通物资；成功后普通/核心物资每批最多三件，伤员一件。
- 危险目标和未知类别不能成为任务目标。候选需连续三个有效观测确认，锁定后不能换目标 ID 或类别。
- 前进要求有效的目标、米制几何、路径安全与外部安全监督许可。目标丢失、非有限几何值或观测超过 200 ms 时停车。
- 推行目的地必须是己方安全区，普通/核心进入物资半区，伤员进入伤员半区。
- 不以超时或检测框中心进入区域判定交付。需要目标完全进入对应半区、不压围栏/隔板、静止、交付数量正确，连续八帧确认，随后确认机器人与目标脱离才累计交付。
- 推入和后退有一秒上限，任务活动阶段有八秒上限。后退需单独提供后方安全许可。
- 暂停/安全中断放弃当前批次但保留已确认交付；显式 reset 清空任务统计。

状态流程：

```text
WAIT_START → SEARCH_TARGET → APPROACH_TARGET → PUSH_TARGET
           → ALIGN_ZONE → VERIFY_DELIVERY → BACK_OUT → SEARCH_TARGET
```

## 构建和验证

```bash
cmake -S . -B build-arm64 -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON
cmake --build build-arm64 -j4
ctest --test-dir build-arm64 --output-on-failure
./build-arm64/rescue_upper_host --push-replay tests/fixtures/push_delivery.json
```

在本机也可使用上述构建方式，但本机二进制不能复制到 ARM64 板子上运行。回放不需要模型、摄像头或串口；最后应出现 `SEARCH_TARGET batch=0 delivered=1`。固定示例只验证规则，不代表实车投放成功。

板子单图推理与相机预览：

```bash
./build-arm64/rescue_upper_host --detect-image benchmark_results/npu/sample.jpg --conf 0.25
./build-arm64/rescue_upper_host --dry-run --camera 0
```

## 模块入口

| 路径 | 用途 |
|---|---|
| `src/main.cpp` | 推行回放与相机预览入口，不发送电机/舵机包 |
| `include/rescue/push_task.hpp`、`src/push_task.cpp` | 正式推行任务规则、观测和输出接口 |
| `tests/push_task_tests.cpp` | 首单、批次、交付、目标丢失、超时和暂停等回归验证 |
| `tests/fixtures/push_delivery.json` | 单件普通物资完整交付回放 |
| `src/detector.cpp` | ONNX/RKNN 检测后端；RKNN 运行时动态加载 |
| `src/tracker.cpp`、`camera_calibration.cpp`、`sensor_fusion.cpp` | 可复用跟踪、标定、安全组件，需接入新主链路 |
| `src/uart_controller.cpp` | 10字节浮点运动包发送、4字节A6执行器反馈；尚未接入任务运动主链路 |
| `src/rescue_state_machine.cpp` | 先前的十四状态原型，仅保留用于原有组件测试，不是当前主入口 |
| `src/controller.cpp` | 历史抓取控制器，当前主入口不使用 |

输入字段和接入边界参见 [USAGE_GUIDE.md](USAGE_GUIDE.md)。`config/rescue.yaml` 是配置参考，当前 CLI 不读取该文件。

视觉模型契约、已实现能力及实车差距见 [VISION_INTEGRATION.md](VISION_INTEGRATION.md)。

相机自动标定与人工测量步骤见 [标定工具说明](tools/camera_calibration/README.md)。

## 2026-09-25 电控接口更新

上位机发送 `56 + float32 vx(m/s) + float32 wz(rad/s) + uint8 gripper_closed`，固定10字节、小端、无CRC；下位机反馈 `A6 + uint8 gripper_done + CRC16/Modbus`，固定4字节，CRC覆盖字节0..1。ToF字段保留注释，IMU状态独立。

调用 `UARTController::sendMotion(command)` 发送，使用 `latestActuatorFeedback()` 读取反馈。旧无参数 `execute()` 不再发送13字节协议。详情见 [速度协议](VELOCITY_PROTOCOL.md) 和 [反馈协议](SENSOR_PROTOCOL.md)。本次接口改动不开放主程序实车模式，也不构成执行器动作闭环。
