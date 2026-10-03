# ds_innovation 下载后的部署与运行

本文说明从 GitHub 下载 `ds_innovation` 后，如何在 LubanCat/RK3588S 这类 ARM64 板子上完成编译，并通过网页或 Foxglove 查看主程序的实时检测画面。

当前版本的主程序是**感知预览模式**，底盘运动输出仍关闭。阶段 0/1 已加入 IMU→SensorState、只读实际 pitch 接收、帧时刻匹配的地面映射、关键点过滤、二维位姿拟合、ZoneEstimate 质量门控与 IPPE 交叉校验。关键点模型、投放规划和真实运动闭环仍待后续阶段。

新增参数、六点标注规范、无硬件回放和真实标定要求见 [阶段 0/1 实现说明](STAGE01_IMPLEMENTATION.md)。

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

- `--dry-run` 不发送运动命令；显式 `--imu` 可打开 IMU 接收，显式 `--pitch-feedback` 可打开 MCU 只读反馈接收。
- `--hardware` 打开 MCU 串口下发 `PushTask` 输出（必须同时带 `--imu`，不能与 `--dry-run`/`--pitch-feedback`/回放模式同用），25Hz重发，命令停更150ms改发零速，退出补发零速，详见 [速度协议](VELOCITY_PROTOCOL.md)“主程序下发”。
- `hardware_output_enabled=0` 表示速度是算法计算值，不是底盘实测速度。
- IMU 和安全区几何处理接口已接入；ToF、投放规划尚未接入；`--hardware` 已能下发，但 `safety_ok/path_safe/retreat_safe` 等证据尚无生产者（`ground_contact_valid` 已由 GeometryPipeline 生成，见 STAGE_HW2_GROUND_CONTACT.md，但需要验收过的 PITCH_MODEL 标定与真实 pitch 读回），实车上 `PushTask` 仍停在 WAIT_START。
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
| `src/main.cpp` | 推行/像素几何回放与相机预览入口，不发送电机/舵机包 |
| `src/geometry_pipeline.cpp` | 同帧传感器匹配、地面映射、关键点位姿与区域观测适配 |
| `src/keypoint_filter.cpp`、`src/ground_pose_fitter.cpp` | 关键点过滤、固定尺度 SE(2) 拟合、两点先验及多帧重新捕获 |
| `include/rescue/push_task.hpp`、`src/push_task.cpp` | 正式推行任务规则、观测和输出接口 |
| `tests/push_task_tests.cpp` | 首单、批次、交付、目标丢失、超时和暂停等回归验证 |
| `tests/fixtures/push_delivery.json` | 单件普通物资完整交付回放 |
| `src/detector.cpp` | ONNX/RKNN 检测后端；RKNN 运行时动态加载 |
| `src/tracker.cpp`、`camera_calibration.cpp`、`sensor_fusion.cpp` | 可复用跟踪、标定、安全组件；地面映射与相机读回pitch绑定，`SensorFusion`已在预览中否决`safety_ok` |
| `src/uart_controller.cpp` | 15字节浮点运动包（夹爪+动作编号+相机pitch+CRC16）发送、8字节A6执行器反馈（含换行帧尾）；尚未接入任务运动主链路 |
| `src/imu_adapter.cpp` | HI91快照→`SensorState`（车体坐标姿态），供`SensorFusion`倾斜停车与地面映射倾斜判断 |
| `src/controller.cpp` | 历史抓取控制器，当前主入口不使用 |

输入字段和接入边界参见 [USAGE_GUIDE.md](USAGE_GUIDE.md)。`config/rescue.yaml` 是配置参考，当前 CLI 不读取该文件。

视觉模型契约、已实现能力及实车差距见 [VISION_INTEGRATION.md](VISION_INTEGRATION.md)。

相机自动标定与人工测量步骤见 [标定工具说明](tools/camera_calibration/README.md)。

## 2026-09-25 电控接口更新

上位机发送 `56 + float32 vx(m/s) + float32 wz(rad/s) + uint8 gripper_open(0关闭/1张开) + uint8 gripper_action_id + int16 camera_pitch_cdeg(0.01°，0平视、正值向下) + CRC16/Modbus`，固定15字节、小端，CRC覆盖字节0..12；下位机反馈 `A6 + gripper_done + gripper_action_id + int16 camera_pitch_cdeg(读回) + CRC16/Modbus`，固定8字节（末尾0x0A换行），CRC覆盖字节0..4；上位机用 `gripperActionResult()` 按编号确认夹爪完成，用 `cameraPitchResult()` 按读回角确认相机到位。ToF字段保留注释，IMU状态独立。

调用 `UARTController::sendMotion(command)` 发送，使用 `latestActuatorFeedback()` 读取反馈。旧无参数 `execute()` 不再发送旧双电机/双舵机协议。详情见 [速度协议](VELOCITY_PROTOCOL.md) 和 [反馈协议](SENSOR_PROTOCOL.md)。本次接口改动不开放主程序实车模式，也不构成执行器动作闭环。


### 终端手动控制下位机（不启动主程序）

夹爪开关可直接用脚本，自动读取当前动作编号并加1，重复发送直到下位机报完成（默认3s超时）：

```bash
tools/gripper/gripper_status.sh   # 查看当前动作编号/完成标志/pitch
tools/gripper/gripper_open.sh     # 张开
tools/gripper/gripper_close.sh    # 合上
```

可加 `--dry-run` 只打印将发送的包、`--port`/`--baud`/`--timeout` 修改参数。需要同时控制速度或相机时用下面的通用脚本。

相机pitch调整用 `tools/camera_pitch/`，夹爪保持张开。脚本先读A6反馈拿到当前动作编号，用“编号+1”发一次张开并等done=1，之后每包都沿用该编号和 `gripper_open=1`，调pitch不会再触发夹爪动作。角度单位为度，0平视、正值向下、负值向上，超出舵机限位时截断（下位机已放宽到±35°，脚本仍按±25°截断）：

```bash
tools/camera_pitch/pitch_status.sh     # 查看动作编号/夹爪开关状态/pitch读回（不发包）
tools/camera_pitch/pitch_set.sh 5     # 张开夹爪，相机向下20°；直接输入真实角度
tools/camera_pitch/pitch_set.sh -5     # 相机向上5°；限位±40°
tools/camera_pitch/pitch_repl.sh       # 交互模式：终端输入角度实时调整
```

交互模式下直接输入角度（如 `10`、`-5`、`0`）即可转动，`s` 查看读回角，`x` 打印当前包，`q` 或 Ctrl+C 退出。后台以20Hz持续发当前目标，速度固定为0；退出后下位机200ms超时停车，相机保持当前角度。

`pitch_set.sh` 在读回角进入容差（默认3°，`--tol` 修改）后再保持发送0.5s，`--timeout`（默认3s）内未到位则返回非0。同样支持 `--dry-run`、`--port`、`--baud`。读回精度约1~3°，不要把读回角当作精确外参使用，见 [速度协议](VELOCITY_PROTOCOL.md)“相机pitch”。

先断开网页串口助手、`rescue_upper_host --hardware` 等占用 `/dev/ttyACM0` 的程序（主程序独占该串口）。参数依次为：vx(m/s)、wz(rad/s)、夹爪(0关/1开)、动作编号(1..255)、pitch(0.01°，正值向下，舵机限位±2500)。脚本只发一包并打印下位机回包；速度非0时车只动一下，200ms后下位机超时停车。

```bash
python3 - 0 0 1 1 0 <<'EOF'
import os, sys, struct, termios, tty, time
vx, wz = float(sys.argv[1]), float(sys.argv[2])
g, gid, pitch = int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5])
pitch = max(-2500, min(2500, pitch))  # 舵机限位±25°
def crc(b):
    c = 0xFFFF
    for x in b:
        c ^= x
        for _ in range(8): c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c
b = struct.pack('<BffBBh', 0x56, vx, wz, g, gid, pitch)
pkt = b + struct.pack('<H', crc(b))
print('TX', pkt.hex(' ').upper())
fd = os.open('/dev/ttyACM0', os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
tty.setraw(fd); a = termios.tcgetattr(fd); a[4] = a[5] = termios.B115200; termios.tcsetattr(fd, termios.TCSANOW, a)
os.write(fd, pkt); time.sleep(0.3)
try: print('RX', os.read(fd, 64).hex(' ').upper())
except BlockingIOError: print('RX 无数据')
os.close(fd)
EOF
```

只需改第一行参数。夹爪每次换状态动作编号都要加1，编号不变下位机按重复包忽略。

| 第一行参数 | 效果 | 对应字节 |
|---|---|---|
| `0 0 1 1 0` | 夹爪张开，编号1 | `56 00 00 00 00 00 00 00 00 01 01 00 00 9A 41` |
| `0 0 0 2 0` | 夹爪合上，编号2 | `56 00 00 00 00 00 00 00 00 00 02 00 00 6B BD` |
| `0 0 0 2 2000` | 夹爪保持合上，相机向下20° | — |
| `0 0 0 2 -2000` | 相机向上20° | — |

回包 `A6 <done> <编号> <pitch低> <pitch高> <CRC低> <CRC高> 0A`，done=1且编号等于最新编号才表示夹爪动作完成。

### 终端发送速度脚本

`tools/send_velocity.py` 默认使用当前 15 字节运动协议，预览模式不会打开串口：

```bash
python3 tools/send_velocity.py --port /dev/ttyACM0 --vx 0.10 --wz 0
```

确认端口和机械状态后，添加 `--send` 才会实际发送。默认 25 Hz 发送 1 秒，结束时补发零速度停车帧：

```bash
python3 tools/send_velocity.py \
  --port /dev/ttyACM0 --baud 115200 \
  --vx 0 --wz 10 --gripper closed --action-id 0 \
  --duration 1 --send
```

默认退出时补发零速度停车帧；需要自行控制是否停车时可显式选择：

```bash
python3 tools/send_velocity.py --no-stop-on-exit --vx 0 --wz 10 --send
python3 tools/send_velocity.py --stop-on-exit --vx 0 --wz 0 --send
```

持续发送直到 `Ctrl+C`：

```bash
python3 tools/send_velocity.py --port /dev/ttyACM0 \
  --vx 0 --wz 0.5 --duration 0 --send
```

当前协议中 `--gripper open` 为 `1`、`--gripper closed` 为 `0`；夹爪状态变化时使用新的 `--action-id`（1..255），`0` 表示不触发新动作。`--pitch` 使用真实度数，限幅 ±40°。发送模式会监听现有 A6 执行器反馈并显示夹爪状态、动作编号和 pitch；下位机不回传速度帧，因此终端不能显示实测 `vx/wz`，只能显示发送命令。使用前应停止占用 `/dev/ttyACM0` 的其他程序。

旧六字节协议可用 `--protocol legacy6` 显式选择；不要将两种协议混用。

## HiPNUC IMU 接收（2026-09-26）

已接入 HI91 接收、CRC 校验、SI 单位转换、新鲜度检查及网页/Foxglove 数据显示。详细包格式、数据接口和故障边界见 [HIPNUC_IMU.md](HIPNUC_IMU.md)。

```bash
# 只看IMU，无需相机/模型，不发送控制字节
./build-arm64-telemetry/rescue_imu_monitor /dev/ttyUSB0 115200 30

# 在真实相机预览中接收IMU，网页增加IMU数据卡片
python3 tools/remote_camera_telemetry/manage_project.py start --imu --imu-port /dev/ttyUSB0 --imu-baud 115200
```

两个命令二选一，不能同时读取同一个IMU串口。已运行预览时，先执行管理脚本的 `stop` 再带上述参数启动。默认不启用IMU，避免未接设备时影响原有预览。`--dry-run --imu` 只允许IMU接收，不启用下位机运动输出。


### 查看速度脚本收到的下位机原始数据

在原有发送命令末尾增加 `--show-rx`：

```bash
python3 tools/send_velocity.py --vx 0 --wz 0 --duration 2 --send --show-rx
```

将 vx/wz 换成已验证的测试值即可。默认退出补发零速度帧。
`RX RAW` 是每次串口实际读取的字节块，可能包含半帧或多帧；
`RX A6 [CRC OK]` 是校验通过的完整8字节反馈，随后显示夹爪状态、动作编号与pitch。
结束时汇总收到的字节数和有效帧数；不加参数保持原有状态变化显示。
当前A6不包含实际线速度或角速度；此参数只显示已有回包，不能让固件自动增加速度反馈。
