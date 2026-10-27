# ds_innovation · 智能救援机器人上位机

运行于 LubanCat / RK3588S 的智能救援机器人上位机，使用 C++17、OpenCV 和 RKNN 完成视觉感知、任务决策和 MCU 控制，Python/Shell 提供标定、调试和遥测工具。

**本文件是唯一日常维护文档。** 以下说明以板端 `/home/cat/ds_innovation` 为准，更新于 2026-10-27。下发控制包固定 **15 字节**；IMU 在上位机内部参与观测、导航和健康检查，不下发 yaw 或目标转角。软件回归与整车实机验收分开。

目录：[项目与代码](#project) · [编译与测试](#build) · [启动与查看](#run) · [夹爪、相机与底盘控制](#manual-control) · [配置](#config) · [通信与IMU](#protocol) · [相机标定](#calibration) · [工具与排错](#tools) · [开发参考](#development)

<a id="project"></a>
## 项目与代码

上位机接收相机、IMU 和 MCU 状态，识别目标、估计几何、推进任务状态，再由 MotionLink 向 MCU 发送底盘和机构请求；遥测在旁路发布图像和状态。

| 检测类别（模型顺序） | 任务含义 | 处理方式 |
| --- | --- | --- |
| blue | dangerous_object，危险物 | 不作为搬运目标 |
| orange | injured_person，伤员 | 单独送入伤员半区 |
| green | ordinary_supply，普通物资 | 送入物资半区 |
| black | core_supply，核心物资 | 送入物资半区 |
| zone_blue | blue_safe_zone，蓝色安全区 | 区域归属与寻区，不作为搬运物或物块遮挡 |
| zone_red | red_safe_zone，红色安全区 | 区域归属与寻区，不作为搬运物或物块遮挡 |

Pose 模型的 `zone_left` 是物资半区，`zone_right` 是伤员半区；左右以站在安全区入口面向区内定义，不等于画面左右。队伍归属优先由与 pose 可见角点匹配的 zone_blue / zone_red 检测框确定；没有匹配检测时沿用已标定的 HSV 颜色判断。相互矛盾的区域检测保持未知。检测框可提供粗略寻区方向，精确几何与交付位置仍只使用 pose 关键点。

当前默认检测模型为 detect_v5_int8.rknn，类别 ID 0–5 顺序为 blue、orange、green、black、zone_blue、zone_red。检测输出支持单张量 [1,10,N] 或 boxes [1,4,N] + scores [1,6,N]；旧四分类模型仍可通过显式 --model 与 --classes blue,orange,green,black 使用。pose 文件为 zone_pose_int8.rknn，类别、4 个关键点顺序与 [1,18,N] 契约不变。models/detect_v4_manifest.json 仅描述旧模型，不是 v5 元数据。

~~~text
相机 → 检测 + 安全区 pose → 跟踪与几何 → 任务观测 → PushTask
IMU + MCU反馈 ────────────────────────↑               ↓
比赛许可 / 健康检查 → MotionLink → MCU → 底盘与机构
任务与感知快照 → 遥测文件 → 网页 / Foxglove
~~~

业务主流程为 `Startup（冲出）→ Search（搜索）→ Capture（捕获）→ Transport（搬运）→ Search`。

| 业务阶段 | 阶段内动作 | 实现 |
| --- | --- | --- |
| Startup | 暂不实现冲出；保持底盘零速度和方框原命令，下一次更新转入搜索 | `src/task/startup_phase.cpp` |
| Search | 扫描、候选重找、接近、准备、清扫；交付后的离区转向也归搜索 | `src/task/search_phase.cpp`，`search_stage.cpp`，`sweep_stage.cpp` |
| Capture | 前冲、落框、捕获核验 | `src/task/capture_phase.cpp` |
| Transport | 搬运寻区、门口对齐、入区、抬框释放、后退、交付核验 | `src/task/transport_phase.cpp` |

`include/rescue/task/task_phase.hpp` 定义四个 `TaskPhase`、阶段内动作及运行状态；`src/task/push_task.cpp` 的 `PushTask::update()` 负责公共输入处理、运行许可和阶段调度。阶段通过 `PhaseOutcome` 请求交接，由主调度统一应用。`SearchPhaseData`、`CapturePhaseData`、`TransportPhaseData` 分别保存动作进度和阶段数据，`TripContext` 保存跨阶段目标与本趟货物；`TaskTick` 只存本次更新的输入、权限与输出。

暂停/等待/故障由独立 `RunStatus` 表示，不增加业务阶段。失败恢复保留所属阶段并运行 `src/task/recovery_phase.cpp`，完成后回到搜索；暂停恢复仍执行原有重新观察/安全退回规则。捕获结果区分 `Verified` 与 `Assumed`，交付结果分别记录应交数量、记账数量、视觉确认，视觉通过不自动等于计分成功。清扫预算、限时、黑名单、帧事务及核验策略沿用已有实现与测试。

当前 Startup 是有意跳过的占位实现；旧 `startup_advance_*` 配置暂不驱动运动，后续冲出动作在 `startup_phase.cpp` 中实现。正常完成一趟或暂停恢复不会重新执行 Startup，显式 reset 才回到启动等待。

遥测提供 `decision.phase`、`decision.action`、`state.run_status`、恢复标志和捕获/交付结果。感知、导航与运动就绪层改读明确的请求字段。`PushState` 暂保留为兼容动作编号，用于既有硬件辅助逻辑、日志及离线回放；主流程已按四阶段调度，但本轮没有删除全部旧动作兼容代码。

回归入口 `rescue_task_phase_tests` 验证启动占位、搜索到捕获的行程/货物交接、暂停保货、恢复归属和故障停止；捕获/交付集成测试覆盖真实证据与假设证据、全部/部分/未确认交付。冻结历史基线保留，`runtime_four_phase_20261026.json` 仅在原协议补充基础上声明 Startup 占位造成的启动输出差异。

| 位置 | 职责 |
| --- | --- |
| `src/main.cpp`、`src/app/` | 入口、配置加载、生命周期、感知流水线、观测和离线任务 |
| `src/task/` | PushTask 与捕获、交付、退区、搜索、清扫等阶段 |
| `src/perception/` | 检测、RKNN、跟踪、颜色、像素和目标选择 |
| `src/geometry/` | 相机标定、地面映射、安全区定位和规划 |
| `src/hardware/` | 相机、MCU/UART、IMU、传感器融合、MotionLink |
| `src/control/`、`src/config/` | 比赛许可、控制服务、运行配置和标定策略 |
| `src/core/`、`src/telemetry/`、`src/tools/` | 基础类型、日志、遥测、独立监视器 |
| `include/rescue/` | 与实现职责对应的头文件 |
| `config/`、`models/` | 统一配置、相机 YAML、模型文件 |
| `tools/`、`tests/` | 按场景组织的工具、按模块组织的软件测试 |

<a id="build"></a>
## 编译与测试

### 进入板端

~~~bash
ssh cat
whoami
hostname
cd /home/cat/ds_innovation
~~~

用户和主机应分别为 `cat`、`lubancat`；连接地址使用当前已核实 SSH 配置。不要用旧 IP 或本地旧副本覆盖板端。

依赖：C++17 编译器、OpenCV 4.x、nlohmann/json、CMake 3.21+、Ninja、Python 3；工具测试还需要 `cv2`、`numpy`、`websockets`。

~~~bash
sudo apt install build-essential cmake ninja-build pkg-config libopencv-dev nlohmann-json3-dev python3 python3-venv v4l-utils
~~~

RKNN 模型和 SDK 资源独立部署，Git 克隆不包含这些被忽略的文件：

~~~text
models/detect_v5_int8.rknn
models/zone_pose_int8.rknn
benchmark_results/librknnrt.so
benchmark_results/rknn_api.h
~~~

### 标准构建与回归

~~~bash
cmake --preset cat-release
cmake --build --preset cat-release
ctest --preset cat-release
~~~

板端 `CMakeUserPresets.json` 的 `cat-release` 继承共享 `release`：Ninja、Release、RKNN、遥测和测试，输出为 `build-release/`。板端预设包含本机依赖路径；其他机器使用自己的依赖配置，不直接复制。已有构建缓存沿用原生成器。

~~~bash
# 只构建主程序
cmake --build --preset cat-release --target rescue_upper_host
# 列出测试，按模块或层次筛选
ctest --preset cat-release -N
ctest --preset cat-release -L protocol
ctest --preset cat-release -L task
ctest --preset cat-release -L calibration
ctest --preset cat-release -L integration
# 单项回归
ctest --preset cat-release -R '^rescue_push_tests$'
~~~

最终验收使用未筛选的全量 CTest。测试使用合成观测、虚拟串口、临时文件和本机回环服务，不启动真实运动。

<a id="run"></a>
## 启动与查看

### 无设备检查

~~~bash
./build-release/rescue_upper_host --config config/project.json#/runtime_profiles/push_replay
./build-release/rescue_upper_host --config config/project.json --dry-run --imu --pitch-feedback --dump-config /tmp/rescue-effective-config.json
tools/match/match_main.sh --check
~~~

回放不打开相机或串口，成功摘要包含 `delivered=1 first=1`。配置导出在设备初始化前退出。比赛入口检查应打印 `build-release/rescue_upper_host` 的实际路径、摘要和构建时间；`RESCUE_BINARY` 可显式覆盖，应核对它没有指向旧程序。

### 主程序只读预览

相机和串口应没有被其他进程占用。当前 MCU 默认 `/dev/ttyACM0`，IMU 默认 `/dev/ttyUSB0`，均为 115200。网页桥需要 `websockets` 10.x；缺少时可在虚拟环境中安装：

~~~bash
python3 -m venv .venv-telemetry
. .venv-telemetry/bin/activate
python -m pip install 'websockets>=10,<11'
~~~

**运行观测启动命令**（在板端执行；采集视觉、IMU 和 MCU 相机角度反馈，不发送运动控制）：

~~~bash
cd /home/cat/ds_innovation
python3 tools/remote_camera_telemetry/manage_project.py start \
  --config config/project.json \
  --dry-run --no-show --imu --pitch-feedback --vision-log-dir none
~~~

启动前退出相机角度控制台；观测运行期间仍会占用 MCU 串口。查看运行状态：

~~~bash
python3 tools/remote_camera_telemetry/manage_project.py status
~~~

| 查看入口 | 地址 / 内容 |
| --- | --- |
| 网页 | `http://<当前板子IP>:8080/`，图像与状态；`/status` 返回状态数据 |
| Foxglove WebSocket | `ws://<当前板子IP>:8765` |
| 主程序日志 | `telemetry_logs/main.log` |
| 桥接日志 | `telemetry_logs/bridge.log` |
| 板上监看 | `python3 tools/board_monitor/monitor.py --open`；本机 `http://127.0.0.1:18080/` |

常用通道：`/camera/image`、`/detections`、`/fsm/state`、`/decision/live`、`/runtime/config`、`/cmd/motion`、`/system/health`、`/imu/data`。任务计算速度不等于实际车速，图像刷新也不代表 IMU 或 MCU 反馈新鲜。

~~~bash
python3 tools/remote_camera_telemetry/manage_project.py stop
~~~

比赛程序已运行时直接连接它的查看服务，不再启动第二个预览。`project_bridge.py` 读取 `/dev/shm/rescue-telemetry.bin`，不打开相机、不发送控制；独立 `server.py` 会打开相机。Foxglove 可能保留最后一帧，应同时查看连接状态和数据年龄。

#### 观测启动失败：残留 match socket

若启动时显示 `Project preview and viewing service stopped`，随后提示 `A process exited; see telemetry_logs/main.log and bridge.log`，先查看日志；前一行是启动失败后的自动清理提示，具体原因以日志为准。

~~~bash
cd /home/cat/ds_innovation
tail -n 80 telemetry_logs/main.log
tail -n 80 telemetry_logs/bridge.log
~~~

如果主程序日志包含以下报错，表示控制 socket 已被占用，或上次退出后留下了文件：

~~~text
[FATAL] Cannot bind match socket (already running or stale path): /tmp/rescue-match.sock
~~~

先确认是否仍有主程序或 socket 监听者：

~~~bash
pgrep -af '[r]escue_upper_host'
ss -xlpn | grep -F /tmp/rescue-match.sock
ls -l /tmp/rescue-match.sock
~~~

仅在前两条均无输出、确认主程序已经退出且 socket 文件仍存在时，删除残留文件并重新启动观测。有运行进程时先正常停止对应程序，不要删除正在使用的 socket。

~~~bash
rm /tmp/rescue-match.sock
python3 tools/remote_camera_telemetry/manage_project.py start \
  --config config/project.json \
  --dry-run --no-show --imu --pitch-feedback --vision-log-dir none
~~~

### 比赛与机构入口

| 命令 / 模式 | 行为 |
| --- | --- |
| `--dump-config`、`--check-config` | 配置导出或检查，不打开设备 |
| `--dry-run` | 可采集与推理，不发送 MCU 控制 |
| `tools/match/match_main.sh --check` | 默认静态检查 |
| `tools/match/match_main.sh --status` | 只读 MCU 状态，需串口独占 |
| `tools/match/match_main.sh --run --site-clear …` | 实机比赛运行，执行底盘和机构动作 |

比赛启动需要选择现场确认的方框初态及满足运行许可；通过 `bash tools/match/match_main.sh --help` 查看参数。`--frame open/close` 会先发反向再发目标动作，`--frame keep` 需要核对已有状态和编号。停止、急停与 MCU 失联行为须独立实机验收。自动启动是否启用以板端服务状态为准；本次整理不更改服务。

<a id="manual-control"></a>
### 手动控制夹爪、相机与底盘

以下命令在板端执行，MCU 串口为 `/dev/ttyACM0`、波特率 115200。先退出占用该串口的主程序或其他串口工具；每次只运行一个控制命令。

#### 夹爪打开、关闭与状态

下面带 `/frame/assume` 的打开命令已由用户在实机确认有效。该配置使用 A6 状态 `1=开、0=关`，TX 偏移为 `+5=开、-5=关`。配置仍标记为临时假设，不代表整车验收。

~~~bash
# 打开夹爪
python3 /home/cat/ds_innovation/tools/frame/frame_ctl.py open \
  --frame-config '/home/cat/ds_innovation/config/project.json#/frame/assume'

# 关闭夹爪
python3 /home/cat/ds_innovation/tools/frame/frame_ctl.py close \
  --frame-config '/home/cat/ds_innovation/config/project.json#/frame/assume'

# 只读状态，不发送控制包
python3 /home/cat/ds_innovation/tools/frame/frame_ctl.py status
~~~

打开、关闭命令同时发送零底盘速度，并保持首次 A6 反馈中的相机角度。需要只读反馈并预览控制包时，在开合命令末尾加 `--dry-run`。省略 `--frame-config` 会读取尚未验收的 `/frame/pending`，报错 `A6 / physical action mapping has not been accepted`。

#### 相机角度控制

先退出占用 MCU 串口的主程序或其他串口工具。通过遥测管理器启动的预览可用 `python3 tools/remote_camera_telemetry/manage_project.py stop` 停止；`--dry-run` 主程序仍会占用串口。

~~~bash
cd /home/cat/ds_innovation

# 只读当前相机角度，不发送控制包（同样需要串口独占）
sudo python3 tools/remote_debug/camera_angle_console.py --status

# 启动交互式角度控制台
sudo python3 tools/remote_debug/camera_angle_console.py
~~~

出现 `相机角度>` 提示后，输入 `90..150` 内的角度并回车，例如 `120` 回中位、`130` 向下调整；角度增大向下。输入 `s` 读取状态，`q` 退出，退出时不自动复位相机。控制包中的底盘速度固定为零，并保持指定夹爪偏移。

如果提示“编号非0，无法推断夹爪偏移”，需显式提供当前已知的夹爪偏移（范围 `-5..5`）；按上述已确认的夹爪配置，打开为 `5`、关闭为 `-5`。例如，已确认夹爪当前保持打开时：

~~~bash
sudo python3 tools/remote_debug/camera_angle_console.py --hold-gripper-offset 5
~~~

不要用猜测的偏移代替当前值，以免同时改变夹爪。默认串口为 `/dev/ttyACM0`、波特率 115200，可通过 `--port` 和 `--baud` 指定。若出现 `[Errno 11] Resource temporarily unavailable`，先用 `fuser /dev/ttyACM0` 或 `lslocks` 查找占用者并退出对应程序；`sudo` 不会绕过串口独占锁。

#### 底盘前进、后退与转向

先在当前终端选择已有的演示配置；每个新终端都需执行一次。该 profile 使用 `/frame/assume` 和临时演示验收配置。

~~~bash
export RESCUE_DEMO_PROFILE='/home/cat/ds_innovation/config/project.json#/profiles/demo_assume'

# 只检查参数和配置，不打开设备
python3 /home/cat/ds_innovation/tools/demo/demo.py move-forward --speed 0.05 --seconds 0.5 --check

# 前进：0.05 m/s，持续 0.5 秒
python3 /home/cat/ds_innovation/tools/demo/demo.py move-forward --speed 0.05 --seconds 0.5 --run --site-clear

# 后退：0.05 m/s，持续 0.5 秒
python3 /home/cat/ds_innovation/tools/demo/demo.py move-backward --speed 0.05 --seconds 0.5 --run --site-clear

# 原地左转（逆时针）：0.20 rad/s，持续 0.5 秒
python3 /home/cat/ds_innovation/tools/demo/demo.py turn-left --wz 0.20 --seconds 0.5 --run --site-clear

# 原地右转（顺时针）：0.20 rad/s，持续 0.5 秒
python3 /home/cat/ds_innovation/tools/demo/demo.py turn-right --wz 0.20 --seconds 0.5 --run --site-clear
~~~

`--run` 执行动作；`--site-clear` 表示操作者已确认运动空间清空。省略 `--run` 默认仅检查。直线速度 `--speed` 范围为 `(0, 0.10]` m/s，转向速度 `--wz` 为 `(0, 0.25]` rad/s；方向由子命令决定，参数填写正数。显式 `--seconds` 范围为 `(0, 1]` 秒。

**去掉 `--seconds 0.5` 后会持续运动**，直到当前终端按 `Ctrl+C`，或在另一板端终端执行停止命令：

~~~bash
# 查看本套 demo 工具启动的会话
python3 /home/cat/ds_innovation/tools/demo/demo.py status

# 停止本套 demo 工具启动的会话
python3 /home/cat/ds_innovation/tools/demo/demo.py stop
~~~

底盘工具按初始 A6 反馈保持夹爪和相机目标，正常结束或中断时补发零速度。若提示 A6 动作编号为 0、无法确定夹爪状态，先执行上面的夹爪开合命令再重试。`stop` 只管理本套 demo 会话，不会停止其他独立程序；没有会话记录不等于实物已停车。

<a id="config"></a>
## 配置

`config/project.json` 是运行参数、演示 profile、机构与区域资源的统一入口；相机矩阵继续使用 OpenCV YAML。

| 分组 / 资源 | 用途 |
| --- | --- |
| `hardware` | 相机模式、MCU 和 IMU 端口、波特率、反馈开关 |
| `models` | 检测/pose 模型、类别、阈值、NPU 核与并行推理 |
| `calibration` | 相机 YAML、区域/任务/颜色资源、俯仰预设 |
| `behavior`、`runtime` | 行为参数、队伍、运行模式、遥测与日志 |
| `profiles` | demo 工具 profile，与主程序 runtime profile 区分 |
| `runtime_profiles/preview`、`runtime_profiles/push_replay` | 独立完整的主程序配置片段 |
| `frame`、`acceptance`、`task_calibration`、`zone_color`、`zone_geometry` | 测量、假设与验收资源，不能仅凭路径当作已验收 |
| `config/camera.yaml` | 当前唯一运行相机参数：2026-10-27 手眼旋转与尺量平移，固定 UI 90°（内部 -3000 cdeg），地面验证已由用户确认完成 |

主程序与所有 demo profile 统一使用上述相机文件；旧 5°/40° 参数仅在 `tests/fixtures/legacy_config/` 作为冻结回归输入保留，不用于启动。原始标定会话保留在 `calibration_runs/`。

请求按“默认值 → JSON → 显式命令行”解析，之后兼容策略和标定还可能调整最终值；用 `--dump-config` 同时检查请求、来源和最终参数。导出快照不是可直接回读的输入 profile。

~~~bash
./build-release/rescue_upper_host --config config/project.json#/runtime_profiles/preview --dump-config -
RESCUE_DEMO_PROFILE="$PWD/config/project.json#/profiles/demo_default" python3 tools/demo/demo.py recognize
~~~

JSON 片段是完整 schema v1 配置，不与根节点叠加；文件路径相对 `project.json` 所在目录，命令行路径相对当前目录，不展开 `~` 或环境变量。未知键、重复键、类型错误和非法数字会报错。`schema_version` 为整数 1，`behavior_policy` 为 `legacy_field_v1` 或用于任务回放的 `legacy_replay_v1`。

配置字段的完整映射见 `src/app/config_loader.cpp`，运行参数见 `./build-release/rescue_upper_host --help`。旧分散配置只在 `tests/fixtures/legacy_config/` 保留冻结字节，不作为当前启动配置。

当前运行配置使用已确认地面验证的 2026-10-27 外参，俯仰预设统一为 `[-3000,-3000,-3000]`，对应 UI 90°；比赛入口也定位到此角度。

<a id="protocol"></a>
## 通信与 IMU

### 上位机下发：固定 15 字节

MCU 串口默认 115200、8N1。小端、1 字节对齐，浮点为 IEEE754 float32。

| 偏移 | 类型 | 含义 |
| --- | --- | --- |
| 0 | uint8 | 帧头 `0x56` |
| 1..4 | float32 | `vx`，m/s；正前进、负后退 |
| 5..8 | float32 | `wz`，rad/s；正逆时针、负顺时针 |
| 9 | int8 | 夹爪偏移：+5 开、-5 关、0 为 85° 中位 |
| 10 | uint8 | 动作编号，1..255 循环；0 表示无动作 |
| 11..12 | int16 | 相机整数度偏移，-30..+30；0 对应绝对舵机角 120° |
| 13..14 | uint16 | CRC16/Modbus，覆盖字节 0..12，低字节在前 |

CRC 初值 `0xFFFF`、多项式 `0xA001`，无最终异或。包内没有 IMU yaw、目标转角或其他扩展字段，15 字节布局不与历史 17/19 字节包混用。

业务层仍用方框逻辑值 `0=开、20=关`，编码器映射为 +5/-5；不能把业务层 0 当作线上中位。MCU 按偏移变化计数，上位机重试同一目标保持编号，重启后通过 A6 同步。协议编码器透传有限速度；任一速度为 NaN/Inf 时两轴归零，上层运动许可和限速仍生效。

MotionLink 在业务提交时立即发送，默认每 40 ms 重发；命令超过 150 ms 或许可撤销时替换为零速度，保持机构目标。首次提交前不发送，退出补发零速度。转向仍由 `wz` 请求，上位机不发送相对角度控制请求。

金向量（零速度、相机中位）：

~~~text
开夹爪、编号1：56 00 00 00 00 00 00 00 00 05 01 00 00 9B 71
关夹爪、编号2：56 00 00 00 00 00 00 00 00 FB 02 00 00 5A 99
~~~

`tools/send_velocity.py` 默认 `latest15`，仅预览，`--send` 被既有工具策略拒绝；旧 `legacy6` 仅用于明确指定的六字节预览。`--yaw`、`--target-angle` 和旧 `latest17/latest19` 选项已移除。方框、俯仰、demo 和相机控制台均使用相同 15 字节布局。

### MCU 反馈：A6 固定 8 字节

| 偏移 | 类型 | 含义 |
| --- | --- | --- |
| 0 | uint8 | `0xA6` |
| 1 | uint8 | 夹爪状态，当前约定 0 关、1 开 |
| 2 | uint8 | 动作编号 |
| 3..4 | int16 | 相机绝对舵机角，90..150° |
| 5..6 | uint16 | CRC16/Modbus，覆盖 0..4，低字节在前 |
| 7 | uint8 | 帧尾 `0x0A` |

A6 二值状态与 TX 有符号偏移不能混用；状态、编号匹配不独立证明夹持或交付成功。解析按帧头、定长、CRC、帧尾进行，不能按换行切包。默认反馈超过 200 ms 失效；非法角度转换为 `-32768`。

主程序和标定内部角度是偏移百分之一度：`pitch_cdeg=(A6绝对角−120)×100`，例如 125° 对应 500；发送时除以 100 并限幅 ±30°。俯仰工具使用绝对角，详见各工具 `--help`。

### IMU 保留在上位机内部

HI91 独立接 `/dev/ttyUSB0`，不在 A6 中。`HipnucImu::snapshot()` 提供原始及车体姿态、样本序号、新鲜度和数值有效性，供观测、导航、健康判断及遥测使用。

当前安装补偿为绕 Y 轴 180°：设备到 `base_link`（X 前、Y 左、Z 上）使用 `diag(-1,+1,-1)`；加速度/角速度变为 `(-x,+y,-z)`，四元数按 `q_body=q_device⊗(0,0,1,0)`，再提取姿态。不要额外叠加设备内部旋转补偿，也不要把几何模块的 x右/y前坐标与 base_link 直接混用。

默认 200 ms 无推进样本即过期；原始状态字、CRC、序号和数据年龄保留用于诊断。IMU 接收有效不等于安装和动态姿态已经实机验收。

<details>
<summary>HI91 原始包参考</summary>

小端 82 字节：`5A A5` 帧头、uint16 载荷长度 76、uint16 CRC、类型 `0x91`、状态、温度、气压、设备时间、加速度、角速度、磁场、欧拉角及四元数。CRC16/XMODEM 初值 0、多项式 `0x1021`，覆盖 0..3 和 6..81，跳过 CRC 本身。

| 偏移 | 数据 |
| --- | --- |
| 6、7..8、9 | 类型、uint16 状态、int8 温度 |
| 10..13、14..17 | float32 气压 Pa、uint32 设备时间 ms |
| 18..29、30..41、42..53 | float32[3] 加速度 g、角速度 °/s、磁场 µT |
| 54..65、66..81 | float32[3] roll/pitch/yaw °、float32[4] 四元数 w/x/y/z |

实现和异常处理见 `src/hardware/hipnuc_imu.cpp`，回归见 `tests/hardware/imu_tests.cpp`。

</details>

<a id="calibration"></a>
## 相机标定

### 相机—IMU 旋转外参：固定板、多轴运动 AX=XB

现在外参采集可使用网页的 handeye 模式。保留原有内参和地面测量入口；本模式只求相机光学系到原始 IMU 设备系的旋转，不把未知平移设为零，也不自动覆盖 config/camera.yaml。

~~~bash
cd /home/cat/ds_innovation
python3 tools/camera_calibration/guided.py --mode handeye \
  --intrinsics config/camera.yaml --imu-port /dev/ttyUSB0 --imu-baud 115200 \
  --stop-preview
~~~

浏览器打开 http://<当前板端地址>:8081/。默认 1280×720，必须与内参一致；Foxglove 仍为 8766，沿用 /calibration/image、/calibration/status、/calibration/result。仅预览不记录 IMU；点击开始后连续记录，程序不发送底盘、相机舵机或其他机构动作。

1. 将实物 9×6 圆点板固定竖立在距相机约 1 m 处；填写实测横向 8 间距和纵向 5 间距。板太小或识别失败时使用更大板或调整距离，不能假定 1 m 一定满足像素要求。
2. 相机与 IMU 刚性连接，保持焦点、相机舵机角度固定。使用稳定支撑，人工整体改变偏航、俯仰、横滚；板保持不动，不使用舵机代替整体俯仰。不具备多轴动作条件时无法完成本模式。
3. 开始后整体静止至少 5 秒，第一张估计陀螺仪零偏。之后按网页指引转动约 10°～30°，每次停稳约 2 秒再拍摄。两张间隔不超过 45 秒。允许回到之前姿态；相邻两张至少转动 5°，相邻转角不超过 80°。
4. 共 25 张：前 17 张拟合，后 8 张组成独立验证序列，拟合与验证之间不构造交叉运动对。全部样本都记录图像时间戳、圆心、原图以及完整 IMU JSONL，撤销照片时仍保留原记录。
5. 自动检查零偏静止段、IMU 数值和序号、设备时间回绕/倒退/100 ms 断流、两路转角一致性、PnP 重投影与单轴退化；对称圆点倒序按姿态连续性处理。拟合/验证分别要求至少 6 段有效转动，旋转轴第二/第一奇异值比至少 0.15。
6. 默认残差检查为拟合及验证最大 2°、验证中位数 1°；它们是软件初始质量门限，不代表实机精度保证。记录过程使用陀螺仪原始 imu_device 轴、rad/s，积分采用设备时钟间隔；图像使用读取完成的主机单调时间，停稳端点降低时间差影响。--camera-time-offset-ms 将固定毫秒偏移加到图像时间，不自动估计延迟；若存在同步问题，应修正后重采。

输出目录为 calibration_runs/handeye_日期_时间_随机编号/，包含 intrinsics.yaml 快照、imu.jsonl、imu_stderr.log、samples/、detected/、逐帧 JSON、session.json、camera_imu_rotation.yaml、report.json 和使用说明。结果定义为 v_imu_device = R_imu_from_camera · v_camera_optical；同时保存逆矩阵 R_camera_from_imu，translation_estimated=0、extrinsics_validated=0。有告警时状态为 needs_recapture，否则仍为 candidate_requires_validation，需要重复采集和实机独立验证。

~~~bash
# 保留原始记录，用当前求解器重算完整会话；模式自动从 session.json 识别
python3 tools/camera_calibration/guided.py --recalculate calibration_runs/<handeye会话目录>
# 单独检查只读 IMU 记录入口；不连接 MCU
./build-release/rescue_imu_record --help
ctest --preset cat-release -L calibration
~~~

主程序的 T_camera_from_robot、ground_homography 需要平移、地面关系及适用俯仰范围，本结果不能直接替代它们。相机舵机改变角度后 R_imu_from_camera 也会改变，必须在同一机构角度使用或另行建模。项目已确认 IMU→base_link FLU 为 diag(-1,1,-1)；转换车体旋转时可左乘，但地面几何的 robot 坐标约定需另外核对，不混用 FLU 与 x右/y前。

### 原有内参与地面标定

### 推荐流程：9×6 对称圆点板

1. 固定镜头焦点、分辨率和相机安装姿态；避免其他预览占用摄像头。
2. 在板端启动 `python3 tools/camera_calibration/guided.py --stop-preview`，打开 `http://<当前板子IP>:8081/`；圆点板地址为 `/board.pdf`。默认 1280×720，须与实际主程序一致。
3. 平板显示完整圆点板，锁定缩放、亮度和旋转。量横向第 1 至第 9 圆心跨度、纵向第 1 至第 6 圆心跨度，单位 mm；分别除以 8 和 5 得间距。两方向差异超过 1% 时先修正显示或测量。
4. 按网页完成 25 个姿态，停稳后保存原始 PNG；前 22 张拟合，后 3 张独立验证。采集期间不能改变焦点、分辨率或图案缩放。
5. 检查 `report.json` 和去畸变结果。`intrinsics.yaml` 只有内参，不可直接当完整 `camera.yaml` 使用。
6. 将圆点板水平放在地面，测量其相对车体的位置，拍照求外参，再用板外独立地面点验收。
7. 通过后备份原相机配置，部署正式 `camera.yaml`，在所用 profile 的 `calibration.calibration_file` 中明确选择，并用配置导出核对。

网页采集结果在 `calibration_runs/circles_日期_时间_随机编号/`，含原图、检测图、会话、内参与报告；中断后照片保留，半途会话不自动恢复。完整会话可用 `guided.py --recalculate <会话目录>` 重算。

标定 Foxglove 为 `ws://<当前板子IP>:8766`，通道 `/calibration/image`、`/calibration/status`、`/calibration/result`；拍摄由网页操作，Foxglove 只读。退出标定后按“主程序只读预览”重新启动查看。

<details>
<summary>外参、地面检查与角度范围</summary>

几何坐标为 x右/y前/z上，平移单位 m；相机坐标为 x图像右/y图像下/z光轴前。测量参数：

| 参数 | 要求 |
| --- | --- |
| `--square-mm` | 实测圆心间距 mm |
| `--origin-x/--origin-y` | 编号 0 圆心相对底盘中心地面投影的位置 m |
| `--yaw-deg` | 板 +X（0→1）从车体 +x 向 +y 的转角 |
| `--board-height-mm` | 圆点表面离地高度，包含垫板 |
| `--camera-pitch-cdeg` | 当前相机偏移百分之一度，例如 A6 125° 填 500 |

先保存原分辨率图像并计算候选。以下尖括号均须替换为自己的文件和实测值：

~~~bash
python3 tools/camera_calibration/calibrate.py snapshot --camera /dev/video0 --width 1280 --height 720 --output ground_board.png
python3 tools/camera_calibration/calibrate.py extrinsics \
  --pattern circles --cols 9 --rows 6 \
  --intrinsics <会话目录>/intrinsics.yaml --image ground_board.png \
  --square-mm <间距mm> --origin-x <x米> --origin-y <y米> \
  --yaw-deg <方向角> --board-height-mm <表面高度mm> \
  --camera-pitch-cdeg <偏移百分之一度> --output <新的候选目录>
~~~

核对 `corners_numbered.png` 的编号 0、0→1 和 0→9 是否符合测量基准。可用 `--flip-cols/--flip-rows` 调整编号；当板 +Y 朝车后时使用 `--reverse-board-y`。改变编号时同时复核坐标。

在板外实测至少 3 个分布于近远左右的地面检查点，创建 `world.json`（`fit: []`，`check` 中每项含实测 `world_m: [x,y]`），在原图上选点：

~~~bash
python3 tools/camera_calibration/calibrate.py pick --image ground_board.png --world world.json --output checks.json
~~~

使用原候选的同一组参数和新输出目录重新执行 `extrinsics`，增加 `--confirm-order --check-points checks.json --max-error-mm 20`。只有方向确认、位姿检查及独立误差通过才产生正式 `camera.yaml`；否则保留候选。默认重投影 RMS 门槛 0.5 px，地面点最大误差 20 mm；小 RMS 不替代独立测量。

单一姿态标定只适用于其验证角度。扩展范围须在其他俯仰角重新量取独立地面点，用 `calibrate.py pitch-range --camera <已验收camera.yaml> --check <偏移cdeg>:<checks.json> --output <新目录>` 验证；可重复 `--check`，默认相邻验证角间隔不超过 15°。旧 40° 结果不能作为当前 30° 的标定。

手工平面映射备用流程为 `snapshot → pick → ground`，至少 6 个拟合点与 3 个独立检查点。棋盘内参备用入口为 `calibrate.py intrinsics`；全部参数以对应子命令 `--help` 为准。

同一相机的焦点、分辨率、安装位置或俯仰改变后应复核适用范围；标定工具不会自动覆盖当前生效配置。

</details>

<a id="tools"></a>
## 工具与排错

| 工具 | 用途 |
| --- | --- |
| `tools/match/` | 正式比赛、只读 MCU 状态、遥测桥管理 |
| `tools/demo/demo.py` | recognize/search/carry_once 等分项入口，读取 demo profile |
| `tools/frame/frame_ctl.py` | open/close/status；up/down 为兼容别名 |
| `tools/gripper/` | 方框工具的旧名称兼容入口 |
| `tools/camera_pitch/pitch_ctl.py` | 相机俯仰，保持反馈中的夹爪状态 |
| `tools/remote_debug/camera_angle_console.py` | 零底盘速度的相机控制台，仍会操作相机机构 |
| `tools/send_velocity.py` | 15 字节包预览，不打开串口 |
| `tools/board_monitor/monitor.py` | 板上浏览器监看；`install_desktop.py` 安装桌面入口 |
| `tools/runtime/record_baseline.py` | 冻结行为基线记录与比较 |

方框工具的 `status` 与 `--dry-run` 只读串口；执行 open/close 时速度为零，但机构会动作。它保持首次新鲜 A6 相机角，要求有效状态和编号，超时停止重发且不反向动作。相机/方框工具与主程序须独占串口。传入 `--help` 查看用法不会执行动作。

包预览示例：

~~~bash
python3 tools/send_velocity.py --protocol latest15 --vx 0.1 --wz -0.25 --frame-offset 0 --pitch 120
~~~

| 现象 | 检查方向 |
| --- | --- |
| SSH 名称失效 | 检查当前 SSH 别名和网络解析，连接后核对用户与主机 |
| 编译缺依赖 | 检查板端预设、OpenCV/nlohmann、RKNN 头和库；工具测试需 cv2/numpy/websockets |
| 刚编译却像旧程序 | 查看比赛入口 identity、`RESCUE_BINARY` 和实际构建目录 |
| 相机/串口被占用 | 查询原服务和进程，正常停止后使用；不同时启动多个读取器 |
| 无 A6 或 CRC 失败 | 核对 115200/8N1、A6 定长 8 字节和 TX 定长 15 字节；反馈状态与发送偏移分开 |
| pitch_not_calibrated | 核对 A6 绝对角→内部偏移换算、实际选中的 YAML 和已验收角度范围 |
| 网页有旧图/IMU失效 | 查主程序及桥日志、frame_age、IMU age、连接和有效标志 |
| 修改参数没变化 | 导出最终配置，检查所选片段、CLI 和后续兼容策略 |
| 冻结回归失败 | 先检查资源摘要、历史夹具和差异，不能重录预期消除失败 |

<a id="development"></a>
## 开发参考

### 测试组织与格式

`tests/CMakeLists.txt` 统一注册测试。`task/stages` 为单阶段，`task/integration` 为阶段接入，`task/regression` 为跨阶段；其余按 perception、geometry、hardware、config、control、telemetry、tools、regression 分类。`fixtures`、`samples` 保留冻结数据与样本。工具本地测试继续位于对应工具目录。

`rescue_push_tests` 保留统一入口和场景顺序，共享模拟器位于 `tests/support/push_task_sim.hpp`。新增测试登记源码、依赖、标签与超时；主题相近不代表测试重复。

<a id="formatting"></a>
C/C++ 使用 `.clang-format`，固定 `clang-format 18.1.8`：4 空格、100 列、Attach 大括号、不排序 include。工具优先选择 `.tools/venv/bin/clang-format`。

~~~bash
# 缺少固定版本工具时安装
python3 -m venv .tools/venv
.tools/venv/bin/python -m pip install clang-format==18.1.8
# 检查；需要格式修复时用 --fix
python3 tools/format_code.py --check
cmake --build build-release --target format-check
~~~

普通构建不会重写源码；格式工具仅处理 src/include/tests/foxglove_telemetry/src 的 C/C++，不改 Python、Shell、配置或第三方文件。每次项目修改在板端编译并运行相关验证。

<details>
<summary>独立 Foxglove / MCAP 工程</summary>

主程序使用共享内存快照和 Python 桥，无需此独立 SDK publisher。需要独立协议检查时：

~~~bash
cmake -S foxglove_telemetry -B build/foxglove_telemetry
cmake --build build/foxglove_telemetry -j2
./build/foxglove_telemetry/rescue_telemetry_smoke
~~~

构建真实 publisher 时增加 `-DRESCUE_BUILD_FOXGLOVE_PUBLISHER=ON -DFOXGLOVE_SDK_DIST=<Foxglove SDK cpp/dist>`，程序为 `rescue_foxglove_publisher`。该工程使用 C++20；只读订阅，不接受客户端控制。

~~~bash
python3 -m venv .venv-mcap
. .venv-mcap/bin/activate
pip install -r foxglove_telemetry/tools/requirements.txt
python3 foxglove_telemetry/tools/generate_sample_mcap.py rescue_sample.mcap
~~~

生成的 MCAP 可由兼容客户端回放。上游依赖为 Foxglove SDK、MCAP Python writer，代码与协议入口保留在 `foxglove_telemetry/`。

</details>

### 当前限制与历史证据

当前配置含假设的机构映射；相机地面标定已由用户确认完成；识别/几何资格、机构、急停、底盘、捕获搬运和完整交付仍需按现场条件验收。软件回归只证明测试输入下的行为。旧报告中的主机异常、吞吐量和实测记录是历史状态，不直接作为当前结论。

仅保留以下三份不可编辑的原始测试证据，内容可能描述历史协议；**当前使用只看本文**：

- [ARM 基线证据](tests/fixtures/doc_evidence/CAT_HOST_20261020.md)
- [历史实车验证证据](tests/fixtures/doc_evidence/CAT_VEHICLE_ACCEPTANCE_20261024.md)
- [历史协议快照](tests/fixtures/doc_evidence/通信协议.md)

冻结证据保留原始字节和历史链接；它们不是现行操作手册。基线补充文件只改证据位置，不重录预期输出。比赛资料 `intelligence20260617.pdf`、标定图案 PDF、测试夹具和标定数据保留。

2026-10-26 本次整理前文件及校验清单位于板端项目外 `/home/cat/ds_protocol15_docs_20261026_183706/`。已移出的旧标定记录位置由 `calibration_runs/README.archive.txt` 指示。后续操作说明集中更新本文；验证日志留在项目外，不再新增逐轮报告文档。
