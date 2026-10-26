# 新接手快速上手指南

> **适用范围提醒**：本文件记录 2026-10-24 的实测快照，当时车端使用隔离目录
> `/home/cat/ds_innovation-refactor-20261024`（现已不存在）与 `build-arm64-selfcontained/`。
> 当前分支入口与目录见 [运行与调试](运行与调试.md)；下文的目录名和测试计数都属于那次快照，不要当作当前值。

验证日期：2026-10-24。适用于本机 `/home/jy/ds_innovation` 与当时车端的对应快照。

## 1. 先看结论

**软件离线任务、ARM 构建、真实 RKNN 推理、相机/IMU/MCU 只读链路可以运行；整车运动和比赛流程尚未验收。**

本次实际操作结果：

| 检查 | 结果 |
|---|---|
| 本机 x86 Release、RKNN OFF、遥测 ON 构建 | 通过，目录 `build-onboarding-offline/` |
| 本机 CTest | **42/43 通过**；`rescue_runtime_legacy_baseline` 失败 |
| 车端 ARM64 Release、RKNN ON、遥测 ON 构建 | 通过，目录 `build-arm64-selfcontained/` |
| 车端 CTest | **42/43 通过**，同一冻结基线测试失败 |
| 无设备任务回放 | 本机、车端均 `rc=0`，走到捕获、搬运、交付核验，最终 `delivered=1` |
| 配置导出 | 成功导出合法 JSON，不初始化模型或打开设备 |
| Python 遥测桥接单测 | **9/9 通过** |
| 网页管理启动 → 状态查询 → 停止 | 通过；图像实时更新，`hardware_output_enabled=false` |
| 真实 split INT8 双模型样图推理 | `rc=0`，检测 9 个目标，解析左右半区及 6 个关键点 |
| 相机 + RKNN + IMU + MCU 只读实时预览 | 运行 20 秒，无崩溃；`--dry-run`，没有发送控制字节 |
| 短时实时性能 | 37 个日志样本，平均 `24.16 FPS`；后段约 `25.8 FPS`；非长期性能验收 |

当前主要边界：

1. **冻结基线未全通过。** 当前未提交修改将方框 A6 语义由 `done_flag` 改为
   `state_mapping`，两份资源哈希及两个静态检查输出随设计变化；旧补充基线没有覆盖这组差异。
   不应修改冻结预期来“变绿”，应先审查这些策略改动并建立独立、版本化的预期。
2. **实时几何未通过。** 当前 A6 为 RX120，即内部 `0 cdeg`；标定从内部 `+5°`
   开始，因此 `[GEOMETRY] reason=pitch_not_calibrated` 是正确的拒绝，不是程序崩溃。
3. **现场没有可评价的目标检测。** 本次实时预览 `detections=0`，不能据此判断识别准确率。
   样图推理只验证模型入口和输出，不验证现场识别、区域颜色或几何精度。
4. **方框/机械/现场参数仍含假设。** 不猜方框方向或编号，不以 `--assume-all-safe`
   代替现场安全证据。当前文件虽改为状态映射，物理开闭方向仍需单独验收。
5. 车端本次读取 EXT4 `errors_count=0`，内核日志未匹配到旧 EXT4 错误，编译器哈希也与历史可信值一致；
   **这不构成完整文件系统健康证明**。历史异常见 [验收记录](validation/CAT_VEHICLE_ACCEPTANCE_20261024.md)。

## 2. 环境、目录与版本意识

SSH 配置：

```sshconfig
Host cat
  HostName 10.63.153.7
  User cat
```

连接并进入车端项目：

```bash
ssh cat
```

```bash
cd /home/cat/ds_innovation
```

- 本机：x86，GCC 11.4、OpenCV 4.5.4，可进行无 RKNN 构建和离线测试。
- 车端：`lubancat`、ARM64/RK3588S，GCC 13.3、OpenCV 4.6、Python 3.12、`websockets 10.4`。
- 设备：相机 `/dev/video0`、MCU `/dev/ttyACM0`、IMU `/dev/ttyUSB0`。
- 模型与 `benchmark_results/librknnrt.so` 是 Git 忽略的运行资源，不能只克隆仓库就期待完整运行。
- 车端目标目录是**不含 `.git` 的部署快照**。不能用 `git pull` 更新，也不要将本机整个工作区无差别覆盖过去。
- 本机工作树有大量未提交改动和新文件，**不要 `git reset --hard`、`git clean` 或覆盖原文件**。
- 本指南不替换旧部署目录 `/home/cat/ds_innovation`，也不启用硬件整场入口。

## 3. 最短跑通路径

### 3.1 无设备任务回放

适合先理解任务状态机，不需要相机、模型或串口。

本机：

```bash
./build-onboarding-offline/rescue_upper_host --config config/runtime/push_replay.json
```

车端将二进制换成 `build-release/rescue_upper_host`（当时快照为 `build-arm64-selfcontained/`）。输出中的 `vx/wz` 是算法计算值，
**不是实际车速，也不表示命令已发送**。

正常结果：状态从搜索、接近、捕获，进入搬运、区域入口、交付核验，最终返回 `SCAN`，
包含 `delivered=1 first=1`，退出码 0。

本机需要重建时：

```bash
cmake -G Ninja -S . -B build-onboarding-offline -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DRESCUE_ENABLE_RKNN=OFF -DRESCUE_ENABLE_TELEMETRY=ON
```

```bash
cmake --build build-onboarding-offline -j4
```

```bash
ctest --test-dir build-onboarding-offline --output-on-failure
```

### 3.2 真实样图双模型推理

在车端项目根目录执行；不打开相机或串口。**单图模式不要附带 `--calibration`、
`--imu` 或 `--pitch-feedback`，这些组合会被参数校验拒绝。**

```bash
./build-arm64-selfcontained/rescue_upper_host --detect-image tests/samples/zone_20260930_212538_000400.jpg --model models/detect_int8_split_20261022.rknn --pose-model models/zone_pose_int8_split_20261022.rknn --rknn-library benchmark_results/librknnrt.so --parallel-infer --detect-core 1 --pose-core 2 --team blue
```

本次样图检出 9 个目标，`total_ms≈28.8`。`ZONE_COLOR=color_unmeasured` 表示此命令没有提供
颜色标定，不是模型加载失败。

### 3.3 网页/Foxglove 只读感知预览

先确认没有其他程序占用相机。管理脚本固定使用 `build-release/`，
所以两个 ARM 构建目录都应保持当前版本；本指南已重建此目录以包含 split 多输出修复。

```bash
python3 tools/remote_camera_telemetry/manage_project.py start --model models/detect_int8_split_20261022.rknn --pose-model models/zone_pose_int8_split_20261022.rknn --rknn-library benchmark_results/librknnrt.so --parallel-infer --detect-core 1 --pose-core 2 --vision-log-dir none --camera 0 --width 1280 --height 720 --fps 60
```

```bash
python3 tools/remote_camera_telemetry/manage_project.py status
```

浏览器打开 `http://10.63.153.7:8080/`；Foxglove 连接 `ws://10.63.153.7:8765`。
连接前确认电脑与板卡网络互通及防火墙允许相应端口。

```bash
python3 tools/remote_camera_telemetry/manage_project.py stop
```

管理脚本强制附加 `--dry-run --no-show --telemetry`，不需要也不应添加 `--hardware`。
遥测桥只读取 `/dev/shm/rescue-telemetry.bin`，不接受客户端运动控制请求。
若出现 `Expected one input and one output`，先重建 `build-release/`，不要换模型逃避问题。

## 4. 车端重建与静态检查

车端采用 Ninja，保留正式入口使用的 `build-release/`。在项目根目录执行：

```bash
cmake --preset cat-release
cmake --build --preset cat-release
ctest --preset cat-release
```

`cat-release` 继承共享 `release` 预设，默认 4 并行；板端本机路径由不入库的
`CMakeUserPresets.json` 提供。其他机器安装 Ninja 并配置依赖后使用 `release`。
预设要求 CMake 3.21+。用户目录 Ninja 不在 PATH 时，手动配置前执行
`export PATH="$HOME/.local/bin:$PATH"`。
从 Make 首次迁移需先备份构建目录，再在原路径重新配置；不能混用生成器缓存。

静态入口检查不会打开设备；方框未确认时只显示占位配置，不得将占位值用于运动：

```bash
tools/match/match_main.sh --check
```

`--status` 独占只读 MCU 串口，查看前应先停止预览和其他串口程序：

```bash
tools/match/match_main.sh --status
```

导出最终配置，不加载模型、不开设备：

```bash
./build-arm64-selfcontained/rescue_upper_host --config config/runtime/preview.json --dump-config /tmp/effective_config.json
```

## 5. 代码地图：一帧数据怎样走

```text
src/main.cpp
  -> loadAppConfig：JSON + 显式 CLI 请求
  -> resolveLegacyRuntimeConfig：标定、兼容策略和最终覆盖
  -> Runtime::run
       -> 相机/IMU/MCU 生命周期
       -> PerceptionPipeline：检测 + pose（可并行）
       -> Tracker + ObservationBuilder：几何、目标、区域、持有证据
       -> MatchControl / 预检 / 导航许可
       -> PushTask：任务状态机与动作决策
       -> MotionLink：仅 hardware 路径发送
       -> TelemetryPublisher -> 快照 -> 网页/Foxglove
```

| 位置 | 职责 |
|---|---|
| [src/main.cpp](../src/main.cpp) | 16 行进程入口，信号与异常处理 |
| [src/app/runtime.cpp](../src/app/runtime.cpp) | 设备装配、主循环、预检、输出生命周期 |
| [src/app/config_loader.cpp](../src/app/config_loader.cpp) | 严格 runtime JSON、请求来源和 CLI 优先级 |
| [src/app/runtime_config.cpp](../src/app/runtime_config.cpp) | 最终兼容参数、假设和导出 |
| [src/app/perception_pipeline.cpp](../src/app/perception_pipeline.cpp) | 检测/姿态模型、共享 letterbox、并行推理 |
| [src/perception/rknn_model.cpp](../src/perception/rknn_model.cpp)、[src/perception/rknn_output_layout.cpp](../src/perception/rknn_output_layout.cpp) | RKNN 后端及单/多输出布局归一化 |
| [src/app/observation_builder.cpp](../src/app/observation_builder.cpp) | 把感知与传感器变成任务观测 |
| [src/task/push_task.cpp](../src/task/push_task.cpp) | 顶层任务协调，仍保留较多历史策略 |
| `src/task/` | 已提取的捕获、交付、退区、推入和清扫阶段 |
| [include/rescue/task/push_task.hpp](../include/rescue/task/push_task.hpp) | `PushState`、观测与输出的核心契约 |
| [src/hardware/uart_controller.cpp](../src/hardware/uart_controller.cpp)、[src/hardware/motion_link.cpp](../src/hardware/motion_link.cpp) | A6 协议、反馈与运动发送链路 |
| `tests/` | 合成观测、协议、阶段、配置和端到端回放测试 |

正常搬运主链可概括为：

```text
SCAN -> APPROACH -> PREPARE -> RUSH -> LOWER_FRAME
     -> VERIFY_CAPTURE -> CARRY -> GATE -> ENTER
     -> RAISE_RELEASE -> BACK_OUT -> VERIFY_DELIVERY -> 下一趟
```

它不是“机械臂抓取”：方框打开、冲入包围目标、闭框搬运，到目标区域后开框释放。
真实决策仍受目标、几何、区域身份、路径、IMU、比赛许可及机构反馈门控。

更完整定义见 [当前架构](ARCHITECTURE.md) 与 [运行配置](RUNTIME_CONFIGURATION.md)。

## 6. 配置与安全模式

### 请求值不一定等于最终值

`--config` 的请求优先级是：默认值 → JSON → 显式 CLI；随后旧兼容策略和标定仍可能覆盖。
因此调参后要检查 `--dump-config` 中的 `request`、最终 `task`、`assumptions`、`field_sources`。
不要把 `legacy_field_v1` 理解成已经彻底消除了历史覆盖。

### 两类 profile 不要混淆

- `config/runtime/*.json`：主程序严格 runtime schema v1。
- `config/demo_20261024.assume.json`：演示脚本资源与设备 profile，**不是**主程序 `--config` 的 schema。

### 硬件边界

| 模式 | 设备行为 |
|---|---|
| 回放、单图、`--dump-config` | 不开实时设备 |
| `--dry-run` | 算法和相机可运行，不发送 MCU 控制 |
| `--dry-run --pitch-feedback` | 额外以 `O_RDONLY` 打开 MCU，只接收 A6；发送函数拒绝写入 |
| `--dry-run --imu` | 接收 IMU，不授予运动输出 |
| `--hardware` | 独占 MCU 读写串口，真正可能发送底盘、方框、相机命令 |

`--hardware` 还必须配合 IMU，不能与 dry-run、pitch-feedback 或回放组合。
`hardware_output_enabled=0` 时遥测速度只是计算值。

## 7. 建议的接手顺序

1. 先跑无设备回放，熟悉状态与 `reason`，再阅读 `PushObservation` 和 `PushTask`。
2. 用样图验证检测/姿态模型；修改感知代码时同时跑串行、并行对照。
3. 用网页只读预览检查现场目标、几何拒绝原因和传感器新鲜度。
4. 完成当前 RX120 姿态的真实标定，并现场确认方框物理状态，不直接扩大标定范围绕过门控。
5. 审查并补齐当前 A6 状态映射的版本化基线，恢复完整 CTest。
6. 在清场、急停可用、有人值守且现场确认后，才分别进行单机构、单俯仰、最小底盘动作验收。
7. 最后才是单次搬运和 180 秒整场；`match_main.sh --run` 不是新手默认启动命令。

## 8. 本次同步与留痕

- 本地与车端受版本控制的源文件做了逐文件 SHA-256 核对。
- 唯一模型清单差异 `models/zone_pose_red_v2_manifest.json` 已同步为本地版本；
  车端原件保留在 `onboarding-backup/zone_pose_red_v2_manifest.before-onboarding.json`。
- 未改动本机任务代码、配置或冻结基线；未提交、未清理任何既有修改。
- 车端 `build-arm64-selfcontained/`、`build-release/` 已按当前源码重建。
- 预览验证结束后停止管理进程，不保留后台占用相机或串口的服务。
