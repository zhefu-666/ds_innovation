# 重构前 README 历史阶段摘录

来源：提交 bd73a38 的 README 对应段落，2026-10-19 在应用职责重构时归档。以下保留旧阶段说明，含已过时的状态名、入口、默认模型和控制接入判断；当前实现见 [当前架构](../ARCHITECTURE.md) 与 [运行配置](../RUNTIME_CONFIGURATION.md)。文中的测试和现场结论仅属于原阶段，不能视为本轮验收。

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

相机自动标定与人工测量步骤见 [标定工具说明](../../tools/camera_calibration/README.md)。
