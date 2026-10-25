# 代码目录布局

项目的实现代码按职责分层，`src/` 与 `include/rescue/` 使用对应的目录结构。新增文件应放入职责最接近的目录，避免重新堆回顶层。

| 目录 | 职责 |
| --- | --- |
| `src/main.cpp` | 唯一进程入口、信号处理 |
| `src/app/`、`include/rescue/app/` | 应用装配、运行时生命周期、配置加载、观测组装、离线任务和诊断 |
| `src/task/`、`include/rescue/task/` | PushTask 状态机、运输规则和各任务阶段 |
| `src/perception/`、`include/rescue/perception/` | 检测器、RKNN/模型输入输出、跟踪、像素选择、视觉区域与目标识别 |
| `src/geometry/`、`include/rescue/geometry/` | 相机标定、地面映射、安全区几何、导航和规划 |
| `src/hardware/`、`include/rescue/hardware/` | 相机、MCU/UART、IMU、传感器融合和 MotionLink |
| `src/control/`、`include/rescue/control/` | 比赛控制、运动许可、受控场和控制服务 |
| `src/config/`、`include/rescue/config/` | 启动配置、任务标定和演示策略 |
| `src/core/`、`include/rescue/core/` | 基础类型、日志、异步日志、审计和通用工具 |
| `src/telemetry/`、`include/rescue/telemetry/` | Foxglove/遥测发布 |
| `src/tools/` | 独立的传感器和 IMU 监视器可执行文件 |

CMake 中的源文件路径与上述物理目录保持一致；头文件 include 也使用完整职责路径，例如：

```cpp
#include "rescue/perception/detector.hpp"
#include "rescue/geometry/zone_estimate.hpp"
#include "rescue/hardware/uart_controller.hpp"
#include "rescue/task/push_task.hpp"
```

目录调整只改变文件组织和引用路径，不改变运行时状态机、协议和算法行为。
